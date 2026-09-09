#include "simulation_core.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include "convex_mpc/utils.hpp"

namespace ConvexMPC {
namespace {

constexpr double kControlDt = 0.001;
constexpr double kMpcDt = 0.02;
constexpr size_t kMpcHorizon = 16;
constexpr double kStanceDuration = 0.2;
constexpr double kSwingDuration = 0.2;
constexpr double kSettleDuration = 0.7;
constexpr double kLinearCommandSlewRate = 0.6;
constexpr double kAngularCommandSlewRate = 1.5;

const char* kFootSites[LEG_NUM] = {"LF_FOOT", "RF_FOOT", "LH_FOOT", "RH_FOOT"};

}  // namespace

SimulationCore::SimulationCore(const std::string& xml_path, bool stand_only) {
    char error[1000] = "";
    model_ = mj_loadXML(xml_path.c_str(), nullptr, error, sizeof(error));
    if (!model_) throw std::runtime_error("failed to load " + xml_path + ": " + error);
    model_->opt.timestep = kControlDt;
    data_ = mj_makeData(model_);
    if (!data_) throw std::runtime_error("failed to allocate MuJoCo data");

    const int standing_key = mj_name2id(model_, mjOBJ_KEY, "standing");
    if (standing_key < 0) throw std::runtime_error("missing standing keyframe");
    mj_resetDataKeyframe(model_, data_, standing_key);
    mj_forward(model_, data_);

    base_body_id_ = mj_name2id(model_, mjOBJ_BODY, "base");
    if (base_body_id_ < 0) throw std::runtime_error("missing base body");
    for (int leg = 0; leg < LEG_NUM; ++leg) {
        foot_site_ids_[leg] = mj_name2id(model_, mjOBJ_SITE, kFootSites[leg]);
        if (foot_site_ids_[leg] < 0) throw std::runtime_error(std::string("missing site ") + kFootSites[leg]);
    }

    settle_steps_ = long(kSettleDuration / kControlDt);
    mpc_every_ = int(kMpcDt / kControlDt + 0.5);
    settle_qpos_ = Eigen::Map<Eigen::VectorXd>(data_->qpos, model_->nq);

    const double mass = model_->body_subtreemass[base_body_id_];
    RobotModel robot_model(compute_composite_inertia(), mass, params::gravity, params::mu, kMpcDt, params::f_min, 700.0);
    const Eigen::Vector3d init_position(data_->qpos);
    const Eigen::Vector4d init_quaternion(data_->qpos + 3);
    Eigen::Vector3d nominal_position = init_position;
    nominal_position[2] = 0.5;
    std::array<Eigen::Vector3d, LEG_NUM> nominal_feet;
    for (int leg = 0; leg < LEG_NUM; ++leg)
        nominal_feet[leg] = Eigen::Vector3d(data_->site_xpos + 3 * foot_site_ids_[leg]) - init_position;
    RobotState nominal_state(utils::quaternion_to_euler(init_quaternion),
                             nominal_position,
                             Eigen::Vector3d::Zero(),
                             Eigen::Vector3d::Zero(),
                             nominal_feet);

    base_q_weights_ << 50, 150, 30, 25, 25, 100, 1, 3, 2, 30, 30, 10, 0;
    base_r_weights_ << 3e-4, 3e-4, 1e-4, 3e-4, 3e-4, 1e-4, 3e-4, 3e-4, 1e-4, 3e-4, 3e-4, 1e-4;
    gait_q_weights_ = base_q_weights_;
    gait_r_weights_ = base_r_weights_;
    controller_ = std::make_unique<RobotController>(robot_model,
                                                    nominal_state,
                                                    base_q_weights_,
                                                    base_r_weights_,
                                                    500.0,
                                                    40.0,
                                                    kMpcHorizon,
                                                    kControlDt,
                                                    stand_only ? 1e9 : kStanceDuration,
                                                    kSwingDuration);
    robot_state_ = nominal_state;
    robot_state_.updateContactState(stand_only ? std::array<bool, LEG_NUM>{true, true, true, true}
                                               : std::array<bool, LEG_NUM>{true, false, false, true});
}

SimulationCore::~SimulationCore() {
    if (data_) mj_deleteData(data_);
    if (model_) mj_deleteModel(model_);
}

void SimulationCore::set_velocity_command(const Eigen::Vector3d& command) { command_target_ = command; }

void SimulationCore::set_body_height(double height) { controller_->set_nominal_height(std::clamp(height, 0.35, 0.75)); }

void SimulationCore::set_mpc_tuning(double position_scale, double velocity_scale, double force_scale) {
    Eigen::Vector<double, MPC_STATE_DIM> q_weights = gait_q_weights_;
    Eigen::Vector<double, MPC_INPUT_DIM> r_weights = gait_r_weights_;
    q_weights.segment<3>(3) *= std::clamp(position_scale, 0.1, 5.0);
    q_weights.segment<3>(9) *= std::clamp(velocity_scale, 0.1, 5.0);
    r_weights *= std::clamp(force_scale, 0.1, 5.0);
    controller_->set_mpc_weights(q_weights, r_weights);
}

void SimulationCore::step(int count) {
    for (int index = 0; index < count; ++index) control_and_step();
}

void SimulationCore::update_command() {
    command_target_[0] = std::clamp(command_target_[0], -1.2, 1.2);
    command_target_[1] = std::clamp(command_target_[1], -max_linear_speed_ * 0.5, max_linear_speed_ * 0.5);
    command_target_[2] = std::clamp(command_target_[2], -max_yaw_rate_, max_yaw_rate_);
    for (int axis = 0; axis < 3; ++axis) {
        const double rate = (axis == 2 ? kAngularCommandSlewRate : kLinearCommandSlewRate) * kControlDt;
        command_[axis] += std::clamp(command_target_[axis] - command_[axis], -rate, rate);
    }
}

Eigen::Matrix3d SimulationCore::compute_composite_inertia() const {
    const Eigen::Vector3d com(data_->subtree_com + 3 * base_body_id_);
    Eigen::Matrix3d inertia = Eigen::Matrix3d::Zero();
    for (int body = base_body_id_; body < model_->nbody; ++body) {
        const double mass = model_->body_mass[body];
        if (mass <= 0) continue;
        const Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> rotation(data_->ximat + 9 * body);
        const Eigen::Vector3d principal_inertia(model_->body_inertia + 3 * body);
        const Eigen::Vector3d offset = Eigen::Vector3d(data_->xipos + 3 * body) - com;
        inertia += rotation * principal_inertia.asDiagonal() * rotation.transpose() +
                   mass * (offset.dot(offset) * Eigen::Matrix3d::Identity() - offset * offset.transpose());
    }
    return inertia;
}

void SimulationCore::control_and_step() {
    const Eigen::Vector3d position(data_->qpos);
    const Eigen::Vector4d quaternion(data_->qpos + 3);
    const Eigen::Matrix3d rotation =
        Eigen::Quaterniond(quaternion[0], quaternion[1], quaternion[2], quaternion[3]).toRotationMatrix();
    Eigen::Vector3d euler = utils::quaternion_to_euler(quaternion);
    if (!yaw_initialized_) {
        yaw_continuous_ = euler[2];
        yaw_initialized_ = true;
    }
    yaw_continuous_ = utils::unwrap_angle(yaw_continuous_, euler[2]);
    euler[2] = yaw_continuous_;
    robot_state_.updateState(euler, position, rotation * Eigen::Vector3d(data_->qvel + 3), Eigen::Vector3d(data_->qvel));

    std::array<Eigen::Vector3d, LEG_NUM> feet, foot_velocities;
    std::array<Eigen::Matrix3d, LEG_NUM> jacobians;
    for (int leg = 0; leg < LEG_NUM; ++leg) {
        feet[leg] = Eigen::Vector3d(data_->site_xpos + 3 * foot_site_ids_[leg]) - position;
        Eigen::Matrix<mjtNum, 3, Eigen::Dynamic, Eigen::RowMajor> jacobian(3, model_->nv);
        mj_jacSite(model_, data_, jacobian.data(), nullptr, foot_site_ids_[leg]);
        jacobians[leg] = jacobian.block<3, 3>(0, 6 + 3 * leg);
        foot_velocities[leg] = jacobians[leg] * Eigen::Vector3d(data_->qvel + 6 + 3 * leg);
    }
    robot_state_.updateFootPosition(feet);
    robot_state_.updateFootVelocity(foot_velocities);

    if (step_count_ < settle_steps_) {
        for (int joint = 0; joint < 12; ++joint)
            data_->ctrl[joint] = 100.0 * (settle_qpos_[7 + joint] - data_->qpos[7 + joint]) - 5.0 * data_->qvel[6 + joint] +
                                 data_->qfrc_bias[6 + joint];
        mj_step(model_, data_);
        ++step_count_;
        return;
    }

    update_command();
    if (step_count_ % mpc_every_ == 0) ground_reaction_forces_ = controller_->computeGRF(robot_state_, command_);
    const auto swing_forces = controller_->computeSwingForce(robot_state_, command_);
    for (int leg = 0; leg < LEG_NUM; ++leg) {
        Eigen::Vector3d torque = robot_state_.contact_state(leg)
                                     ? Eigen::Vector3d(-jacobians[leg].transpose() * ground_reaction_forces_[leg])
                                     : Eigen::Vector3d(jacobians[leg].transpose() * swing_forces[leg]);
        torque += Eigen::Vector3d(data_->qfrc_bias + 6 + 3 * leg);
        for (int joint = 0; joint < 3; ++joint) data_->ctrl[3 * leg + joint] = torque[joint];
    }
    mj_step(model_, data_);
    ++step_count_;
}

SimulationSnapshot SimulationCore::snapshot() const {
    SimulationSnapshot result;
    result.time = data_->time;
    for (int axis = 0; axis < 3; ++axis) result.position[axis] = data_->qpos[axis];
    for (int axis = 0; axis < 4; ++axis) result.quaternion[axis] = data_->qpos[3 + axis];
    const Eigen::Vector4d quaternion(data_->qpos + 3);
    const Eigen::Vector3d euler = utils::quaternion_to_euler(quaternion);
    for (int axis = 0; axis < 3; ++axis) result.euler[axis] = euler[axis];
    for (int leg = 0; leg < LEG_NUM; ++leg) {
        for (int axis = 0; axis < 3; ++axis) {
            result.foot_positions[3 * leg + axis] = data_->site_xpos[3 * foot_site_ids_[leg] + axis];
            result.ground_reaction_forces[3 * leg + axis] = ground_reaction_forces_[leg][axis];
        }
    }
    return result;
}

std::vector<VisualGeometrySnapshot> SimulationCore::visual_geometries() const {
    std::vector<VisualGeometrySnapshot> result;
    for (int geometry = 0; geometry < model_->ngeom; ++geometry) {
        if (model_->geom_type[geometry] != mjGEOM_MESH || model_->geom_group[geometry] != 2) continue;
        const char* mesh_name = mj_id2name(model_, mjOBJ_MESH, model_->geom_dataid[geometry]);
        if (!mesh_name) continue;

        const int mesh_id = model_->geom_dataid[geometry];
        const Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> geometry_rotation(data_->geom_xmat + 9 * geometry);
        mjtNum mesh_rotation_data[9];
        mju_quat2Mat(mesh_rotation_data, model_->mesh_quat + 4 * mesh_id);
        const Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> mesh_rotation(mesh_rotation_data);
        const Eigen::Vector3d mesh_position(model_->mesh_pos + 3 * mesh_id);
        const Eigen::Matrix3d inverse_mesh_rotation = mesh_rotation.transpose();
        const Eigen::Vector3d inverse_mesh_position = -inverse_mesh_rotation * mesh_position;

        VisualGeometrySnapshot visual;
        visual.mesh_name = mesh_name;
        const Eigen::Vector3d position =
            Eigen::Vector3d(data_->geom_xpos + 3 * geometry) + geometry_rotation * inverse_mesh_position;
        const Eigen::Matrix3d rotation = geometry_rotation * inverse_mesh_rotation;
        for (int axis = 0; axis < 3; ++axis) {
            visual.position[axis] = position[axis];
            visual.scale[axis] = model_->mesh_scale[3 * mesh_id + axis];
        }
        for (int row = 0; row < 3; ++row)
            for (int column = 0; column < 3; ++column) visual.rotation[3 * row + column] = rotation(row, column);
        result.push_back(std::move(visual));
    }
    return result;
}

}  // namespace ConvexMPC
