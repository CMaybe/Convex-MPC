#include <emscripten/bind.h>
#include <emscripten/val.h>

#include <array>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Dense>
#ifdef CONVEX_MPC_WITH_MUJOCO
#include <mujoco/mujoco.h>
#include "simulation_core.hpp"
#endif

#include "convex_mpc/params.hpp"
#include "convex_mpc/robot_controller.hpp"
#include "convex_mpc/robot_model.hpp"
#include "convex_mpc/robot_state.hpp"

namespace {

using namespace ConvexMPC;

#ifdef CONVEX_MPC_WITH_MUJOCO
std::string mujoco_version() { return mj_versionString(); }

class BrowserSimulation {
public:
    BrowserSimulation() { reset(); }

    void reset() {
        try {
            simulation_ = std::make_unique<SimulationCore>("/rsc/anymal_c/scene.xml");
        } catch (const std::exception& error) {
            emscripten::val::global("Error").new_(std::string(error.what())).throw_();
        }
    }

    void step(double vx, double vy, double wz, int count) {
        simulation_->set_velocity_command(Eigen::Vector3d(vx, vy, wz));
        simulation_->step(count);
    }

    void set_body_height(double height) { simulation_->set_body_height(height); }

    void set_mpc_tuning(double position_scale, double velocity_scale, double force_scale) {
        simulation_->set_mpc_tuning(position_scale, velocity_scale, force_scale);
    }

    emscripten::val snapshot() const {
        const SimulationSnapshot state = simulation_->snapshot();
        emscripten::val result = emscripten::val::object();
        result.set("time", state.time);
        result.set("position", to_js_array(state.position));
        result.set("quaternion", to_js_array(state.quaternion));
        result.set("footPositions", to_js_array(state.foot_positions));
        result.set("groundReactionForces", to_js_array(state.ground_reaction_forces));
        return result;
    }

    emscripten::val visual_geometries() const {
        emscripten::val result = emscripten::val::array();
        const auto visuals = simulation_->visual_geometries();
        for (unsigned index = 0; index < visuals.size(); ++index) {
            emscripten::val visual = emscripten::val::object();
            visual.set("meshName", visuals[index].mesh_name);
            visual.set("position", to_js_array(visuals[index].position));
            visual.set("rotation", to_js_array(visuals[index].rotation));
            visual.set("scale", to_js_array(visuals[index].scale));
            result.set(index, visual);
        }
        return result;
    }

private:
    template <size_t Size>
    static emscripten::val to_js_array(const std::array<double, Size>& values) {
        emscripten::val result = emscripten::val::array();
        for (unsigned index = 0; index < values.size(); ++index) result.set(index, values[index]);
        return result;
    }

    std::unique_ptr<SimulationCore> simulation_;
};
#endif

class MpcPreview {
public:
    MpcPreview()
        : robot_model_(default_inertia(), 44.965, params::gravity, params::mu, 0.02, params::f_min, 700.0)
        , nominal_state_(default_euler(), default_position(), zero_vector(), zero_vector(), default_feet())
        , q_weights_(default_q_weights())
        , r_weights_(default_r_weights())
        , controller_(robot_model_, nominal_state_, q_weights_, r_weights_, 500.0, 40.0, 16, 0.001, 0.2, 0.2) {
        controller_.resetGaitPhase({0.0, 0.0, 0.0, 0.0});
    }

    emscripten::val step(const emscripten::val& state,
                         const emscripten::val& foot_positions,
                         const emscripten::val& foot_velocities,
                         const emscripten::val& command,
                         const emscripten::val& contacts) {
        const std::vector<double> state_values = read_doubles(state, MPC_STATE_DIM, "state");
        const std::vector<double> foot_values = read_doubles(foot_positions, LEG_NUM * 3, "foot_positions");
        const std::vector<double> foot_velocity_values = read_doubles(foot_velocities, LEG_NUM * 3, "foot_velocities");
        const std::vector<double> command_values = read_doubles(command, 3, "command");
        const std::vector<int> contact_values = read_ints(contacts, LEG_NUM, "contacts");

        const Eigen::Map<const Eigen::Vector<double, MPC_STATE_DIM>> state_map(state_values.data());
        const Eigen::Map<const Eigen::Vector3d> command_map(command_values.data());
        Eigen::Vector<double, MPC_STATE_DIM> state_copy = state_map;
        Eigen::Vector3d euler = state_copy.segment<3>(0);
        Eigen::Vector3d position = state_copy.segment<3>(3);
        Eigen::Vector3d angular_velocity = state_copy.segment<3>(6);
        Eigen::Vector3d linear_velocity = state_copy.segment<3>(9);

        std::array<Eigen::Vector3d, LEG_NUM> feet;
        std::array<Eigen::Vector3d, LEG_NUM> foot_velocities_w;
        for (int leg = 0; leg < LEG_NUM; ++leg) feet[leg] = Eigen::Map<const Eigen::Vector3d>(foot_values.data() + leg * 3);
        for (int leg = 0; leg < LEG_NUM; ++leg)
            foot_velocities_w[leg] = Eigen::Map<const Eigen::Vector3d>(foot_velocity_values.data() + leg * 3);

        RobotState state_model(euler, position, angular_velocity, linear_velocity, feet);
        std::array<bool, LEG_NUM> contact_state{};
        for (int leg = 0; leg < LEG_NUM; ++leg) contact_state[leg] = contact_values[leg] != 0;
        state_model.updateContactState(contact_state);

        state_model.updateFootVelocity(foot_velocities_w);

        const auto grf = controller_.computeGRF(state_model, command_map);
        const auto swing_forces = controller_.computeSwingForce(state_model, command_map);
        last_grf_.clear();
        last_grf_.reserve(LEG_NUM * 3);
        for (const auto& force : grf)
            for (int axis = 0; axis < 3; ++axis) last_grf_.push_back(force[axis]);

        last_swing_forces_.clear();
        last_swing_forces_.reserve(LEG_NUM * 3);
        for (const auto& force : swing_forces)
            for (int axis = 0; axis < 3; ++axis) last_swing_forces_.push_back(force[axis]);

        last_contacts_.clear();
        last_contacts_.reserve(LEG_NUM);
        for (const bool contact : state_model.contact_state()) last_contacts_.push_back(contact ? 1 : 0);

        const auto& prediction = controller_.mpc_result();
        last_prediction_.assign(prediction.data(), prediction.data() + prediction.size());
        return to_js_array(last_grf_);
    }

    emscripten::val prediction() const { return to_js_array(last_prediction_); }
    emscripten::val swing_forces() const { return to_js_array(last_swing_forces_); }
    emscripten::val contacts() const { return to_js_array(last_contacts_); }

    emscripten::val swing_step(const emscripten::val& state,
                               const emscripten::val& foot_positions,
                               const emscripten::val& foot_velocities,
                               const emscripten::val& command,
                               const emscripten::val& contacts) {
        const std::vector<double> state_values = read_doubles(state, MPC_STATE_DIM, "state");
        const std::vector<double> foot_values = read_doubles(foot_positions, LEG_NUM * 3, "foot_positions");
        const std::vector<double> velocity_values = read_doubles(foot_velocities, LEG_NUM * 3, "foot_velocities");
        const std::vector<double> command_values = read_doubles(command, 3, "command");
        const std::vector<int> contact_values = read_ints(contacts, LEG_NUM, "contacts");

        const Eigen::Map<const Eigen::Vector<double, MPC_STATE_DIM>> state_map(state_values.data());
        const Eigen::Map<const Eigen::Vector3d> command_map(command_values.data());
        std::array<Eigen::Vector3d, LEG_NUM> feet, velocities;
        std::array<bool, LEG_NUM> contact_state{};
        for (int leg = 0; leg < LEG_NUM; ++leg) {
            feet[leg] = Eigen::Map<const Eigen::Vector3d>(foot_values.data() + leg * 3);
            velocities[leg] = Eigen::Map<const Eigen::Vector3d>(velocity_values.data() + leg * 3);
            contact_state[leg] = contact_values[leg] != 0;
        }

        RobotState state_model(
            state_map.segment<3>(0), state_map.segment<3>(3), state_map.segment<3>(6), state_map.segment<3>(9), feet);
        state_model.updateFootVelocity(velocities);
        state_model.updateContactState(contact_state);
        const auto swing_forces = controller_.computeSwingForce(state_model, command_map);
        last_swing_forces_.clear();
        last_contacts_.clear();
        for (const auto& force : swing_forces)
            for (int axis = 0; axis < 3; ++axis) last_swing_forces_.push_back(force[axis]);
        for (const bool contact : state_model.contact_state()) last_contacts_.push_back(contact ? 1 : 0);
        return to_js_array(last_swing_forces_);
    }

private:
    static std::vector<double> read_doubles(const emscripten::val& values, size_t expected, const char* name) {
        const unsigned length = values["length"].as<unsigned>();
        if (length != expected) throw std::invalid_argument(std::string(name) + " has an invalid size");
        std::vector<double> result(expected);
        for (unsigned index = 0; index < length; ++index) result[index] = values[index].as<double>();
        return result;
    }

    static std::vector<int> read_ints(const emscripten::val& values, size_t expected, const char* name) {
        const unsigned length = values["length"].as<unsigned>();
        if (length != expected) throw std::invalid_argument(std::string(name) + " has an invalid size");
        std::vector<int> result(expected);
        for (unsigned index = 0; index < length; ++index) result[index] = values[index].as<int>();
        return result;
    }

    static emscripten::val to_js_array(const std::vector<double>& values) {
        emscripten::val result = emscripten::val::array();
        for (unsigned index = 0; index < values.size(); ++index) result.set(index, values[index]);
        return result;
    }

    static emscripten::val to_js_array(const std::vector<int>& values) {
        emscripten::val result = emscripten::val::array();
        for (unsigned index = 0; index < values.size(); ++index) result.set(index, values[index]);
        return result;
    }

    static Eigen::Matrix3d default_inertia() {
        Eigen::Matrix3d inertia = Eigen::Matrix3d::Zero();
        inertia.diagonal() << 2.0117, 3.8337, 3.9700;
        return inertia;
    }

    static Eigen::Vector3d default_euler() { return Eigen::Vector3d::Zero(); }
    static Eigen::Vector3d default_position() { return Eigen::Vector3d(0.0, 0.0, 0.54); }
    static Eigen::Vector3d zero_vector() { return Eigen::Vector3d::Zero(); }

    static std::array<Eigen::Vector3d, LEG_NUM> default_feet() {
        return {Eigen::Vector3d(0.35, -0.20, -0.54),
                Eigen::Vector3d(0.35, 0.20, -0.54),
                Eigen::Vector3d(-0.35, -0.20, -0.54),
                Eigen::Vector3d(-0.35, 0.20, -0.54)};
    }

    static Eigen::Vector<double, MPC_STATE_DIM> default_q_weights() {
        Eigen::Vector<double, MPC_STATE_DIM> weights;
        weights << 50, 150, 30, 25, 25, 100, 1, 3, 2, 30, 30, 10, 0;
        return weights;
    }

    static Eigen::Vector<double, MPC_INPUT_DIM> default_r_weights() {
        Eigen::Vector<double, MPC_INPUT_DIM> weights;
        weights << 3e-4, 3e-4, 1e-4, 3e-4, 3e-4, 1e-4, 3e-4, 3e-4, 1e-4, 3e-4, 3e-4, 1e-4;
        return weights;
    }

    RobotModel robot_model_;
    RobotState nominal_state_;
    Eigen::Vector<double, MPC_STATE_DIM> q_weights_;
    Eigen::Vector<double, MPC_INPUT_DIM> r_weights_;
    RobotController controller_;
    std::vector<double> last_grf_;
    std::vector<double> last_swing_forces_;
    std::vector<int> last_contacts_;
    std::vector<double> last_prediction_;
};

}  // namespace

EMSCRIPTEN_BINDINGS(convex_mpc) {
#ifdef CONVEX_MPC_WITH_MUJOCO
    emscripten::function("mujocoVersion", &mujoco_version);
    emscripten::class_<BrowserSimulation>("SimulationCore")
        .constructor<>()
        .function("reset", &BrowserSimulation::reset)
        .function("step", &BrowserSimulation::step)
        .function("setBodyHeight", &BrowserSimulation::set_body_height)
        .function("setMpcTuning", &BrowserSimulation::set_mpc_tuning)
        .function("snapshot", &BrowserSimulation::snapshot)
        .function("visualGeometries", &BrowserSimulation::visual_geometries);
#endif
    emscripten::class_<MpcPreview>("MpcPreview")
        .constructor<>()
        .function("step", &MpcPreview::step)
        .function("prediction", &MpcPreview::prediction)
        .function("swingForces", &MpcPreview::swing_forces)
        .function("contacts", &MpcPreview::contacts)
        .function("swingStep", &MpcPreview::swing_step);
}
