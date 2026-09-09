#include "convex_mpc/robot_controller.hpp"

#include <algorithm>
#include <cmath>
namespace ConvexMPC {
RobotController::RobotController(const RobotModel& robot_model,
                                 const Eigen::Ref<const Eigen::Vector<double, MPC_STATE_DIM>>& q_weights,
                                 const Eigen::Ref<const Eigen::Vector<double, MPC_INPUT_DIM>>& r_weights,
                                 const double& kp,
                                 const double& kd,
                                 const size_t& mpc_horizon,
                                 const double& control_dt,
                                 const double& stance_duration,
                                 const double& swing_duration)
    : robot_model_(robot_model)
    , q_weights_(q_weights)
    , r_weights_(r_weights)
    , Kp_(Eigen::Matrix3d::Identity() * kp)
    , Kd_(Eigen::Matrix3d::Identity() * kd)
    , mpc_horizon_(mpc_horizon)
    , control_dt_(control_dt)
    , stance_duration_(stance_duration)
    , swing_duration_(swing_duration) {
    constraint_coefficient_.setZero(MPC_CONSTRAINT_DIM, MPC_INPUT_DIM);
    Eigen::MatrixXd contraints(5, 3);
    // clang-format off
    contraints <<
    1, 0, robot_model.mu(),
    1, 0, -robot_model.mu(),
    0, 1, robot_model.mu(),
    0, 1, -robot_model.mu(),
    0, 0, 1;
    // clang-format on

    for (int leg_idx = 0; leg_idx < LEG_NUM; leg_idx++) {
        constraint_coefficient_.block<5, 3>(5 * leg_idx, 3 * leg_idx) = contraints;
    }
    mpc_result_.setZero();
}

RobotController::RobotController(const RobotModel& robot_model,
                                 const RobotState& nominal_state,
                                 const Eigen::Ref<const Eigen::Vector<double, MPC_STATE_DIM>>& q_weights,
                                 const Eigen::Ref<const Eigen::Vector<double, MPC_INPUT_DIM>>& r_weights,
                                 const double& kp,
                                 const double& kd,
                                 const size_t& mpc_horizon,
                                 const double& control_dt,
                                 const double& stance_duration,
                                 const double& swing_duration)
    : robot_model_(robot_model)
    , robot_nominal_state_(nominal_state)
    , q_weights_(q_weights)
    , r_weights_(r_weights)
    , Kp_(Eigen::Matrix3d::Identity() * kp)
    , Kd_(Eigen::Matrix3d::Identity() * kd)
    , mpc_horizon_(mpc_horizon)
    , control_dt_(control_dt)
    , stance_duration_(stance_duration)
    , swing_duration_(swing_duration) {
    constraint_coefficient_.setZero(MPC_CONSTRAINT_DIM, MPC_INPUT_DIM);
    Eigen::MatrixXd contraints(5, 3);
    // clang-format off
    contraints <<
    1, 0, robot_model.mu(),
    1, 0, -robot_model.mu(),
    0, 1, robot_model.mu(),
    0, 1, -robot_model.mu(),
    0, 0, 1;
    // clang-format on
    for (int leg_idx = 0; leg_idx < LEG_NUM; leg_idx++) {
        constraint_coefficient_.block<5, 3>(5 * leg_idx, 3 * leg_idx) = contraints;
    }
    mpc_result_.setZero();
}

void RobotController::resetGaitPhase(const std::array<double, LEG_NUM>& stance_elapsed) {
    stance_counter_ = stance_elapsed;
    swing_counter_ = {};
}

void RobotController::configure_gait(double stance_duration,
                                     double swing_duration,
                                     const std::array<bool, LEG_NUM>& contact_state,
                                     const std::array<double, LEG_NUM>& stance_elapsed,
                                     const std::array<double, LEG_NUM>& swing_elapsed,
                                     size_t mpc_horizon,
                                     double swing_height,
                                     double foothold_velocity_error_gain,
                                     double pitch_reference) {
    stance_duration_ = stance_duration;
    swing_duration_ = swing_duration;
    mpc_horizon_ = mpc_horizon;
    swing_height_ = swing_height;
    foothold_velocity_error_gain_ = foothold_velocity_error_gain;
    pitch_reference_ = pitch_reference;
    stance_counter_ = stance_elapsed;
    swing_counter_ = swing_elapsed;
    for (int leg = 0; leg < LEG_NUM; ++leg) {
        if (!contact_state[leg]) swing_start_position_[leg] = robot_nominal_state_.foot_positions_abs(leg);
    }
}

double RobotController::predictCyclePosition(const RobotState& robot_state,
                                             const size_t& leg_idx,
                                             const double& time_ahead) const {
    const double cycle_duration = stance_duration_ + swing_duration_;
    // Phase timers are zeroed on every phase transition, so the elapsed time within
    // the current phase is stance_counter_ (in stance) or swing_counter_ (in swing).
    double cycle_position = robot_state.contact_state(leg_idx) ? stance_counter_[leg_idx] + time_ahead
                                                               : stance_duration_ + swing_counter_[leg_idx] + time_ahead;
    return std::fmod(cycle_position, cycle_duration);
}

bool RobotController::predictContactState(const RobotState& robot_state, const size_t& leg_idx, const double& time_ahead) const {
    return predictCyclePosition(robot_state, leg_idx, time_ahead) < stance_duration_;
}

std::array<Eigen::Vector3d, LEG_NUM> RobotController::computeGRF(RobotState& robot_state,
                                                                 const Eigen::Ref<const Eigen::Vector3d>& cmd_vel_b) {
    std::array<Eigen::Vector3d, LEG_NUM> grf;
    double mpc_dt = robot_model_.dt();

    // Advance stance timers (swing timers advance in computeSwingForce at control rate).
    for (int leg_idx = 0; leg_idx < LEG_NUM; leg_idx++) {
        if (robot_state.contact_state(leg_idx)) {
            stance_counter_[leg_idx] += mpc_dt;
            if (stance_counter_[leg_idx] >= stance_duration_) {
                stance_counter_[leg_idx] = 0;
                robot_state.updateContactState(leg_idx, false);
            }
        }
    }

    // Reference trajectory (Di Carlo et al., Sec. IV-B): non-zero xy-velocity, xy-position,
    // z position, yaw and yaw rate; all other states are zero. Yaw and xy-position are
    // determined by integrating the commanded velocities (with a saturation band around
    // the measured state so the reference is regenerated after large disturbances).
    // The k-th block is the reference for x_{k+1}, i.e. (k+1) MPC steps ahead.
    Eigen::Vector3d euler = robot_state.euler_angle();
    Eigen::Vector3d position = robot_state.position();
    Eigen::Vector3d linear_velocity_w = robot_state.Rz() * Eigen::Vector3d{cmd_vel_b[0], cmd_vel_b[1], 0};
    if (!reference_initialized_) {
        yaw_ref_ = euler[2];
        position_ref_ = position.head<2>();
        reference_initialized_ = true;
    }
    yaw_ref_ += cmd_vel_b[2] * mpc_dt;
    position_ref_ += linear_velocity_w.head<2>() * mpc_dt;
    constexpr double kMaxYawError = 0.3;       // [rad]
    constexpr double kMaxPositionError = 0.2;  // [m]
    yaw_ref_ = std::clamp(yaw_ref_, euler[2] - kMaxYawError, euler[2] + kMaxYawError);
    for (int axis = 0; axis < 2; axis++)
        position_ref_[axis] =
            std::clamp(position_ref_[axis], position[axis] - kMaxPositionError, position[axis] + kMaxPositionError);
    const double z = robot_nominal_state_.position()[2];
    Eigen::VectorXd mpc_states_d;
    mpc_states_d.setZero(MPC_STATE_DIM * mpc_horizon_);
    for (size_t mpc_step = 0; mpc_step < mpc_horizon_; mpc_step++) {
        const double t_ahead = mpc_dt * (mpc_step + 1);
        // clang-format off
        mpc_states_d.segment(mpc_step * MPC_STATE_DIM, MPC_STATE_DIM) <<
		// orientation
        pitch_reference_,
        0,
        yaw_ref_ + cmd_vel_b[2] * t_ahead,
		// position
        position_ref_[0] + linear_velocity_w[0] * t_ahead,
		position_ref_[1] + linear_velocity_w[1] * t_ahead,
		z,
		// angular velocity
        0,
		0,
		cmd_vel_b[2],
		// linear velocity
        linear_velocity_w[0],
		linear_velocity_w[1],
		0,
		// gravity
        robot_model_.gravity();
        // clang-format on
    }

    robot_model_.updateAc(robot_state.Rz());
    robot_model_.updateBc(robot_state.Rz(), robot_state.foot_positions_abs());
    robot_model_.updateDiscretizedModel();

    // Per-step force constraints built from the predicted contact schedule, so the QP
    // plans through upcoming stance/swing transitions (Di Carlo et al., eq. 20-21).
    Eigen::VectorXd lower_bound, upper_bound;
    lower_bound.setZero(MPC_CONSTRAINT_DIM * mpc_horizon_);
    upper_bound.setZero(MPC_CONSTRAINT_DIM * mpc_horizon_);
    for (size_t mpc_step = 0; mpc_step < mpc_horizon_; mpc_step++) {
        const size_t offset = mpc_step * MPC_CONSTRAINT_DIM;
        for (int leg_idx = 0; leg_idx < LEG_NUM; leg_idx++) {
            const bool contact = predictContactState(robot_state, leg_idx, mpc_step * mpc_dt);
            const double fz_min = contact ? robot_model_.f_min() : 0;
            const double fz_max = contact ? robot_model_.f_max() : 0;
            lower_bound.segment(offset + leg_idx * 5, 5) << 0, -qpOASES::INFTY, 0, -qpOASES::INFTY, fz_min;
            upper_bound.segment(offset + leg_idx * 5, 5) << qpOASES::INFTY, 0, qpOASES::INFTY, 0, fz_max;
        }
    }

    ConvexMPC mpc_problem(q_weights_, r_weights_, lower_bound, upper_bound, constraint_coefficient_, mpc_horizon_);
    mpc_problem.updateQP(robot_model_.Ad(), robot_model_.Bd(), robot_state.mpc_state(), mpc_states_d);
    Eigen::VectorXd solution = mpc_problem.solve();
    mpc_result_ = (mpc_problem.A_qp() * robot_state.mpc_state() + mpc_problem.B_qp() * solution).tail(13);
    for (int leg_idx = 0; leg_idx < LEG_NUM; leg_idx++) {
        grf[leg_idx].setZero();
        if (!std::isnan(solution.segment<3>(leg_idx * 3).norm())) grf[leg_idx] = solution.segment<3>(leg_idx * 3);
    }

    return grf;
}

std::array<Eigen::Vector3d, LEG_NUM> RobotController::computeSwingForce(RobotState& robot_state,
                                                                        const Eigen::Ref<const Eigen::Vector3d>& cmd_vel_b) {
    // Swing trajectories are planned in the world frame (Di Carlo et al., Sec. II-D), so
    // they stay level when the body rolls or pitches. Foot positions/velocities are
    // world-frame, relative to the base; the returned PD forces are world-frame (eq. 1)
    // and the nominal footholds only rotate with yaw.
    std::array<Eigen::Vector3d, LEG_NUM> result;
    Eigen::Matrix3d Rz = robot_state.Rz();
    // Foothold heuristic (eq. 33) plus a velocity-error term: when the body is slower
    // than commanded, feet land further back, tipping the body into acceleration.
    // This carries most of the velocity tracking so the MPC needs less horizontal force.
    Eigen::Vector3d com_velocity_w = robot_state.linear_velocity();
    com_velocity_w[2] = 0;
    Eigen::Vector3d cmd_velocity_w = Rz * Eigen::Vector3d{cmd_vel_b[0], cmd_vel_b[1], 0};
    Eigen::Vector3d foothold_offset_w =
        com_velocity_w * stance_duration_ / 2 + foothold_velocity_error_gain_ * (com_velocity_w - cmd_velocity_w);
    for (int leg_idx = 0; leg_idx < LEG_NUM; leg_idx++) {
        result[leg_idx].setZero();
        if (robot_state.contact_state(leg_idx) == false) {
            // Capture the lift-off position on the first tick of the swing phase.
            // xy is kept body-relative (footholds move with the body); z is stored in
            // world coordinates so the height profile is anchored to the ground and
            // does not fight the MPC's body-height regulation.
            if (swing_counter_[leg_idx] <= 0) {
                swing_start_position_[leg_idx] = robot_state.foot_positions_abs(leg_idx);
                swing_start_position_[leg_idx][2] += robot_state.position()[2];
            }
            swing_counter_[leg_idx] += control_dt_;
            const double s = std::min(swing_counter_[leg_idx] / swing_duration_, 1.0);

            Eigen::Vector3d touchdown = Rz * robot_nominal_state_.foot_positions_abs(leg_idx) + foothold_offset_w;
            touchdown[2] = robot_nominal_state_.foot_positions_abs(leg_idx)[2] + robot_nominal_state_.position()[2];

            // Time-parameterized trajectory from lift-off to touchdown: smoothstep in
            // the ground plane, bezier height profile, with velocity feedforward.
            const double blend = s * s * (3 - 2 * s);
            const double blend_rate = 6 * s * (1 - s) / swing_duration_;
            Eigen::Vector3d foot_position_d =
                swing_start_position_[leg_idx] + blend * (touchdown - swing_start_position_[leg_idx]);
            Eigen::Vector3d foot_velocity_d = blend_rate * (touchdown - swing_start_position_[leg_idx]);
            const std::vector<double> height_profile = {0, 0, swing_height_, 0, 0};
            foot_position_d[2] += utils::bezier_curve(s, height_profile);
            foot_velocity_d[2] += utils::bezier_curve_derivative(s, height_profile) / swing_duration_;
            foot_position_d[2] -= robot_state.position()[2];  // back to body-relative for the PD error

            if (swing_counter_[leg_idx] >= swing_duration_) {
                swing_counter_[leg_idx] = 0;
                robot_state.updateContactState(leg_idx, true);
            }
            result[leg_idx] = Kp_ * (foot_position_d - robot_state.foot_positions_abs(leg_idx)) +
                              Kd_ * (foot_velocity_d - robot_state.foot_velocity(leg_idx));
        }
    }
    return result;
}

void RobotController::set_nominal_height(double height) {
    Eigen::Vector3d position = robot_nominal_state_.position();
    position[2] = height;
    robot_nominal_state_.updateState(robot_nominal_state_.euler_angle(),
                                     position,
                                     robot_nominal_state_.angular_velocity(),
                                     robot_nominal_state_.linear_velocity());
}

void RobotController::set_mpc_weights(const Eigen::Ref<const Eigen::Vector<double, MPC_STATE_DIM>>& q_weights,
                                      const Eigen::Ref<const Eigen::Vector<double, MPC_INPUT_DIM>>& r_weights) {
    q_weights_ = q_weights;
    r_weights_ = r_weights;
}

const Eigen::Vector<double, MPC_STATE_DIM>& RobotController::mpc_result() const { return mpc_result_; }

}  // namespace ConvexMPC