#ifndef RobotController_H
#define RobotController_H

#include <Eigen/Dense>
#include <qpOASES.hpp>

#include "convex_mpc/convex_mpc.hpp"
#include "convex_mpc/params.hpp"
#include "convex_mpc/robot_model.hpp"
#include "convex_mpc/robot_state.hpp"
#include "convex_mpc/utils.hpp"

namespace ConvexMPC {
class RobotController {
public:
    RobotController(const RobotModel& robot_model,
                    const Eigen::Ref<const Eigen::Vector<double, MPC_STATE_DIM>>& q_weights,
                    const Eigen::Ref<const Eigen::Vector<double, MPC_INPUT_DIM>>& r_weights,
                    const double& kp,
                    const double& kd,
                    const size_t& mpc_horizon = 10,
                    const double& control_dt = 0.001,
                    const double& stance_duration = 0.2,
                    const double& swing_duration = 0.2);
    RobotController(const RobotModel& robot_model,
                    const RobotState& nominal_state,
                    const Eigen::Ref<const Eigen::Vector<double, MPC_STATE_DIM>>& q_weights,
                    const Eigen::Ref<const Eigen::Vector<double, MPC_INPUT_DIM>>& r_weights,
                    const double& kp,
                    const double& kd,
                    const size_t& mpc_horizon,
                    const double& control_dt = 0.001,
                    const double& stance_duration = 0.2,
                    const double& swing_duration = 0.2);
    std::array<Eigen::Vector3d, LEG_NUM> computeGRF(RobotState& robot_state, const Eigen::Ref<const Eigen::Vector3d>& cmd_vel);
    // Sets each leg's elapsed time within its current stance phase, defining the
    // phase offsets between legs (e.g. a half-cycle offset between trot pairs).
    // Call once at gait start, with every leg in contact.
    void resetGaitPhase(const std::array<double, LEG_NUM>& stance_elapsed);
    std::array<Eigen::Vector3d, LEG_NUM> computeSwingForce(RobotState& robot_state,
                                                           const Eigen::Ref<const Eigen::Vector3d>& cmd_vel);
    const Eigen::Vector<double, MPC_STATE_DIM>& mpc_result() const;

private:
    // Position within the gait cycle (0 = touchdown) `time_ahead` seconds from now,
    // from the fixed-timing gait schedule (stance/swing phase timers).
    double predictCyclePosition(const RobotState& robot_state, const size_t& leg_idx, const double& time_ahead) const;
    // Predicts whether a leg is in contact `time_ahead` seconds from now.
    bool predictContactState(const RobotState& robot_state, const size_t& leg_idx, const double& time_ahead) const;

    static constexpr double kForceRampDuration = 0.06;  // fz ramp-in time after touchdown [s]

    RobotModel robot_model_;
    RobotState robot_nominal_state_;

    Eigen::Vector<double, MPC_STATE_DIM> q_weights_;
    Eigen::Vector<double, MPC_INPUT_DIM> r_weights_;
    Eigen::Matrix3d Kp_, Kd_;
    size_t mpc_horizon_;
    double control_dt_;
    double stance_duration_;
    double swing_duration_;

    Eigen::MatrixXd constraint_coefficient_;

    static constexpr double kSwingHeight = 0.3;  // apex of the swing-foot height profile [m]

    std::array<double, LEG_NUM> swing_counter_{};
    std::array<double, LEG_NUM> stance_counter_{};
    std::array<Eigen::Vector3d, LEG_NUM> swing_start_position_{};

    // Integrated command reference (yaw and xy-position), see computeGRF.
    bool reference_initialized_ = false;
    double yaw_ref_ = 0;
    Eigen::Vector2d position_ref_ = Eigen::Vector2d::Zero();

    Eigen::Vector<double, MPC_STATE_DIM> mpc_result_;
};
}  // namespace ConvexMPC

#endif  // RobotController_H