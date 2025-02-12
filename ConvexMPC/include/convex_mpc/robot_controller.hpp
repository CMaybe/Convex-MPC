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
                    const double& swing_duration = 0.2,
                    const double& stance_duration = 0.2);
    RobotController(const RobotModel& robot_model,
                    const RobotState& nominal_state,
                    const Eigen::Ref<const Eigen::Vector<double, MPC_STATE_DIM>>& q_weights,
                    const Eigen::Ref<const Eigen::Vector<double, MPC_INPUT_DIM>>& r_weights,
                    const double& kp,
                    const double& kd,
                    const double& swing_duration = 0.2,
                    const double& stance_duration = 0.2);
    std::array<Eigen::Vector3d, LEG_NUM> computeGRF(RobotState& robot_state, const Eigen::Ref<const Eigen::Vector3d>& cmd_vel);
    std::array<Eigen::Vector3d, LEG_NUM> computeSwingForce(RobotState& robot_state,
                                                           const Eigen::Ref<const Eigen::Vector3d>& cmd_vel);

private:
    RobotModel robot_model_;
    RobotState robot_nominal_state_;

    Eigen::Vector<double, MPC_STATE_DIM> q_weights_;
    Eigen::Vector<double, MPC_INPUT_DIM> r_weights_;
    Eigen::Vector<double, MPC_CONSTRAINT_DIM> lower_bound_;
    Eigen::Vector<double, MPC_CONSTRAINT_DIM> upper_bound_;
    Eigen::MatrixXd constraint_coefficient_;

    Eigen::Matrix3d Kp_, Kd_;

    std::array<double, LEG_NUM> swing_counter_;
    std::array<double, LEG_NUM> stance_counter_;

    double swing_duration_;
    double stance_duration_;
};
}  // namespace ConvexMPC

#endif  // RobotController_H