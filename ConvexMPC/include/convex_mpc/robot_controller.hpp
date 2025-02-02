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
    RobotController(const RobotModel& robot_model);
    std::array<Eigen::Vector3d, LEG_NUM> compute_grf();

private:
    RobotModel robot_model_;
    RobotState robot_state_;
    RobotState robot_desired_state_;

    Eigen::Vector<double, MPC_STATE_DIM> q_weights_;
    Eigen::Vector<double, MPC_INPUT_DIM> r_weights_;
    Eigen::Vector<double, MPC_CONSTRAINT_DIM> lower_bound_;
    Eigen::Vector<double, MPC_CONSTRAINT_DIM> upper_bound_;
    Eigen::MatrixXd constraint_coefficient_;
};
}  // namespace ConvexMPC

#endif  // RobotController_H