#include "convex_mpc/robot_model.hpp"
#include "convex_mpc/utils.hpp"

#include <iostream>
namespace ConvexMPC {
RobotModel::RobotModel(const RobotModel& other) : state_(other.state_) {}
void RobotModel::setState(const RobotState& other) { state_ = other; }
void RobotModel::setState(const Eigen::Vector<double, MPC_STATE_DIM>& state) { state_ = RobotState(state); }
void RobotModel::setState(const Eigen::Vector3d& euler_angle,
                          const Eigen::Vector3d& position,
                          const Eigen::Vector3d& angular_velocity,
                          const Eigen::Vector3d& linear_velocity) {
    state_ = RobotState(euler_angle, position, angular_velocity, linear_velocity);
}

void RobotModel::updateAc() {
    Eigen::Matrix3d R_yaw;
    Eigen::Vector3d euler_angle = state_.getEulerAngle();
    // clang-format off
    R_yaw << cos(euler_angle[2]), sin(euler_angle[2]), 0,
			-sin(euler_angle[2]), cos(euler_angle[2]), 0,
							   0, 					0, 1;
    // clang-format on

    Ac_.block<3, 3>(0, 6) = R_yaw;
    Ac_.block<3, 3>(3, 9) = Eigen::Matrix3d::Identity();
}

void RobotModel::updateBc(const std::array<Eigen::Vector3d, LEG_NUM>& foot_positions) {
    Eigen::Matrix3d world_inertia;
    world_inertia = state_.getOrientationMatrix() * inertia_ * state_.getOrientationMatrix().transpose();
    for (int leg_idx = 0; leg_idx < foot_positions.size(); ++leg_idx) {
        Bc_.block<3, 3>(6, 3 * leg_idx) = world_inertia.inverse() * ConvexMPC::skew(foot_positions[leg_idx]);
        Bc_.block<3, 3>(9, 3 * leg_idx) = (1 / mass_) * Eigen::Matrix3d::Identity();
    }
}

void RobotModel::updateDiscretizedModel() {
    Ad_ = (Eigen::Matrix<double, MPC_STATE_DIM, MPC_STATE_DIM>::Identity() + 0.5 * dt_ * Ac_) *
          (Eigen::Matrix<double, MPC_STATE_DIM, MPC_STATE_DIM>::Identity() - 0.5 * dt_ * Ac_).inverse();
    Bd_ = Bc_ * dt_;
}

Eigen::Matrix<double, MPC_STATE_DIM, MPC_STATE_DIM> RobotModel::getAc() const { return Ac_; }
Eigen::Matrix<double, MPC_STATE_DIM, MPC_STATE_DIM> RobotModel::getAd() const { return Ad_; }
Eigen::Matrix<double, MPC_STATE_DIM, MPC_INPUT_DIM> RobotModel::getBc() const { return Bc_; }
Eigen::Matrix<double, MPC_STATE_DIM, MPC_INPUT_DIM> RobotModel::getBd() const { return Bd_; }
double RobotModel::getDt() const { return dt_; }
}  // namespace ConvexMPC