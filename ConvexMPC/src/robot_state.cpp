#include "convex_mpc/robot_state.hpp"
#include "robot_state.hpp"

namespace ConvexMPC {
RobotState::RobotState(const RobotState& other)
    : euler_angle_(other.euler_angle_)
    , position_(other.position_)
    , angular_velocity_(other.angular_velocity_)
    , linear_velocity_(other.linear_velocity_) {
    mpc_state_ << euler_angle_, position_, angular_velocity_, linear_velocity_, -9.81;
}

RobotState::RobotState(const Eigen::Vector<double, 13>& state) : mpc_state_(state) {
    euler_angle_ = mpc_state_.block(0, 0, 3, 1);
    position_ = mpc_state_.block(3, 0, 3, 1);
    angular_velocity_ = mpc_state_.block(6, 0, 3, 1);
    linear_velocity_ = mpc_state_.block(9, 0, 3, 1);
}

RobotState::RobotState(const Eigen::Vector3d& euler_angle,
                       const Eigen::Vector3d& position,
                       const Eigen::Vector3d& angular_velocity,
                       const Eigen::Vector3d& linear_velocity) {
    euler_angle_ = euler_angle;
    position_ = position;
    angular_velocity_ = angular_velocity;
    linear_velocity_ = linear_velocity;
    mpc_state_ << euler_angle_, position_, angular_velocity_, linear_velocity_, -9.81;
}

Eigen::Vector3d RobotState::getPosition() const { return position_; }
Eigen::Vector3d RobotState::getEulerAngle() const { return euler_angle_; }
Eigen::Vector3d RobotState::getLinearVelocity() const { return linear_velocity_; }
Eigen::Vector3d RobotState::getAngularVelocity() const { return angular_velocity_; }
Eigen::Vector<double, 13> RobotState::getControlState() const { return mpc_state_; }

}  // namespace ConvexMPC
