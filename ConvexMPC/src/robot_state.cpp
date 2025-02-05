#include "convex_mpc/robot_state.hpp"
#include "convex_mpc/utils.hpp"

namespace ConvexMPC {
RobotState::RobotState(const RobotState& other)
    : euler_angle_(other.euler_angle_)
    , position_(other.position_)
    , angular_velocity_(other.angular_velocity_)
    , linear_velocity_(other.linear_velocity_) {
    mpc_state_ << euler_angle_, position_, angular_velocity_, linear_velocity_, -9.81;
}

RobotState::RobotState(const Eigen::Ref<const Eigen::Vector<double, MPC_STATE_DIM>>& mpc_state) : mpc_state_(mpc_state) {
    euler_angle_ = mpc_state_.segment(0, 3);
    position_ = mpc_state_.segment(3, 3);
    angular_velocity_ = mpc_state_.segment(6, 3);
    linear_velocity_ = mpc_state_.segment(9, 3);
}

RobotState::RobotState(const Eigen::Ref<const Eigen::Vector3d>& euler_angle,
                       const Eigen::Ref<const Eigen::Vector3d>& position,
                       const Eigen::Ref<const Eigen::Vector3d>& angular_velocity,
                       const Eigen::Ref<const Eigen::Vector3d>& linear_velocity) {
    euler_angle_ = euler_angle;
    position_ = position;
    angular_velocity_ = angular_velocity;
    linear_velocity_ = linear_velocity;
    mpc_state_ << euler_angle_, position_, angular_velocity_, linear_velocity_, -9.81;
}

void RobotState::setContactState(const std::array<bool, LEG_NUM>& contact_state) { contact_state_ = contact_state; }

}  // namespace ConvexMPC
