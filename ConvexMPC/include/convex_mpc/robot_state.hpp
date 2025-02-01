#ifndef ROBOT_STATE_HPP
#define ROBOT_STATE_HPP

#include "convex_mpc/params.hpp"
#include "convex_mpc/utils.hpp"

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace ConvexMPC {
class RobotState {
public:
    RobotState() = default;
    RobotState(const Eigen::Ref<const Eigen::Vector3d>& euler_angle,
               const Eigen::Ref<const Eigen::Vector3d>& position,
               const Eigen::Ref<const Eigen::Vector3d>& angular_velocity,
               const Eigen::Ref<const Eigen::Vector3d>& linear_velocity);
    RobotState(const RobotState& other);
    RobotState(const Eigen::Ref<const Eigen::Vector<double, MPC_STATE_DIM>>& mpc_state);

    // getter
    const Eigen::Vector3d& position() const { return position_; }
    const Eigen::Vector3d& euler_angle() const { return euler_angle_; }
    const Eigen::Vector3d& linear_velocity() const { return linear_velocity_; }
    const Eigen::Vector3d& angular_velocity() const { return angular_velocity_; }
    const Eigen::Matrix3d& rotation_matrix() const { return euler_to_matrix(euler_angle_); }
    const Eigen::Vector<double, MPC_STATE_DIM>& mpc_state() const { return mpc_state_; }
    const Eigen::Vector3d foot_pos(const size_t& leg_idx) const { return foot_pos_[leg_idx]; }
    const std::array<Eigen::Vector3d, LEG_NUM>& foot_pos() const { return foot_pos_; }

private:
    // state
    Eigen::Vector3d euler_angle_;
    Eigen::Vector3d position_;
    Eigen::Vector3d angular_velocity_;
    Eigen::Vector3d linear_velocity_;
    Eigen::Vector<double, MPC_STATE_DIM> mpc_state_;

    //
    Eigen::Quaterniond body_quaternion_;
    Eigen::Vector3d linear_acceleration;
    std::array<Eigen::Vector3d, LEG_NUM> foot_pos_;
};

}  // namespace ConvexMPC

#endif