#ifndef ROBOT_STATE_HPP
#define ROBOT_STATE_HPP

#include "convex_mpc/params.hpp"

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace ConvexMPC {
class RobotState {
public:
    RobotState() = delete;
    RobotState(const Eigen::Ref<const Eigen::Vector3d>& euler_angle,
               const Eigen::Ref<const Eigen::Vector3d>& position,
               const Eigen::Ref<const Eigen::Vector3d>& angular_velocity,
               const Eigen::Ref<const Eigen::Vector3d>& linear_velocity);
    RobotState(const RobotState& other);
    RobotState(const Eigen::Ref<const Eigen::Vector<double, MPC_STATE_DIM>>& mpc_state);

    Eigen::Vector<double, MPC_STATE_DIM> getControlState() const;
    Eigen::Vector3d getPosition() const;
    Eigen::Vector3d getEulerAngle() const;
    Eigen::Vector3d getLinearVelocity() const;
    Eigen::Vector3d getAngularVelocity() const;
    Eigen::Matrix3d getRotationMatrix() const;

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
};

}  // namespace ConvexMPC

#endif