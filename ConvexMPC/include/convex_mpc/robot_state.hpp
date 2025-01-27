#ifndef ROBOT_STATE_HPP
#define ROBOT_STATE_HPP

#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace ConvexMPC {
class RobotState {
public:
    RobotState() = delete;
    RobotState(const Eigen::Vector3d& euler_angle,
               const Eigen::Vector3d& position,
               const Eigen::Vector3d& angular_velocity,
               const Eigen::Vector3d& linear_velocity);
    RobotState(const RobotState& other);
    RobotState(const Eigen::Vector<double, 13>& state);
    Eigen::Vector<double, 13> getControlState() const;
    Eigen::Vector3d getPosition() const;
    Eigen::Vector3d getEulerAngle() const;
    Eigen::Vector3d getLinearVelocity() const;
    Eigen::Vector3d getAngularVelocity() const;
    Eigen::Matrix3d getOrientationMatrix() const;

private:
    // state
    Eigen::Vector3d euler_angle_;
    Eigen::Vector3d position_;
    Eigen::Vector3d angular_velocity_;
    Eigen::Vector3d linear_velocity_;
    Eigen::Vector<double, 13> mpc_state_;

    //
    Eigen::Quaterniond body_orientation_quaternion_;
    Eigen::Matrix3d body_orientation_matrix_;
    Eigen::Vector3d linear_acceleration;
};

}  // namespace ConvexMPC

#endif