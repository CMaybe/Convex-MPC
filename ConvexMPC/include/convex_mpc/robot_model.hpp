#ifndef ROBOT_MODEL_HPP
#define ROBOT_MODEL_HPP

#include "convex_mpc/robot_state.hpp"

#include <Eigen/Dense>
#include <cmath>

namespace ConvexMPC {
class RobotModel {
public:
    RobotModel() = default;
    RobotModel(const RobotModel& other);

    void setState(const Eigen::Vector3d& euler_angle,
                  const Eigen::Vector3d& position,
                  const Eigen::Vector3d& angular_velocity,
                  const Eigen::Vector3d& linear_velocity);
    void setState(const RobotState& other);
    void setState(const Eigen::Vector<double, 13>& state);
    void RobotModel::updateAc();
    void RobotModel::updateBc(Eigen::Matrix<double, 3, 4> foot_position);

    Eigen::Matrix<double, 13, 13> getAc() const;
    Eigen::Matrix<double, 13, 12> getBc() const;
    Eigen::Matrix<double, 13, 13> getAd() const;
    Eigen::Matrix<double, 13, 12> getBd() const;

    double getDt() const;
    void updateDiscretizedModel();

private:
    Eigen::Matrix<double, 13, 13> Ac_;
    Eigen::Matrix<double, 13, 12> Bc_;
    Eigen::Matrix<double, 13, 13> Ad_;
    Eigen::Matrix<double, 13, 12> Bd_;

    Eigen::Matrix3d inertia_;
    double mass_, mu_, gravity_;
    double dt_;

    RobotState state_;
};
}  // namespace ConvexMPC

#endif  // ROBOT_MODEL_HPP
