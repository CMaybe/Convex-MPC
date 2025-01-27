#ifndef ROBOT_MODEL_HPP
#define ROBOT_MODEL_HPP

#include "convex_mpc/params.hpp"
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
    void setState(const Eigen::Vector<double, MPC_STATE_DIM>& state);
    void RobotModel::updateAc();
    void RobotModel::updateBc(const std::array<Eigen::Vector3d, LEG_NUM>& foot_position);

    Eigen::Matrix<double, MPC_STATE_DIM, MPC_STATE_DIM> getAc() const;
    Eigen::Matrix<double, MPC_STATE_DIM, MPC_INPUT_DIM> getBc() const;
    Eigen::Matrix<double, MPC_STATE_DIM, MPC_STATE_DIM> getAd() const;
    Eigen::Matrix<double, MPC_STATE_DIM, MPC_INPUT_DIM> getBd() const;

    double getDt() const;
    void updateDiscretizedModel();

private:
    Eigen::Matrix<double, MPC_STATE_DIM, MPC_STATE_DIM> Ac_;
    Eigen::Matrix<double, MPC_STATE_DIM, MPC_INPUT_DIM> Bc_;
    Eigen::Matrix<double, MPC_STATE_DIM, MPC_STATE_DIM> Ad_;
    Eigen::Matrix<double, MPC_STATE_DIM, MPC_INPUT_DIM> Bd_;

    Eigen::Matrix3d inertia_;
    double mass_, mu_, gravity_;
    double dt_;

    RobotState state_;
};
}  // namespace ConvexMPC

#endif  // ROBOT_MODEL_HPP
