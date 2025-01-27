#include "convex_mpc/robot_model.hpp"
#include "convex_mpc/utils.hpp"

#include <iostream>
namespace ConvexMPC {
void RobotModel::updateAc(const Eigen::Ref<const Eigen::Vector3d>& euler_angle) {
    Eigen::Matrix3d rotation_matrix = euler_to_matrix(euler_angle);
    Ac_.block<3, 3>(0, 6) = rotation_matrix;
    Ac_.block<3, 3>(3, 9) = Eigen::Matrix3d::Identity();
}

void RobotModel::updateAc(const double& yaw) {
    Eigen::Matrix3d yaw_matrix = Eigen::Matrix3d::Identity() * Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ());
    Ac_.block<3, 3>(0, 6) = yaw_matrix;
    Ac_.block<3, 3>(3, 9) = Eigen::Matrix3d::Identity();
}

void RobotModel::updateBc(const Eigen::Ref<const Eigen::Matrix3d>& rotation_matrix,
                          const std::array<Eigen::Vector3d, LEG_NUM>& foot_positions) {
    Eigen::Matrix3d world_inertia;
    world_inertia = rotation_matrix * inertia_ * rotation_matrix.transpose();
    for (int leg_idx = 0; leg_idx < foot_positions.size(); ++leg_idx) {
        Bc_.block<3, 3>(6, 3 * leg_idx) = world_inertia.inverse() * vector_to_skew(foot_positions[leg_idx]);
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