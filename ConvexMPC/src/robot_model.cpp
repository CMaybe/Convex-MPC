#include "convex_mpc/robot_model.hpp"
#include "convex_mpc/utils.hpp"

#include <iostream>
namespace ConvexMPC {

RobotModel::RobotModel(const double& mass, const double& gravity, const double& mu, const double& dt)
    : mass_(mass), gravity_(gravity), mu_(mu), dt_(dt) {
    inertia_.setIdentity();
}

RobotModel::RobotModel(const Eigen::Ref<const Eigen::Matrix3d>& inertia,
                       const double& mass,
                       const double& gravity,
                       const double& mu,
                       const double& dt)
    : inertia_(inertia), mass_(mass), gravity_(gravity), mu_(mu), dt_(dt) {}

void RobotModel::updateAc(const Eigen::Ref<const Eigen::Vector3d>& euler_angle) {
    Eigen::Matrix3d rotation_matrix = utils::euler_to_matrix(euler_angle);
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
    for (size_t leg_idx = 0; leg_idx < LEG_NUM; leg_idx++) {
        Bc_.block<3, 3>(6, 3 * leg_idx) = world_inertia.inverse() * utils::vector_to_skew(foot_positions[leg_idx]);
        Bc_.block<3, 3>(9, 3 * leg_idx) = (1 / mass_) * Eigen::Matrix3d::Identity();
    }
}

void RobotModel::updateDiscretizedModel() {
    Ad_ = (Eigen::Matrix<double, MPC_STATE_DIM, MPC_STATE_DIM>::Identity() + 0.5 * dt_ * Ac_) *
          (Eigen::Matrix<double, MPC_STATE_DIM, MPC_STATE_DIM>::Identity() - 0.5 * dt_ * Ac_).inverse();
    Bd_ = Bc_ * dt_;
}

}  // namespace ConvexMPC