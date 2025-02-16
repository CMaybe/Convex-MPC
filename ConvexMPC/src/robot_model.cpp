#include "convex_mpc/robot_model.hpp"
#include "convex_mpc/utils.hpp"

#include <iostream>
namespace ConvexMPC {

RobotModel::RobotModel(
    const double& mass, const double& gravity, const double& mu, const double& dt, const double& f_min, const double& f_max)
    : mass_(mass), gravity_(gravity), mu_(mu), dt_(dt), f_min_(f_min), f_max_(f_max) {
    inertia_.setIdentity();
    Ac_.setZero();
    Bc_.setZero();
    Ad_.setZero();
    Bd_.setZero();
}

RobotModel::RobotModel(const Eigen::Ref<const Eigen::Matrix3d>& inertia,
                       const double& mass,
                       const double& gravity,
                       const double& mu,
                       const double& dt,
                       const double& f_min,
                       const double& f_max)
    : inertia_(inertia), mass_(mass), gravity_(gravity), mu_(mu), dt_(dt), f_min_(f_min), f_max_(f_max) {
    Ac_.setZero();
    Bc_.setZero();
    Ad_.setZero();
    Bd_.setZero();
}

void RobotModel::updateAc(const Eigen::Ref<const Eigen::Matrix3d>& rotation_matrix) {
    Ac_.block<3, 3>(0, 6) = rotation_matrix;
    Ac_.block<3, 3>(3, 9) = Eigen::Matrix3d::Identity();
    Ac_(11, 12) = 1.0;
}

void RobotModel::updateBc(const Eigen::Ref<const Eigen::Matrix3d>& rotation_matrix,
                          const std::array<Eigen::Vector3d, LEG_NUM>& foot_positions_b) {
    Eigen::Matrix3d world_inertia;
    world_inertia = rotation_matrix * inertia_ * rotation_matrix.transpose();
    for (size_t leg_idx = 0; leg_idx < LEG_NUM; leg_idx++) {
        Bc_.block<3, 3>(6, 3 * leg_idx) = world_inertia.inverse() * utils::vector_to_skew(foot_positions_b[leg_idx]);
        Bc_.block<3, 3>(9, 3 * leg_idx) = (1 / mass_) * Eigen::Matrix3d::Identity();
    }
}

void RobotModel::updateDiscretizedModel() {
    Ad_ = Eigen::Matrix<double, MPC_STATE_DIM, MPC_STATE_DIM>::Identity() + Ac_ * dt_;
    Bd_ = Bc_ * dt_;
}

}  // namespace ConvexMPC