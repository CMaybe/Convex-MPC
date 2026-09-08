#include "convex_mpc/robot_model.hpp"
#include "convex_mpc/utils.hpp"

#include <unsupported/Eigen/MatrixFunctions>
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

void RobotModel::updateAc(const Eigen::Ref<const Eigen::Matrix3d>& Rz) {
    Ac_.setZero();
    // Euler-rate mapping for small roll/pitch: d(Theta)/dt = Rz(psi)^T * omega  (Di Carlo et al., eq. 10-12)
    Ac_.block<3, 3>(0, 6) = Rz.transpose();
    Ac_.block<3, 3>(3, 9) = Eigen::Matrix3d::Identity();
    Ac_(11, 12) = 1.0;
}

void RobotModel::updateBc(const Eigen::Ref<const Eigen::Matrix3d>& Rz,
                          const std::array<Eigen::Vector3d, LEG_NUM>& foot_positions_abs) {
    Eigen::Matrix3d world_inertia;
    world_inertia = Rz * inertia_ * Rz.transpose();
    Bc_.setZero();
    for (size_t leg_idx = 0; leg_idx < LEG_NUM; leg_idx++) {
        Bc_.block<3, 3>(6, 3 * leg_idx) = world_inertia.inverse() * utils::vector_to_skew(foot_positions_abs[leg_idx]);
        Bc_.block<3, 3>(9, 3 * leg_idx) = (1 / mass_) * Eigen::Matrix3d::Identity();
    }
}

void RobotModel::updateDiscretizedModel() {
    // Zero-order hold via the state transition matrix of the extended system (Di Carlo et al., eq. 25-26)
    constexpr int EXT_DIM = MPC_STATE_DIM + MPC_INPUT_DIM;
    Eigen::Matrix<double, EXT_DIM, EXT_DIM> M = Eigen::Matrix<double, EXT_DIM, EXT_DIM>::Zero();
    M.topLeftCorner<MPC_STATE_DIM, MPC_STATE_DIM>() = Ac_;
    M.topRightCorner<MPC_STATE_DIM, MPC_INPUT_DIM>() = Bc_;
    const Eigen::Matrix<double, EXT_DIM, EXT_DIM> Md = (M * dt_).exp();
    Ad_ = Md.topLeftCorner<MPC_STATE_DIM, MPC_STATE_DIM>();
    Bd_ = Md.topRightCorner<MPC_STATE_DIM, MPC_INPUT_DIM>();
}

}  // namespace ConvexMPC