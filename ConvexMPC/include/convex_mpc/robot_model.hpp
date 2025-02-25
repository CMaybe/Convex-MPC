#ifndef ROBOT_MODEL_HPP
#define ROBOT_MODEL_HPP

#include "convex_mpc/params.hpp"
#include "convex_mpc/robot_state.hpp"

#include <Eigen/Dense>
#include <cmath>

namespace ConvexMPC {
class RobotModel {
public:
    RobotModel() = delete;
    RobotModel(
        const double& mass, const double& gravity, const double& mu, const double& dt, const double& f_min, const double& f_max);
    RobotModel(const Eigen::Ref<const Eigen::Matrix3d>& inertia,
               const double& mass,
               const double& gravity,
               const double& mu,
               const double& dt,
               const double& f_min,
               const double& f_max);

    void updateAc(const Eigen::Ref<const Eigen::Matrix3d>& Rz);
    void updateBc(const Eigen::Ref<const Eigen::Matrix3d>& Rz, const std::array<Eigen::Vector3d, LEG_NUM>& foot_positions_com);
    void updateDiscretizedModel();

    // getter
    double dt() const { return dt_; };
    double mass() const { return mass_; };
    double gravity() const { return gravity_; };
    double mu() const { return mu_; };
    double f_min() const { return f_min_; };
    double f_max() const { return f_max_; };

    const Eigen::Matrix<double, MPC_STATE_DIM, MPC_STATE_DIM>& Ac() const { return Ac_; };
    const Eigen::Matrix<double, MPC_STATE_DIM, MPC_INPUT_DIM>& Bc() const { return Bc_; };
    const Eigen::Matrix<double, MPC_STATE_DIM, MPC_STATE_DIM>& Ad() const { return Ad_; };
    const Eigen::Matrix<double, MPC_STATE_DIM, MPC_INPUT_DIM>& Bd() const { return Bd_; };

private:
    Eigen::Matrix<double, MPC_STATE_DIM, MPC_STATE_DIM> Ac_;
    Eigen::Matrix<double, MPC_STATE_DIM, MPC_INPUT_DIM> Bc_;
    Eigen::Matrix<double, MPC_STATE_DIM, MPC_STATE_DIM> Ad_;
    Eigen::Matrix<double, MPC_STATE_DIM, MPC_INPUT_DIM> Bd_;

    Eigen::Matrix3d inertia_;
    double mass_, gravity_, mu_, dt_, f_min_, f_max_;
};
}  // namespace ConvexMPC

#endif  // ROBOT_MODEL_HPP
