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
    RobotModel(const double& mass, const double& gravity, const double& mu, const double& dt);
    RobotModel(const Eigen::Ref<const Eigen::Matrix3d>& inertia,
               const double& mass,
               const double& gravity,
               const double& mu,
               const double& dt);

    void updateAc(const double& yaw);
    void updateAc(const Eigen::Ref<const Eigen::Vector3d>& euler_angle);
    void updateBc(const Eigen::Ref<const Eigen::Matrix3d>& rotation_matrix,
                  const std::array<Eigen::Vector3d, LEG_NUM>& foot_position);
    void updateDiscretizedModel();

    // getter
    const double dt() const { return dt_; };
    const double mass() const { return mass_; };
    const double gravity() const { return gravity_; };
    const double mu() const { return mu_; };
    const double f_min() const { return f_min_; };
    const double f_max() const { return f_max_; };

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
    double mass_, mu_, gravity_, dt_, f_min_, f_max_;
};
}  // namespace ConvexMPC

#endif  // ROBOT_MODEL_HPP
