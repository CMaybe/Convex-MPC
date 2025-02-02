#ifndef CONVEX_MPC_HPP
#define CONVEX_MPC_HPP

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <qpOASES.hpp>

#include "convex_mpc/params.hpp"
#include "convex_mpc/robot_model.hpp"
#include "convex_mpc/robot_state.hpp"

namespace ConvexMPC {
class ConvexMPC {
public:
    ConvexMPC(const Eigen::Ref<const Eigen::Vector<double, MPC_STATE_DIM>>& state_weight,
              const Eigen::Ref<const Eigen::Vector<double, MPC_INPUT_DIM>>& input_weight,
              const Eigen::Ref<const Eigen::Vector<double, MPC_STATE_DIM>>& lower_bound,
              const Eigen::Ref<const Eigen::Vector<double, MPC_INPUT_DIM>>& upper_bound,
              const Eigen::Ref<const Eigen::MatrixXd>& constraint_coefficient);
    void updateQP(const Eigen::Ref<const Eigen::Matrix<double, MPC_STATE_DIM, MPC_STATE_DIM>>& Ad,
                  const Eigen::Ref<const Eigen::Matrix<double, MPC_STATE_DIM, MPC_INPUT_DIM>>& Bd,
                  const Eigen::Ref<const Eigen::Vector<double, MPC_STATE_DIM>>& x0,
                  const Eigen::Ref<const Eigen::Vector<double, MPC_STATE_DIM>>& y);

    // getter
    const Eigen::MatrixXd& Q() const { return Q_; }
    const Eigen::MatrixXd& R() const { return R_; }

    const Eigen::MatrixXd& A_qp() const { return A_qp_; }
    const Eigen::MatrixXd& B_qp() const { return B_qp_; }

    const Eigen::MatrixXd& hessian() const { return hessian_; }
    const Eigen::MatrixXd& linear_constraints() const { return linear_constraints_; }

    const Eigen::VectorXd& gradient() const { return gradient_; }
    const Eigen::VectorXd& lb() const { return lb_; }
    const Eigen::VectorXd& ub() const { return ub_; }

private:
    Eigen::MatrixXd Q_;
    Eigen::MatrixXd R_;

    Eigen::MatrixXd A_qp_;
    Eigen::MatrixXd B_qp_;

    Eigen::MatrixXd hessian_;
    Eigen::MatrixXd linear_constraints_;

    Eigen::VectorXd gradient_;  // q
    Eigen::VectorXd lb_;        // lower bound constraints
    Eigen::VectorXd ub_;        // upper bound constraints
};
}  // namespace ConvexMPC
#endif
