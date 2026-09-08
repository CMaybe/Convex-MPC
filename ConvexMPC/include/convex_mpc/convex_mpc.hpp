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
    // lower_bound/upper_bound are stacked per-horizon-step bounds (MPC_CONSTRAINT_DIM * mpc_horizon),
    // so the QP can see future contact-state changes over the prediction horizon.
    ConvexMPC(const Eigen::Ref<const Eigen::Vector<double, MPC_STATE_DIM>>& state_weight,
              const Eigen::Ref<const Eigen::Vector<double, MPC_INPUT_DIM>>& input_weight,
              const Eigen::Ref<const Eigen::VectorXd>& lower_bound,
              const Eigen::Ref<const Eigen::VectorXd>& upper_bound,
              const Eigen::Ref<const Eigen::MatrixXd>& constraint_coefficient,
              const size_t& mpc_horizon);
    void updateQP(const Eigen::Ref<const Eigen::Matrix<double, MPC_STATE_DIM, MPC_STATE_DIM>>& Ad,
                  const Eigen::Ref<const Eigen::Matrix<double, MPC_STATE_DIM, MPC_INPUT_DIM>>& Bd,
                  const Eigen::Ref<const Eigen::Vector<double, MPC_STATE_DIM>>& x0,
                  const Eigen::Ref<const Eigen::VectorXd>& y);
    const Eigen::VectorXd solve();

    // getter
    inline const Eigen::MatrixXd& L() const { return L_; }
    inline const Eigen::MatrixXd& K() const { return K_; }

    inline const Eigen::MatrixXd& A_qp() const { return A_qp_; }
    inline const Eigen::MatrixXd& B_qp() const { return B_qp_; }

    inline const Eigen::MatrixXd& hessian() const { return hessian_; }
    inline const Eigen::MatrixXd& linear_constraints() const { return linear_constraints_; }

    inline const Eigen::VectorXd& gradient() const { return gradient_; }
    inline const Eigen::VectorXd& lb() const { return lb_; }
    inline const Eigen::VectorXd& ub() const { return ub_; }

private:
    Eigen::MatrixXd L_;
    Eigen::MatrixXd K_;

    Eigen::MatrixXd A_qp_;
    Eigen::MatrixXd B_qp_;

    Eigen::MatrixXd hessian_;
    Eigen::MatrixXd linear_constraints_;

    Eigen::VectorXd gradient_;  // q
    Eigen::VectorXd lb_;        // lower bound constraints
    Eigen::VectorXd ub_;        // upper bound constraints

    size_t mpc_horizon_;
};
}  // namespace ConvexMPC

#endif
