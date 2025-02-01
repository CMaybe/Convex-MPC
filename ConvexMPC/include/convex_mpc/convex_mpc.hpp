#ifndef CONVEX_MPC_HPP
#define CONVEX_MPC_HPP

#include <chrono>
#include <vector>

#include <Eigen/Dense>
#include <Eigen/Sparse>

#include "convex_mpc/params.hpp"
#include "convex_mpc/robot_model.hpp"
#include "convex_mpc/robot_state.hpp"

namespace ConvexMPC {
class ConvexMPC {
public:
    ConvexMPC(const RobotModel& robot_model,
              const RobotState& robot_state,
              const RobotState& robot_desired_state,
              const Eigen::Ref<const Eigen::Vector<double, MPC_STATE_DIM>>& state_weight,
              const Eigen::Ref<const Eigen::Vector<double, MPC_INPUT_DIM>>& input_weight);
    void updateQP();

    // getter
    const RobotModel& robot_model() const { return robot_model_; }
    const RobotState& robot_state() const { return robot_state_; }
    const RobotState& robot_desired_state() const { return robot_desired_state_; }

    const Eigen::DiagonalMatrix<double, MPC_STATE_DIM * MPC_HORIZON>& Q() const { return Q_; }
    const Eigen::DiagonalMatrix<double, MPC_INPUT_DIM * MPC_HORIZON>& R() const { return R_; }

    const Eigen::MatrixXd& A_qp() const { return A_qp_; }
    const Eigen::MatrixXd& B_qp() const { return B_qp_; }

    const Eigen::MatrixXd& hessian() const { return hessian_; }
    const Eigen::SparseMatrix<double>& linear_constraints() const { return linear_constraints_; }

    const Eigen::VectorXd& gradient() const { return gradient_; }
    const Eigen::VectorXd& lb() const { return lb_; }
    const Eigen::VectorXd& ub() const { return ub_; }

private:
    RobotModel robot_model_;
    RobotState robot_state_;
    RobotState robot_desired_state_;

    Eigen::DiagonalMatrix<double, MPC_STATE_DIM * MPC_HORIZON> Q_;
    Eigen::DiagonalMatrix<double, MPC_INPUT_DIM * MPC_HORIZON> R_;

    Eigen::MatrixXd A_qp_;
    Eigen::MatrixXd B_qp_;

    Eigen::MatrixXd hessian_;
    Eigen::SparseMatrix<double> linear_constraints_;

    Eigen::VectorXd gradient_;  // q
    Eigen::VectorXd lb_;        // lower bound constraints
    Eigen::VectorXd ub_;        // upper bound constraints
};
}  // namespace ConvexMPC
#endif
