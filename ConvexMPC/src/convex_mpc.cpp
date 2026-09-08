#include "convex_mpc/convex_mpc.hpp"
#include <iostream>
namespace ConvexMPC {

ConvexMPC::ConvexMPC(const Eigen::Ref<const Eigen::Vector<double, MPC_STATE_DIM>>& state_weight,
                     const Eigen::Ref<const Eigen::Vector<double, MPC_INPUT_DIM>>& input_weight,
                     const Eigen::Ref<const Eigen::VectorXd>& lower_bound,
                     const Eigen::Ref<const Eigen::VectorXd>& upper_bound,
                     const Eigen::Ref<const Eigen::MatrixXd>& constraint_coefficient,
                     const size_t& mpc_horizon)
    : mpc_horizon_(mpc_horizon) {
    A_qp_.setZero(MPC_STATE_DIM * mpc_horizon_, MPC_STATE_DIM);
    B_qp_.setZero(MPC_STATE_DIM * mpc_horizon_, MPC_INPUT_DIM * mpc_horizon_);
    lb_ = lower_bound;
    ub_ = upper_bound;
    L_.setZero(MPC_STATE_DIM * mpc_horizon_, MPC_STATE_DIM * mpc_horizon_);
    K_.setZero(MPC_INPUT_DIM * mpc_horizon_, MPC_INPUT_DIM * mpc_horizon_);
    gradient_.setZero(MPC_INPUT_DIM * mpc_horizon_);
    hessian_.setZero(MPC_INPUT_DIM * mpc_horizon_, MPC_INPUT_DIM * mpc_horizon_);

    for (size_t mpc_step = 0; mpc_step < mpc_horizon_; mpc_step++) {
        L_.block<MPC_STATE_DIM, MPC_STATE_DIM>(mpc_step * MPC_STATE_DIM, mpc_step * MPC_STATE_DIM) = state_weight.asDiagonal();
        K_.block<MPC_INPUT_DIM, MPC_INPUT_DIM>(mpc_step * MPC_INPUT_DIM, mpc_step * MPC_INPUT_DIM) = input_weight.asDiagonal();
    }
    linear_constraints_.setZero(MPC_CONSTRAINT_DIM * mpc_horizon_, MPC_INPUT_DIM * mpc_horizon_);

    for (size_t mpc_step = 0; mpc_step < mpc_horizon_; mpc_step++) {
        linear_constraints_.block<MPC_CONSTRAINT_DIM, MPC_INPUT_DIM>(MPC_CONSTRAINT_DIM * mpc_step, MPC_INPUT_DIM * mpc_step) =
            constraint_coefficient;
    }
}

void ConvexMPC::updateQP(const Eigen::Ref<const Eigen::Matrix<double, MPC_STATE_DIM, MPC_STATE_DIM>>& Ad,
                         const Eigen::Ref<const Eigen::Matrix<double, MPC_STATE_DIM, MPC_INPUT_DIM>>& Bd,
                         const Eigen::Ref<const Eigen::Vector<double, MPC_STATE_DIM>>& x0,
                         const Eigen::Ref<const Eigen::VectorXd>& y) {
    for (size_t mpc_step = 0; mpc_step < mpc_horizon_; mpc_step++) {
        if (mpc_step == 0) {
            A_qp_.block<MPC_STATE_DIM, MPC_STATE_DIM>(MPC_STATE_DIM * mpc_step, 0) = Ad;
        } else {
            A_qp_.block<MPC_STATE_DIM, MPC_STATE_DIM>(MPC_STATE_DIM * mpc_step, 0) =
                A_qp_.block<MPC_STATE_DIM, MPC_STATE_DIM>(MPC_STATE_DIM * (mpc_step - 1), 0) * Ad;
        }
        for (size_t idx = 0; idx < mpc_step + 1; idx++) {
            // diagonal term
            if (mpc_step - idx == 0) {
                B_qp_.block<MPC_STATE_DIM, MPC_INPUT_DIM>(MPC_STATE_DIM * mpc_step, MPC_INPUT_DIM * idx) = Bd;
            } else {
                B_qp_.block<MPC_STATE_DIM, MPC_INPUT_DIM>(MPC_STATE_DIM * mpc_step, MPC_INPUT_DIM * idx) =
                    A_qp_.block<MPC_STATE_DIM, MPC_STATE_DIM>(MPC_STATE_DIM * (mpc_step - idx - 1), 0) * Bd;
            }
        }
    }

    hessian_ = 2 * ((B_qp_.transpose() * L_ * B_qp_) + K_);
    gradient_ = 2 * B_qp_.transpose() * L_ * (A_qp_ * x0 - y);
}

const Eigen::VectorXd ConvexMPC::solve() {
    const qpOASES::int_t nV = MPC_INPUT_DIM * mpc_horizon_;       // Number of variables
    const qpOASES::int_t nC = MPC_CONSTRAINT_DIM * mpc_horizon_;  // Number of constraints

    std::vector<qpOASES::real_t> H(nV * nV);
    std::vector<qpOASES::real_t> A(nC * nV);
    std::vector<qpOASES::real_t> g(nV);
    std::vector<qpOASES::real_t> lbA(nC);
    std::vector<qpOASES::real_t> ubA(nC);

    Eigen::Map<Eigen::Matrix<qpOASES::real_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(H.data(), nV, nV) = hessian_;
    Eigen::Map<Eigen::Matrix<qpOASES::real_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(A.data(), nC, nV) =
        linear_constraints_;
    Eigen::Map<Eigen::Matrix<qpOASES::real_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(g.data(), nV, 1) = gradient_;
    Eigen::Map<Eigen::Matrix<qpOASES::real_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(lbA.data(), nC, 1) = lb_;
    Eigen::Map<Eigen::Matrix<qpOASES::real_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(ubA.data(), nC, 1) = ub_;

    qpOASES::SQProblem qp_solver(nV, nC);

    qpOASES::Options qp_option;
    qp_option.setToMPC();
    qp_option.enableRegularisation = qpOASES::BT_TRUE;
    qp_option.epsRegularisation = 1e-6;
    qp_option.printLevel = qpOASES::PL_NONE;
    qp_solver.setOptions(qp_option);

    qpOASES::int_t nWSR = 1000;
    std::vector<qpOASES::real_t> QPsolution(MPC_INPUT_DIM * mpc_horizon_);
    qpOASES::returnValue rv = qp_solver.init(H.data(), g.data(), A.data(), NULL, NULL, lbA.data(), ubA.data(), nWSR);
    if (rv != qpOASES::SUCCESSFUL_RETURN) std::fprintf(stderr, "[ConvexMPC] qpOASES init failed: %d (nWSR=%d)\n", int(rv), int(nWSR));
    qp_solver.getPrimalSolution(QPsolution.data());
    Eigen::VectorXd solution = Eigen::Map<Eigen::VectorXd>(QPsolution.data(), MPC_INPUT_DIM * mpc_horizon_);
    return solution;
}
}  // namespace ConvexMPC
