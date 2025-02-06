#include "convex_mpc/convex_mpc.hpp"
#include <iostream>
namespace ConvexMPC {

ConvexMPC::ConvexMPC(const Eigen::Ref<const Eigen::Vector<double, MPC_STATE_DIM>>& state_weight,
                     const Eigen::Ref<const Eigen::Vector<double, MPC_INPUT_DIM>>& input_weight,
                     const Eigen::Ref<const Eigen::Vector<double, MPC_CONSTRAINT_DIM>>& lower_bound,
                     const Eigen::Ref<const Eigen::Vector<double, MPC_CONSTRAINT_DIM>>& upper_bound,
                     const Eigen::Ref<const Eigen::MatrixXd>& constraint_coefficient) {
    A_qp_.setZero(MPC_STATE_DIM * MPC_HORIZON, MPC_STATE_DIM);
    B_qp_.setZero(MPC_STATE_DIM * MPC_HORIZON, MPC_INPUT_DIM * MPC_HORIZON);
    lb_.setZero(MPC_CONSTRAINT_DIM * MPC_HORIZON);
    ub_.setZero(MPC_CONSTRAINT_DIM * MPC_HORIZON);
    L_.setZero(MPC_STATE_DIM * MPC_HORIZON, MPC_STATE_DIM * MPC_HORIZON);
    K_.setZero(MPC_INPUT_DIM * MPC_HORIZON, MPC_INPUT_DIM * MPC_HORIZON);
    gradient_.setZero(MPC_INPUT_DIM * MPC_HORIZON);
    hessian_.setZero(MPC_INPUT_DIM * MPC_HORIZON, MPC_INPUT_DIM * MPC_HORIZON);

    for (int mpc_step = 0; mpc_step < MPC_HORIZON; mpc_step++) {
        L_.block<MPC_STATE_DIM, MPC_STATE_DIM>(mpc_step * MPC_STATE_DIM, mpc_step * MPC_STATE_DIM) = state_weight.asDiagonal();
        K_.block<MPC_INPUT_DIM, MPC_INPUT_DIM>(mpc_step * MPC_INPUT_DIM, mpc_step * MPC_INPUT_DIM) = input_weight.asDiagonal();
    }
    linear_constraints_.setZero(MPC_CONSTRAINT_DIM * MPC_HORIZON, MPC_INPUT_DIM * MPC_HORIZON);

    for (int mpc_step = 0; mpc_step < MPC_HORIZON; mpc_step++) {
        linear_constraints_.block<MPC_CONSTRAINT_DIM, MPC_INPUT_DIM>(MPC_CONSTRAINT_DIM * mpc_step, MPC_INPUT_DIM * mpc_step) =
            constraint_coefficient;
    }

    for (int mpc_step = 0; mpc_step < MPC_HORIZON; mpc_step++) {
        lb_.segment<MPC_CONSTRAINT_DIM>(mpc_step * MPC_CONSTRAINT_DIM) << lower_bound;
        ub_.segment<MPC_CONSTRAINT_DIM>(mpc_step * MPC_CONSTRAINT_DIM) << upper_bound;
    }
}

void ConvexMPC::updateQP(const Eigen::Ref<const Eigen::Matrix<double, MPC_STATE_DIM, MPC_STATE_DIM>>& Ad,
                         const Eigen::Ref<const Eigen::Matrix<double, MPC_STATE_DIM, MPC_INPUT_DIM>>& Bd,
                         const Eigen::Ref<const Eigen::Vector<double, MPC_STATE_DIM>>& x0,
                         const Eigen::Ref<const Eigen::Vector<double, MPC_STATE_DIM * MPC_HORIZON>>& y) {
    for (int mpc_step = 0; mpc_step < MPC_HORIZON; mpc_step++) {
        if (mpc_step == 0) {
            A_qp_.block<MPC_STATE_DIM, MPC_STATE_DIM>(MPC_STATE_DIM * mpc_step, 0) = Ad;
        } else {
            A_qp_.block<MPC_STATE_DIM, MPC_STATE_DIM>(MPC_STATE_DIM * mpc_step, 0) =
                A_qp_.block<MPC_STATE_DIM, MPC_STATE_DIM>(MPC_STATE_DIM * (mpc_step - 1), 0) * Ad;
        }
        for (int idx = 0; idx < mpc_step + 1; idx++) {
            // diagonal term
            if (mpc_step - idx == 0) {
                B_qp_.block<MPC_STATE_DIM, MPC_INPUT_DIM>(MPC_STATE_DIM * mpc_step, MPC_INPUT_DIM * idx) = Bd;
            } else {
                B_qp_.block<MPC_STATE_DIM, MPC_INPUT_DIM>(MPC_STATE_DIM * mpc_step, MPC_INPUT_DIM * idx) =
                    A_qp_.block<MPC_STATE_DIM, MPC_STATE_DIM>(MPC_STATE_DIM * (mpc_step - idx - 1), 0) * Bd;
            }
        }
    }

    gradient_ = 2 * B_qp_.transpose() * L_ * (A_qp_ * x0 - y);
    hessian_ = 2 * ((B_qp_.transpose() * L_ * B_qp_) + K_);
}

const Eigen::VectorXd ConvexMPC::solve() {
    const qpOASES::int_t nV = MPC_INPUT_DIM * MPC_HORIZON;       // Number of variables
    const qpOASES::int_t nC = MPC_CONSTRAINT_DIM * MPC_HORIZON;  // Number of constraints

    qpOASES::real_t H[MPC_INPUT_DIM * MPC_HORIZON * MPC_INPUT_DIM * MPC_HORIZON];
    qpOASES::real_t A[MPC_CONSTRAINT_DIM * MPC_HORIZON * MPC_INPUT_DIM * MPC_HORIZON];
    qpOASES::real_t g[MPC_INPUT_DIM * MPC_HORIZON];
    qpOASES::real_t lbA[MPC_CONSTRAINT_DIM * MPC_HORIZON];
    qpOASES::real_t ubA[MPC_CONSTRAINT_DIM * MPC_HORIZON];

    Eigen::Map<Eigen::Matrix<qpOASES::real_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(H, nV, nV) = hessian_;
    Eigen::Map<Eigen::Matrix<qpOASES::real_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(A, nC, nV) = linear_constraints_;
    Eigen::Map<Eigen::Matrix<qpOASES::real_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(g, nV, 1) = gradient_;
    Eigen::Map<Eigen::Matrix<qpOASES::real_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(lbA, nC, 1) = lb_;
    Eigen::Map<Eigen::Matrix<qpOASES::real_t, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(ubA, nC, 1) = ub_;

    qpOASES::SQProblem qp_solver(nV, nC);

    qpOASES::Options qp_option;
    qp_option.setToMPC();
    qp_option.enableRegularisation = qpOASES::BT_TRUE;
    qp_option.epsRegularisation = 1e-6;
    qp_option.printLevel = qpOASES::PL_NONE;
    qp_solver.setOptions(qp_option);

    qpOASES::int_t nWSR = 1000;
    qpOASES::real_t QPsolution[MPC_INPUT_DIM * MPC_HORIZON];
    qp_solver.init(H, g, A, NULL, NULL, lbA, ubA, nWSR);
    qp_solver.getPrimalSolution(QPsolution);
    Eigen::VectorXd solution = Eigen::Map<Eigen::VectorXd>(QPsolution, MPC_INPUT_DIM * MPC_HORIZON);
    return solution;
}
}  // namespace ConvexMPC
