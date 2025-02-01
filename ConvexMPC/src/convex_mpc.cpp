#include "convex_mpc/convex_mpc.hpp"
namespace ConvexMPC {

ConvexMPC::ConvexMPC(const RobotModel& robot_model,
                     const RobotState& robot_state,
                     const RobotState& robot_desired_state,
                     const Eigen::Ref<const Eigen::Vector<double, MPC_STATE_DIM>>& state_weight,
                     const Eigen::Ref<const Eigen::Vector<double, MPC_INPUT_DIM>>& input_weight)
    : robot_model_(robot_model), robot_state_(robot_state), robot_desired_state_(robot_desired_state) {
    A_qp_.resize(MPC_STATE_DIM * MPC_HORIZON, MPC_STATE_DIM);
    B_qp_.resize(MPC_STATE_DIM * MPC_HORIZON, MPC_INPUT_DIM * MPC_HORIZON);
    gradient_.resize(MPC_INPUT_DIM * MPC_HORIZON);
    lb_.resize(MPC_INPUT_DIM * MPC_HORIZON);
    ub_.resize(MPC_INPUT_DIM * MPC_HORIZON);

    Q_.resize(MPC_STATE_DIM * MPC_HORIZON, MPC_STATE_DIM * MPC_HORIZON);
    R_.resize(MPC_INPUT_DIM * MPC_HORIZON, MPC_INPUT_DIM * MPC_HORIZON);

    for (int mpc_step = 0; mpc_step < MPC_HORIZON; mpc_step++) {
        Q_.block<MPC_STATE_DIM, MPC_STATE_DIM>(mpc_step * MPC_STATE_DIM, mpc_step * MPC_STATE_DIM) =
            state_weight.asDiagonal();
        R_.block<MPC_INPUT_DIM, MPC_INPUT_DIM>(mpc_step * MPC_INPUT_DIM, mpc_step * MPC_INPUT_DIM) =
            input_weight.asDiagonal();
    }

    linear_constraints_.resize(MPC_CONSTRAINT_DIM * MPC_HORIZON, MPC_INPUT_DIM * MPC_HORIZON);
    for (int i = 0; i < LEG_NUM * MPC_HORIZON; i++) {
        linear_constraints_.insert(0 + 5 * i, 0 + 3 * i) = 1;
        linear_constraints_.insert(1 + 5 * i, 0 + 3 * i) = 1;
        linear_constraints_.insert(2 + 5 * i, 1 + 3 * i) = 1;
        linear_constraints_.insert(3 + 5 * i, 1 + 3 * i) = 1;
        linear_constraints_.insert(4 + 5 * i, 2 + 3 * i) = 1;

        linear_constraints_.insert(0 + 5 * i, 2 + 3 * i) = robot_model_.mu();
        linear_constraints_.insert(1 + 5 * i, 2 + 3 * i) = -robot_model_.mu();
        linear_constraints_.insert(2 + 5 * i, 2 + 3 * i) = robot_model_.mu();
        linear_constraints_.insert(3 + 5 * i, 2 + 3 * i) = -robot_model_.mu();
    }
}

void ConvexMPC::updateQP(const Eigen::Ref<const Eigen::Vector<double, MPC_STATE_DIM>>& x0,
                         const Eigen::Ref<const Eigen::Vector<double, MPC_STATE_DIM>>& y) {
    Eigen::MatrixXd Ad = robot_model_.Ad();
    Eigen::MatrixXd Bd = robot_model_.Bd();
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

    hessian_ = 2 * ((B_qp_.transpose() * Q_ * B_qp_) + R_);
    gradient_ = 2 * B_qp_.transpose() * Q_ * (A_qp_ * x0 - y);

    for (int mpc_step = 0; mpc_step < MPC_HORIZON; mpc_step++) {
        for (int leg_idx = 0; leg_idx < LEG_NUM; leg_idx++) {
            // clang-format off
            lb_.segment<5>(mpc_step * MPC_CONSTRAINT_DIM + 5 * leg_idx) << 0, -qpOASES::INFTY, 0, -qpOASES::INFTY, params::f_min;
            ub_.segment<5>(mpc_step * MPC_CONSTRAINT_DIM + 5 * leg_idx) << qpOASES::INFTY, 0, qpOASES::INFTY, 0, params::f_max;
            // clang-format on
        }
    }
}

}  // namespace ConvexMPC