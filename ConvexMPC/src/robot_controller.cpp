#include "convex_mpc/robot_controller.hpp"

#include <iostream>
namespace ConvexMPC {
RobotController::RobotController(const RobotModel& robot_model) : robot_model_(robot_model) {
    q_weights_ << 80.0, 80.0, 1.0, 0.0, 0.0, 270.0, 1.0, 1.0, 20.0, 20.0, 20.0, 20.0, 0.0;
    r_weights_ << 1e-5, 1e-5, 1e-6, 1e-5, 1e-5, 1e-6, 1e-5, 1e-5, 1e-6, 1e-5, 1e-5, 1e-6;
    constraint_coefficient_.resize(5, 3);
    // clang-format off
    constraint_coefficient_ <<
    1, 0, robot_model.mu(),
    1, 0, -robot_model.mu(),
    0, 1, robot_model.mu(),
    0, 1, -robot_model.mu(),
    0, 0, 1;
    // clang-format on
    Kp_ = Eigen::Matrix3d::Identity() * 100;
    Kd_ = Eigen::Matrix3d::Identity() * 10;
}

RobotController::RobotController(const RobotModel& robot_model, const RobotState& nominal_state)
    : robot_model_(robot_model), robot_nominal_state_(nominal_state) {
    q_weights_ << 80.0, 80.0, 1.0, 0.0, 0.0, 270.0, 1.0, 1.0, 20.0, 20.0, 20.0, 20.0, 0.0;
    r_weights_ << 1e-5, 1e-5, 1e-6, 1e-5, 1e-5, 1e-6, 1e-5, 1e-5, 1e-6, 1e-5, 1e-5, 1e-6;
    constraint_coefficient_.resize(5, 3);
    // clang-format off
    constraint_coefficient_ <<
    1, 0, robot_model.mu(),
    1, 0, -robot_model.mu(),
    0, 1, robot_model.mu(),
    0, 1, -robot_model.mu(),
    0, 0, 1;
    // clang-format on

    Kp_ = Eigen::Matrix3d::Identity() * 100;
    Kd_ = Eigen::Matrix3d::Identity() * 10;
}

std::array<Eigen::Vector3d, LEG_NUM> RobotController::computeGRF(const RobotState& robot_state,
                                                                 const Eigen::Ref<const Eigen::Vector3d>& cmd_vel_b) {
    std::array<Eigen::Vector3d, LEG_NUM> grf;
    Eigen::Vector<double, MPC_STATE_DIM * MPC_HORIZON> mpc_states_d;
    std::array<Eigen::Vector3d, LEG_NUM> foot_positions_w;

    double mpc_dt = robot_model_.dt();
    Eigen::Vector3d euler = robot_state.euler_angle();
    Eigen::Vector3d position = robot_state.position();
    Eigen::Vector3d linear_velocity = {cmd_vel_b[0], cmd_vel_b[1], 0};
    Eigen::Vector3d linear_velocity_w = robot_state.rotation_matrix() * linear_velocity;

    for (int mpc_step = 0; mpc_step < MPC_HORIZON; mpc_step++) {
        // clang-format off
        mpc_states_d.segment(mpc_step * MPC_STATE_DIM, MPC_STATE_DIM) << 
        euler[0], euler[1], euler[2] + cmd_vel_b[2] * mpc_dt * (mpc_step + 1),
        position[0] + linear_velocity_w[0] * mpc_dt * (mpc_step + 1), position[1] + linear_velocity_w[1] * mpc_dt * (mpc_step + 1), position[2], 
        0, 0, cmd_vel_b[2], 
        linear_velocity_w[0], linear_velocity_w[1], 0, 
        robot_model_.gravity();
        // clang-format on
    }
    for (int leg_idx = 0; leg_idx < LEG_NUM; leg_idx++) {
        foot_positions_w[leg_idx] = position + robot_state.foot_position(leg_idx);
    }

    robot_model_.updateAc(robot_state.euler_angle());
    robot_model_.updateBc(robot_state.rotation_matrix(), foot_positions_w);
    robot_model_.updateDiscretizedModel();
    lower_bound_ << 0, -qpOASES::INFTY, 0, -qpOASES::INFTY, robot_model_.f_min();
    upper_bound_ << qpOASES::INFTY, 0, qpOASES::INFTY, 0, robot_model_.f_max();

    ConvexMPC mpc_problem(q_weights_, r_weights_, lower_bound_, upper_bound_, constraint_coefficient_);
    mpc_problem.updateQP(robot_model_.Ad(), robot_model_.Bd(), robot_state.mpc_state(), mpc_states_d);
    Eigen::VectorXd solution = mpc_problem.solve();
    std::cout << "\n==========================================\n";
    std::cout << solution.segment(0, MPC_INPUT_DIM).transpose() << "\n";
    std::cout << "\n==========================================\n" << std::endl;

    for (int leg_idx = 0; leg_idx < LEG_NUM; leg_idx++) {
        if (!isnan(solution.segment<3>(leg_idx * 3).norm()))
            grf[leg_idx] = robot_state.rotation_matrix().transpose() * solution.segment<3>(leg_idx * 3);
    }

    return grf;
}

std::array<Eigen::Vector3d, LEG_NUM> RobotController::computeSwingForce(const RobotState& robot_state,
                                                                        const Eigen::Ref<const Eigen::Vector3d>& cmd_vel_b) {
    std::array<Eigen::Vector3d, LEG_NUM> result;
    std::array<Eigen::Vector3d, LEG_NUM> foot_position_d;

    Eigen::Matrix3d R = robot_state.rotation_matrix();
    Eigen::Matrix3d R_T = R.transpose();
    Eigen::Vector3d cmd_vel_w = R * cmd_vel_b;
    cmd_vel_w[2] = 0;
    Eigen::Vector3d body_position_w = robot_state.position();

    for (int leg_idx = 0; leg_idx < LEG_NUM; leg_idx++) {
        Eigen::Vector3d hip_position_b = robot_nominal_state_.foot_position(leg_idx);
        Eigen::Vector3d p_ref = body_position_w + hip_position_b;
        p_ref[2] = 0;
        stance_counter_[leg_idx] += 0.01;
        foot_position_d[leg_idx] = p_ref + cmd_vel_w * stance_duration_ / 2;
        Eigen::Vector3d foot_position_w = robot_state.foot_position(leg_idx) + body_position_w;
        // if (stance_counter_[leg_idx] < stance_duration_) {
        //     double s = swing_counter_[leg_idx] / swing_duration_;
        //     for (int i = 0; i < 3; i++) {
        //         foot_position_d[leg_idx](i) = utils::bezier_curve(s,
        //                                                           {foot_position_w(i),
        //                                                            foot_position_w(i),
        //                                                            foot_position_d[leg_idx](i),
        //                                                            foot_position_d[leg_idx](i),
        //                                                            foot_position_d[leg_idx](i)});
        //     }

        //     // Todo
        //     // Add disired foot velocity
        // }

        result[leg_idx] =
            (Kp_ * R_T * (foot_position_d[leg_idx] - foot_position_w) + Kd_ * (-robot_state.foot_velocity(leg_idx)));
    }

    return result;
}

}  // namespace ConvexMPC