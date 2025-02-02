
#include "convex_mpc/robot_controller.hpp"
namespace ConvexMPC {
RobotController::RobotController(const RobotModel &robot_model) : robot_model_(robot_model) {
    q_weights_ << 80.0, 80.0, 1.0, 0.0, 0.0, 270.0, 1.0, 1.0, 20.0, 20.0, 20.0, 20.0, 0.0;
    r_weights_ << 1e-5, 1e-5, 1e-6, 1e-5, 1e-5, 1e-6, 1e-5, 1e-5, 1e-6, 1e-5, 1e-5, 1e-6;
    constraint_coefficient_.resize(5, 3);
    // clang-format off
	constraint_coefficient_ <<
	1, 0, robot_model.mu(),
	1, 0, -robot_model.mu(),
	1, 0, robot_model.mu(),
	1, 0, -robot_model.mu(),
	0, 0, 1;
    // clang-format on	
}

std::array<Eigen::Vector3d, LEG_NUM> RobotController::compute_grf() {
    std::array<Eigen::Vector3d, LEG_NUM> grf;
    Eigen::Vector<double, MPC_STATE_DIM * MPC_HORIZON> mpc_states_d;

    double mpc_dt = robot_model_.dt();
    Eigen::Vector3d euler = robot_desired_state_.euler_angle();
    Eigen::Vector3d position = robot_desired_state_.position();
    Eigen::Vector3d linear_velocity = robot_desired_state_.linear_velocity();
    Eigen::Vector3d angular_veloicty = robot_desired_state_.angular_velocity();
    Eigen::Vector3d linear_velocity_w = robot_state_.rotation_matrix() * linear_velocity;

    for (int mpc_step = 0; mpc_step < MPC_HORIZON; mpc_step++) {
        // clang-format off
        mpc_states_d.segment(mpc_step * MPC_STATE_DIM, MPC_STATE_DIM) << 
        euler[0], euler[1], euler[2] + angular_veloicty[2] * mpc_dt * (mpc_step + 1),
        position[0] + linear_velocity_w[0] * mpc_dt * (mpc_step + 1), position[1] + linear_velocity_w[1] * mpc_dt * (mpc_step + 1), position[2], 
        angular_veloicty[0], angular_veloicty[1], angular_veloicty[2], 
        linear_velocity_w[0], linear_velocity_w[1], 0, 
        params::gravity;
        // clang-format on
    }

    robot_model_.updateAc(robot_state_.euler_angle());
    robot_model_.updateBc(robot_state_.rotation_matrix(), robot_state_.foot_pos());
    robot_model_.updateDiscretizedModel();

    ConvexMPC mpc_problem(q_weights_, r_weights_, lower_bound_, upper_bound_, constraint_coefficient_);
    mpc_problem.update_qp(robot_model_.Ad(), robot_model_.Bd(), robot_state_.mpc_state(), mpc_states_d);

    Eigen::VectorXd solution = mpc_problem.solve_qp();
    for (int i = 0; i < LEG_NUM; i++) {
        if (!isnan(solution.segment<3>(i * 3).norm()))
            grf[i] = robot_state_.rotation_matrix().transpose() * solution.segment<3>(i * 3);
    }

    return grf;
}

}  // namespace ConvexMPC