#include "raisim/RaisimServer.hpp"
#include "raisim/World.hpp"

#include "convex_mpc/params.hpp"
#include "convex_mpc/robot_controller.hpp"
#include "convex_mpc/robot_model.hpp"
#include "convex_mpc/robot_state.hpp"

using namespace ConvexMPC;

int main() {
    raisim::World world;

    double mpc_dt = 0.05;
    double time_step = 0.001;
    world.setTimeStep(time_step);
    auto ground = world.addGround();

    raisim::ArticulatedSystem* a1 = world.addArticulatedSystem("./rsc/a1/a1.urdf");
    std::string leg_list[4] = {"FR_foot_fixed", "FL_foot_fixed", "RR_foot_fixed", "RL_foot_fixed"};
    Eigen::Matrix3d inertia;
    inertia.setZero();
    // clang-format off
    inertia <<  0.0158533, 0.0, 0.0, 
				0.0, 0.0377999, 0.0, 
				0.0, 0.0, 0.0456542;

    // clang-format on
    RobotModel robot_model(inertia,
                           12,
                           params::gravity,
                           0.7,
                           mpc_dt,  // params::dt,
                           0,       // params::f_min,
                           180);    // params::f_max
    // params::f_max);
    Eigen::Vector3d cmd_vel;

    std::atomic<bool> control_execute{};
    control_execute.store(true, std::memory_order_release);

    Eigen::VectorXd jointNominalConfig(a1->getGeneralizedCoordinateDim()), jointVelocityTarget(a1->getDOF());
    Eigen::VectorXd current_jointVelocity(a1->getDOF()), current_jointConfig(a1->getGeneralizedCoordinateDim());
    Eigen::VectorXd jointPgain(a1->getDOF()), jointDgain(a1->getDOF()), jointForce(a1->getDOF()), dynamics_torque(a1->getDOF());
    std::array<Eigen::Vector3d, LEG_NUM> foot_position_b, foot_velocity;
    std::array<Eigen::Vector3d, LEG_NUM> grf, feedback, feedforward;
    std::array<Eigen::Vector3d, LEG_NUM> grf_torque, feedback_torque, feedforward_torque;
    Eigen::Vector3d body_position, euler, linear_velocity, angular_velocity;
    Eigen::Vector4d quat;
    Eigen::VectorXd q_weights(13), r_weights(12);

    jointPgain.setZero();
    jointDgain.setZero();
    jointVelocityTarget.setZero();
    jointNominalConfig << 0, 0, 0.35, 1, 0.0, 0.0, 0.0, 0.0, 0.5, -1, 0, 0.5, -1, 0.0, 0.5, -1, 0, 0.5, -1;
    q_weights << 20, 10, 1, 0, 0, 420, 0.05, 0.05, 0.05, 30, 30, 10, 0.;
    r_weights << 1e-7, 1e-7, 1e-7, 1e-7, 1e-7, 1e-7, 1e-7, 1e-7, 1e-7, 1e-7, 1e-7, 1e-7;

    a1->setControlMode(raisim::ControlMode::PD_PLUS_FEEDFORWARD_TORQUE);
    a1->setPdTarget(jointNominalConfig, jointVelocityTarget);
    a1->setName("a1");

    a1->setGeneralizedCoordinate(jointNominalConfig);
    a1->setPdGains(jointPgain, jointDgain);

    /// launch raisim server
    raisim::RaisimServer server(&world);
    server.focusOn(a1);
    server.launchServer();

    current_jointConfig = a1->getGeneralizedCoordinate().e();
    body_position = current_jointConfig.segment(0, 3);
    quat = current_jointConfig.segment(3, 4);
    euler = utils::quaternion_to_euler(quat);
    linear_velocity = current_jointVelocity.segment(0, 3);
    angular_velocity = current_jointVelocity.segment(3, 3);
    foot_position_b[0] << 0.17, -0.15, -0.35;
    foot_position_b[1] << 0.17, 0.15, -0.35;
    foot_position_b[2] << -0.17, -0.15, -0.35;
    foot_position_b[3] << -0.17, 0.15, -0.35;

    RobotState robot_state(euler, body_position, angular_velocity, linear_velocity, foot_position_b);
    RobotController robot_controller(robot_model, robot_state, q_weights, r_weights, 200, 30);
    robot_state.updateContactState({false, true, true, false});

    /// mpc
    a1->setGeneralizedCoordinate(jointNominalConfig);
    for (int i = 0; i < 10000000; i++) {
        RS_TIMED_LOOP(int(world.getTimeStep() * 10 * 1e6))
        current_jointConfig = a1->getGeneralizedCoordinate().e();
        current_jointVelocity = a1->getGeneralizedVelocity().e();
        body_position = current_jointConfig.segment(0, 3);
        quat = current_jointConfig.segment(3, 4);
        euler = utils::quaternion_to_euler(quat);
        linear_velocity = current_jointVelocity.segment(0, 3);
        angular_velocity = current_jointVelocity.segment(3, 3);
        dynamics_torque = a1->getNonlinearities({0, 0, -9.81}).e();
        robot_state.updateState(euler, body_position, angular_velocity, linear_velocity);

        std::array<Eigen::Matrix3d, LEG_NUM> foot_jacobian;
        cmd_vel = Eigen::Vector3d(0, 0, 0);

        for (int leg_idx = 0; leg_idx < LEG_NUM; leg_idx++) {
            raisim::Vec<3> foot_position_w;
            Eigen::MatrixXd full_jacobian(3, a1->getDOF());
            a1->getFramePosition(leg_list[leg_idx], foot_position_w);
            a1->getDenseFrameJacobian(leg_list[leg_idx], full_jacobian);
            foot_position_w[2] -= 0.02;  // foot size;
            foot_jacobian[leg_idx] = full_jacobian.block(0, 6 + 3 * leg_idx, 3, 3);
            foot_position_b[leg_idx] = foot_position_w.e() - body_position;
            foot_velocity[leg_idx] = foot_jacobian[leg_idx] * current_jointVelocity.segment(6 + 3 * leg_idx, 3);
        }

        robot_state.updateFootPosition(foot_position_b);
        robot_state.updateFootVelocity(foot_velocity);
        if (std::fmod(i, (mpc_dt / time_step)) == 0) {
            grf = robot_controller.computeGRF(robot_state, cmd_vel);
        }
        feedback = robot_controller.computeSwingForce(robot_state, cmd_vel);
        for (int leg_idx = 0; leg_idx < LEG_NUM; leg_idx++) {
            grf_torque[leg_idx] = -foot_jacobian[leg_idx].transpose() * grf[leg_idx];
            feedforward[leg_idx] = dynamics_torque.segment(6 + 3 * leg_idx, 3);
            feedback_torque[leg_idx] = foot_jacobian[leg_idx].transpose() * feedback[leg_idx];
        }
        // clang-format off
        jointForce << Eigen::VectorXd::Zero(6), 
						grf_torque[0] + feedback_torque[0] + feedforward[0], 
						grf_torque[1] + feedback_torque[1] + feedforward[1],
						grf_torque[2] + feedback_torque[2] + feedforward[2],
						grf_torque[3] + feedback_torque[3] + feedforward[3];
        // clang-format on
        a1->setGeneralizedForce(jointForce);

        server.integrateWorldThreadSafe();
    }
    server.killServer();
}