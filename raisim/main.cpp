#include "raisim/RaisimServer.hpp"
#include "raisim/World.hpp"

#include "convex_mpc/params.hpp"
#include "convex_mpc/robot_controller.hpp"
#include "convex_mpc/robot_model.hpp"
#include "convex_mpc/robot_state.hpp"

using namespace ConvexMPC;

int main() {
    raisim::World world;
    world.setTimeStep(0.001);
    auto ground = world.addGround();

    raisim::ArticulatedSystem* laikago = world.addArticulatedSystem("./rsc/laikago/laikago.urdf");
    std::string leg_list[4] = {"FR_foot_fixed", "FL_foot_fixed", "RR_foot_fixed", "RL_foot_fixed"};
    Eigen::Matrix3d inertia;
    inertia.setZero();
    inertia(0, 0) = 0.073348887;
    inertia(1, 1) = 0.250684593;
    inertia(2, 2) = 0.254469458;
    RobotModel robot_model(inertia,
                           13.733,  // laikago->getMass()[0],
                           params::gravity,
                           params::mu,
                           0.05,  // params::dt,
                           4,     // params::f_min,
                           666);
    // params::f_max);
    Eigen::Vector3d cmd_vel;

    std::atomic<bool> control_execute{};
    control_execute.store(true, std::memory_order_release);

    Eigen::VectorXd jointNominalConfig(laikago->getGeneralizedCoordinateDim()), jointVelocityTarget(laikago->getDOF());
    Eigen::VectorXd current_jointVelocity(laikago->getDOF()), current_jointConfig(laikago->getGeneralizedCoordinateDim());
    Eigen::VectorXd jointPgain(laikago->getDOF()), jointDgain(laikago->getDOF()), jointForce(laikago->getDOF()),
        dynamics_torque(laikago->getDOF());
    std::array<Eigen::Vector3d, LEG_NUM> foot_position_b, foot_velocity;
    std::array<Eigen::Vector3d, LEG_NUM> grf, feedback, feedforward;
    std::array<Eigen::Vector3d, LEG_NUM> grf_torque, feedback_torque, feedforward_torque;
    Eigen::Vector3d body_position, euler, linear_velocity, angular_velocity;
    Eigen::Vector4d quat;
    Eigen::VectorXd q_weights(13), r_weights(12);

    jointPgain.setZero();
    jointDgain.setZero();
    jointVelocityTarget.setZero();
    jointNominalConfig << 0, 0, 0.49, 1, 0.0, 0.0, 0.0, 0.0, 0.5, -1, 0, 0.5, -1, 0.00, 0.5, -1, 0, 0.5, -0.7;
    q_weights << 5., 5., 50., 10., 10., 50., 0.01, 0.01, 0.2, 0.2, 0.2, 0.2, 0.;
    r_weights << 1e-5, 1e-5, 1e-5, 1e-5, 1e-5, 1e-5, 1e-5, 1e-5, 1e-5, 1e-5, 1e-5, 1e-5;

    laikago->setControlMode(raisim::ControlMode::PD_PLUS_FEEDFORWARD_TORQUE);
    laikago->setPdTarget(jointNominalConfig, jointVelocityTarget);
    laikago->setName("laikago");

    laikago->setGeneralizedCoordinate(jointNominalConfig);
    laikago->setPdGains(jointPgain, jointDgain);

    /// launch raisim server
    raisim::RaisimServer server(&world);
    server.focusOn(laikago);
    server.launchServer();

    current_jointConfig = laikago->getGeneralizedCoordinate().e();
    body_position = current_jointConfig.segment(0, 3);
    quat = current_jointConfig.segment(3, 4);
    euler = {0, 0, 0};  // utils::quaternion_to_euler(quat);
    linear_velocity = current_jointVelocity.segment(0, 3);
    angular_velocity = current_jointVelocity.segment(3, 3);
    for (int leg_idx = 0; leg_idx < LEG_NUM; leg_idx++) {
        raisim::Vec<3> foot_position_w;
        laikago->getFramePosition(leg_list[leg_idx], foot_position_w);
        foot_position_b[leg_idx] = foot_position_w.e() - body_position;
    }

    RobotState robot_state(euler, body_position, angular_velocity, linear_velocity, foot_position_b);
    RobotController robot_controller(robot_model, robot_state, q_weights, r_weights, 2000, 300);
    robot_state.updateContactState({true, true, false, false});

    /// mpc
    laikago->setGeneralizedCoordinate(jointNominalConfig);
    for (int i = 0; i < 10000000; i++) {
        RS_TIMED_LOOP(int(world.getTimeStep() * 1e6))
        current_jointConfig = laikago->getGeneralizedCoordinate().e();
        current_jointVelocity = laikago->getGeneralizedVelocity().e();
        body_position = current_jointConfig.segment(0, 3);
        quat = current_jointConfig.segment(3, 4);
        euler = utils::quaternion_to_euler(quat);
        std::cout << euler.transpose() << std::endl;
        linear_velocity = current_jointVelocity.segment(0, 3);
        angular_velocity = current_jointVelocity.segment(3, 3);
        dynamics_torque = laikago->getNonlinearities({0, 0, -9.81}).e();
        robot_state.updateState(euler, body_position, angular_velocity, linear_velocity);

        std::array<Eigen::Matrix3d, LEG_NUM> foot_jacobian;
        cmd_vel = Eigen::Vector3d(0.3, 0, 0);

        for (int leg_idx = 0; leg_idx < LEG_NUM; leg_idx++) {
            raisim::Vec<3> foot_position_w;
            Eigen::MatrixXd full_jacobian(3, laikago->getDOF());
            laikago->getFramePosition(leg_list[leg_idx], foot_position_w);
            laikago->getDenseFrameJacobian(leg_list[leg_idx], full_jacobian);
            foot_jacobian[leg_idx] = full_jacobian.block(0, 6 + 3 * leg_idx, 3, 3);
            foot_position_b[leg_idx] = foot_position_w.e() - body_position;
            foot_velocity[leg_idx] = foot_jacobian[leg_idx] * current_jointVelocity.segment(6 + 3 * leg_idx, 3);
        }

        robot_state.updateFootPosition(foot_position_b);
        robot_state.updateFootVelocity(foot_velocity);
        if (i % 50 == 0) {
            grf = robot_controller.computeGRF(robot_state, cmd_vel);
            for (int leg_idx = 0; leg_idx < LEG_NUM; leg_idx++) {
                grf_torque[leg_idx] = -foot_jacobian[leg_idx].transpose() * grf[leg_idx];
            }
        }
        feedback = robot_controller.computeSwingForce(robot_state, cmd_vel);
        for (int leg_idx = 0; leg_idx < LEG_NUM; leg_idx++) {
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
        laikago->setGeneralizedForce(jointForce);

        server.integrateWorldThreadSafe();
    }
    server.killServer();
}