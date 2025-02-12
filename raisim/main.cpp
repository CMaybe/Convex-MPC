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

    raisim::ArticulatedSystem* ANYmal = world.addArticulatedSystem("./rsc/anymal_c/anymal.urdf");
    std::string leg_list[4] = {
        "LF_shank_fixed_LF_FOOT", "RF_shank_fixed_RF_FOOT", "LH_shank_fixed_LH_FOOT", "RH_shank_fixed_RH_FOOT"};
    Eigen::Matrix3d inertia;
    inertia.setZero();
    // clang-format off
    inertia <<   3.45791789e-01,  7.73636030e-03,  -9.35025187e-04, 
				 7.73636030e-03,  1.02294188e+00,   2.63540980e-03, 
				-9.35025187e-04,  2.63540980e-03,   7.89181283e-01;

    // clang-format on
    RobotModel robot_model(inertia,
                           52.13485,
                           params::gravity,
                           0.7,
                           mpc_dt,  // params::dt,
                           0,       // params::f_min,
                           666);    // params::f_max
    // params::f_max);
    Eigen::Vector3d cmd_vel;

    std::atomic<bool> control_execute{};
    control_execute.store(true, std::memory_order_release);

    Eigen::VectorXd jointNominalConfig(ANYmal->getGeneralizedCoordinateDim()), jointVelocityTarget(ANYmal->getDOF());
    Eigen::VectorXd current_jointVelocity(ANYmal->getDOF()), current_jointConfig(ANYmal->getGeneralizedCoordinateDim());
    Eigen::VectorXd jointPgain(ANYmal->getDOF()), jointDgain(ANYmal->getDOF()), jointForce(ANYmal->getDOF()),
        dynamics_torque(ANYmal->getDOF());
    std::array<Eigen::Vector3d, LEG_NUM> foot_position_b, foot_velocity;
    std::array<Eigen::Vector3d, LEG_NUM> grf, feedback, feedforward;
    std::array<Eigen::Vector3d, LEG_NUM> grf_torque, feedback_torque, feedforward_torque;
    Eigen::Vector3d body_position, euler, linear_velocity, angular_velocity;
    Eigen::Vector4d quat;
    Eigen::VectorXd q_weights(13), r_weights(12);

    jointPgain.setZero();
    jointDgain.setZero();
    jointVelocityTarget.setZero();
    jointNominalConfig << 0, 0, 0.54, 1, 0, 0, 0, 0.03, 0.4, -0.8, -0.03, 0.4, -0.8, 0.03, -0.4, 0.8, -0.03, -0.4, 0.8;
    q_weights << 20, 10, 1, 0, 0, 420, 0.05, 0.05, 0.05, 30, 30, 10, 0.;
    r_weights << 1e-7, 1e-7, 1e-7, 1e-7, 1e-7, 1e-7, 1e-7, 1e-7, 1e-7, 1e-7, 1e-7, 1e-7;

    ANYmal->setControlMode(raisim::ControlMode::PD_PLUS_FEEDFORWARD_TORQUE);
    ANYmal->setPdTarget(jointNominalConfig, jointVelocityTarget);
    ANYmal->setName("ANYmal");

    ANYmal->setGeneralizedCoordinate(jointNominalConfig);
    ANYmal->setPdGains(jointPgain, jointDgain);

    /// launch raisim server
    raisim::RaisimServer server(&world);
    server.focusOn(ANYmal);
    server.launchServer();

    current_jointConfig = ANYmal->getGeneralizedCoordinate().e();
    body_position = current_jointConfig.segment(0, 3);
    quat = current_jointConfig.segment(3, 4);
    euler = utils::quaternion_to_euler(quat);
    linear_velocity = current_jointVelocity.segment(0, 3);
    angular_velocity = current_jointVelocity.segment(3, 3);
    for (int leg_idx = 0; leg_idx < LEG_NUM; leg_idx++) {
        raisim::Vec<3> foot_position_w;
        ANYmal->getFramePosition(leg_list[leg_idx], foot_position_w);
        foot_position_b[leg_idx] = foot_position_w.e();
    }
    RobotState robot_state(euler, body_position, angular_velocity, linear_velocity, foot_position_b);
    RobotController robot_controller(robot_model, robot_state, q_weights, r_weights, 2000, 200);
    robot_state.updateContactState({true, false, false, true});

    ANYmal->printOutBodyNamesInOrder();

    ANYmal->printOutFrameNamesInOrder();
    std::cout << ANYmal->getDOF() << std::endl;
    /// mpc
    ANYmal->setGeneralizedCoordinate(jointNominalConfig);
    for (int i = 0; i < 10000000; i++) {
        RS_TIMED_LOOP(int(world.getTimeStep() * 1e6))
        current_jointConfig = ANYmal->getGeneralizedCoordinate().e();
        current_jointVelocity = ANYmal->getGeneralizedVelocity().e();
        body_position = current_jointConfig.segment(0, 3);
        quat = current_jointConfig.segment(3, 4);
        euler = utils::quaternion_to_euler(quat);
        linear_velocity = current_jointVelocity.segment(0, 3);
        angular_velocity = current_jointVelocity.segment(3, 3);
        dynamics_torque = ANYmal->getNonlinearities({0, 0, -9.81}).e();
        robot_state.updateState(euler, body_position, angular_velocity, linear_velocity);

        std::array<Eigen::Matrix3d, LEG_NUM> foot_jacobian;
        cmd_vel = Eigen::Vector3d(0, 0, 0);
        for (int leg_idx = 0; leg_idx < LEG_NUM; leg_idx++) {
            raisim::Vec<3> foot_position_w;
            Eigen::MatrixXd full_jacobian(3, ANYmal->getDOF());
            ANYmal->getFramePosition(leg_list[leg_idx], foot_position_w);
            ANYmal->getDenseFrameJacobian(leg_list[leg_idx], full_jacobian);
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
        ANYmal->setGeneralizedForce(jointForce);

        server.integrateWorldThreadSafe();
    }
    server.killServer();
}