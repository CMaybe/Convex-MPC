#include "raisim/RaisimServer.hpp"
#include "raisim/World.hpp"

#include "convex_mpc/params.hpp"
#include "convex_mpc/robot_controller.hpp"
#include "convex_mpc/robot_model.hpp"
#include "convex_mpc/robot_state.hpp"

using namespace ConvexMPC;

int main() {
    raisim::World world;
    world.setTimeStep(0.01);
    world.addGround();

    raisim::ArticulatedSystem* laikago = world.addArticulatedSystem("./rsc/laikago/laikago.urdf");
    std::string leg_list[4] = {"FR_foot_fixed", "FL_foot_fixed", "RR_foot_fixed", "RL_foot_fixed"};
    Eigen::Matrix3d inertia;
    inertia.setZero();
    inertia(0, 0) = 0.41;
    inertia(1, 1) = 2.1;
    inertia(2, 2) = 2.1;
    RobotModel robot_model(inertia,
                           43,  // laikago->getMass()[0],
                           params::gravity,
                           params::mu,
                           params::dt,
                           params::f_min,
                           params::f_max);
    Eigen::Vector3d cmd_vel;

    std::atomic<bool> control_execute{};
    control_execute.store(true, std::memory_order_release);

    Eigen::VectorXd jointNominalConfig(laikago->getGeneralizedCoordinateDim()), jointVelocityTarget(laikago->getDOF());
    Eigen::VectorXd current_jointVelocity(laikago->getDOF()), current_jointConfig(laikago->getGeneralizedCoordinateDim());
    Eigen::VectorXd jointPgain(laikago->getDOF()), jointDgain(laikago->getDOF()), jointForce(laikago->getDOF()),
        dynamics_torque(laikago->getDOF());
    std::array<Eigen::Vector3d, LEG_NUM> foot_position, foot_velocity;
    std::array<Eigen::Vector3d, LEG_NUM> grf, feedback, feedforward;
    Eigen::Vector3d position, euler, linear_velocity, angular_velocity;
    Eigen::Vector4d quat;

    jointPgain.setZero();
    jointDgain.setZero();
    jointVelocityTarget.setZero();
    jointNominalConfig << 0, 0, 0.48, 1, 0.0, 0.0, 0.0, 0.0, 0.5, -1, 0, 0.5, -1, 0.00, 0.5, -1, 0, 0.5, -0.7;

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
    position = current_jointConfig.segment(0, 3);
    quat = current_jointConfig.segment(3, 4);
    euler = {0, 0, 0};  // utils::quaternion_to_euler(quat);
    linear_velocity = current_jointVelocity.segment(0, 3);
    angular_velocity = current_jointVelocity.segment(3, 3);
    for (int leg_idx = 0; leg_idx < LEG_NUM; leg_idx++) {
        raisim::Vec<3> foot_pos;
        laikago->getFramePosition(leg_list[leg_idx], foot_pos);
        foot_position[leg_idx] = foot_pos.e() - position;
    }

    RobotState robot_state(euler, position, angular_velocity, linear_velocity, foot_position);
    RobotController robot_controller(robot_model, robot_state, 20, 3);
    robot_state.setContactState({false, true, true, false});

    /// mpc
    // 몸이 아래로 떨어지는거 고려해야 할듯
    // state에서 항상 몸이 고정

    laikago->setGeneralizedCoordinate(jointNominalConfig);
    RS_TIMED_LOOP(int(world.getTimeStep() * 100 * 1e6))
    for (int i = 0; i < 10000000; i++) {
        RS_TIMED_LOOP(int(world.getTimeStep() * 1e6))
        current_jointConfig = laikago->getGeneralizedCoordinate().e();
        current_jointVelocity = laikago->getGeneralizedVelocity().e();
        position = current_jointConfig.segment(0, 3);
        quat = current_jointConfig.segment(3, 4);
        euler = {0, 0, 0};  // utils::quaternion_to_euler(quat);
        linear_velocity = current_jointVelocity.segment(0, 3);
        angular_velocity = current_jointVelocity.segment(3, 3);
        dynamics_torque = laikago->getNonlinearities({0, 0, -9.81}).e();
        robot_state.updateState(euler, position, angular_velocity, linear_velocity);

        std::array<Eigen::Matrix3d, LEG_NUM> foot_jacobian;
        cmd_vel = Eigen::Vector3d(0.0, 0, 0);

        for (int leg_idx = 0; leg_idx < LEG_NUM; leg_idx++) {
            raisim::Vec<3> foot_pos;
            Eigen::MatrixXd full_jacobian(3, laikago->getDOF());
            laikago->getFramePosition(leg_list[leg_idx], foot_pos);
            laikago->getDenseFrameJacobian(leg_list[leg_idx], full_jacobian);
            foot_jacobian[leg_idx] = full_jacobian.block(0, 6 + 3 * leg_idx, 3, 3);
            foot_position[leg_idx] = foot_pos.e() - position;
            foot_velocity[leg_idx] = foot_jacobian[leg_idx] * current_jointVelocity.segment(6 + 3 * leg_idx, 3);
        }

        robot_state.updateFootPosition(foot_position);
        robot_state.updateFootVelocity(foot_velocity);
        grf = robot_controller.computeGRF(robot_state, cmd_vel);
        feedback = robot_controller.computeSwingForce(robot_state, cmd_vel);
        for (int leg_idx = 0; leg_idx < LEG_NUM; leg_idx++) {
            feedforward[leg_idx].setZero();
            if (robot_state.contact_state(leg_idx) == false) {
                feedforward[leg_idx] = dynamics_torque.segment(6 + 3 * leg_idx, 3);
            }
            feedback[leg_idx] = foot_jacobian[leg_idx].transpose() * feedback[leg_idx];
            grf[leg_idx] = -foot_jacobian[leg_idx].transpose() * grf[leg_idx];
        }
        // clang-format off
        jointForce << Eigen::VectorXd::Zero(6), 
						grf[0] + feedback[0] + feedforward[0], 
						grf[1] + feedback[1] + feedforward[1],
						grf[2] + feedback[2] + feedforward[2],
						grf[3] + feedback[3] + feedforward[3];
        // clang-format on
        laikago->setGeneralizedForce(jointForce);

        server.integrateWorldThreadSafe();
    }
    server.killServer();
}