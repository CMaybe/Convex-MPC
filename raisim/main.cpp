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

    /// create objects
    auto ground = world.addGround();

    raisim::ArticulatedSystem* laikago = world.addArticulatedSystem("./rsc/laikago/laikago.urdf");
    std::string leg_list[4] = {"FR_foot_fixed", "FL_foot_fixed", "RR_foot_fixed", "RL_foot_fixed"};

    RobotModel robot_model(laikago->getInertia()[0].e(),
                           laikago->getMass()[0],
                           params::gravity,
                           params::mu,
                           params::dt,
                           0,  // params::f_min,
                           params::f_max);
    Eigen::Vector3d cmd_vel;

    std::atomic<bool> control_execute{};
    control_execute.store(true, std::memory_order_release);

    Eigen::VectorXd jointNominalConfig(laikago->getGeneralizedCoordinateDim()), jointVelocityTarget(laikago->getDOF());
    Eigen::VectorXd current_jointVelocity(laikago->getDOF()), current_jointConfig(laikago->getGeneralizedCoordinateDim());
    Eigen::VectorXd jointPgain(laikago->getDOF()), jointDgain(laikago->getDOF()), jointForce(laikago->getDOF());
    std::array<Eigen::Vector3d, LEG_NUM> foot_position;
    std::array<Eigen::Vector3d, LEG_NUM> grf;

    jointPgain.setZero();
    jointDgain.setZero();
    jointVelocityTarget.setZero();
    jointNominalConfig << 0, 0, 0.48, 1, 0.0, 0.0, 0.0, 0.0, 0.5, -1, 0, 0.5, -1, 0.00, 0.5, -1, 0, 0.5, -0.7;

    jointPgain.setConstant(0.0);
    jointDgain.setConstant(0.0);

    laikago->setControlMode(raisim::ControlMode::PD_PLUS_FEEDFORWARD_TORQUE);
    laikago->setPdTarget(jointNominalConfig, jointVelocityTarget);
    laikago->setName("laikago");
    jointVelocityTarget.setZero();

    laikago->setGeneralizedCoordinate(jointNominalConfig);
    laikago->setPdGains(jointPgain, jointDgain);
    laikago->setName("laikago");

    /// launch raisim server
    raisim::RaisimServer server(&world);
    server.focusOn(laikago);
    server.launchServer();

    Eigen::Vector3d position = current_jointConfig.block(3, 0, 3, 1);
    Eigen::Vector4d quat = current_jointConfig.block(3, 0, 4, 1);
    Eigen::Vector3d euler = utils::quaternion_to_euler(quat);
    Eigen::Vector3d linear_velocity = current_jointVelocity.block(0, 0, 3, 1);
    Eigen::Vector3d angular_velocity = current_jointVelocity.block(3, 0, 3, 1);
    for (int leg_idx = 0; leg_idx < LEG_NUM; leg_idx++) {
        raisim::Vec<3> foot_pos;
        raisim::Vec<3> body_pos;
        laikago->getBasePosition(body_pos);
        laikago->getFramePosition(leg_list[leg_idx], foot_pos);
        foot_position[leg_idx] = foot_pos.e() - body_pos.e();
    }

    RobotState robot_state(euler, position, angular_velocity, linear_velocity, foot_position);
    RobotController robot_controller(robot_model, robot_state);

    /// mpc

    RS_TIMED_LOOP(int(world.getTimeStep() * 50 * 1e6))
    for (int i = 0; i < 10000000; i++) {
        current_jointConfig = laikago->getGeneralizedCoordinate().e();
        current_jointVelocity = laikago->getGeneralizedVelocity().e();
        Eigen::Vector3d position = current_jointConfig.block(3, 0, 3, 1);
        Eigen::Vector4d quat = current_jointConfig.block(3, 0, 4, 1);
        Eigen::Vector3d euler = utils::quaternion_to_euler(quat);
        Eigen::Matrix3d R_T = utils::euler_to_matrix(euler).transpose();
        Eigen::Vector3d linear_velocity = current_jointVelocity.block(0, 0, 3, 1);
        Eigen::Vector3d angular_velocity = current_jointVelocity.block(3, 0, 3, 1);
        robot_state.updateState(euler, position, angular_velocity, linear_velocity);

        std::array<Eigen::Matrix3d, LEG_NUM> foot_jacobian;
        cmd_vel = Eigen::Vector3d(0, 0, 0);

        for (int leg_idx = 0; leg_idx < LEG_NUM; leg_idx++) {
            raisim::Vec<3> foot_pos;
            raisim::Vec<3> body_pos;
            laikago->getBasePosition(body_pos);
            laikago->getFramePosition(leg_list[leg_idx], foot_pos);
            foot_position[leg_idx] = foot_pos.e() - body_pos.e();
        }
        robot_state.updateFootPosition(foot_position);
        grf = robot_controller.computeGRF(robot_state, cmd_vel);
        for (int leg_idx = 0; leg_idx < LEG_NUM; leg_idx++) {
            Eigen::MatrixXd full_jacobian(3, laikago->getDOF());
            laikago->getDenseFrameJacobian(leg_list[leg_idx], full_jacobian);
            foot_jacobian[leg_idx] = full_jacobian.block(0, 6 + 3 * leg_idx, 3, 3);
            grf[leg_idx] = foot_jacobian[leg_idx].transpose() * grf[leg_idx];
        }
        std::cout << std::endl;

        jointForce << Eigen::VectorXd::Zero(6), grf[0], grf[1], grf[2], grf[3];
        std::cout << jointForce.transpose() << std::endl;
        laikago->setGeneralizedForce(jointForce);

        RS_TIMED_LOOP(int(world.getTimeStep() * 1e6))
        server.integrateWorldThreadSafe();
    }
    server.killServer();
}