#include "raisim/RaisimServer.hpp"
#include "raisim/World.hpp"

int main() {
    raisim::World world;
    world.setTimeStep(0.001);

    /// create objects
    auto ground = world.addGround();

    raisim::ArticulatedSystem* laikago = world.addArticulatedSystem("./rsc/laikago/laikago.urdf");

    Eigen::VectorXd jointNominalConfig(19), jointVelocityTarget(18);
    Eigen::VectorXd jointState(18), jointForce(18), jointPgain(18), jointDgain(18);
    jointPgain.setZero();
    jointDgain.setZero();
    jointVelocityTarget.setZero();
    jointNominalConfig << 0, 0, 0.48, 1, 0.0, 0.0, 0.0, 0.0, 0.5, -1, 0, 0.5, -1, 0.00, 0.5, -1, 0, 0.5, -0.7;

    jointPgain.tail(12).setConstant(200.0);
    jointDgain.tail(12).setConstant(10.0);

    laikago->setGeneralizedCoordinate({0, 0, 0.48, 1, 0.0, 0.0, 0.0, 0.0, 0.5, -1, 0, 0.5, -1, 0.00, 0.5, -1, 0, 0.5, -0.7});
    laikago->setGeneralizedForce(Eigen::VectorXd::Zero(laikago->getDOF()));
    laikago->setControlMode(raisim::ControlMode::PD_PLUS_FEEDFORWARD_TORQUE);
    laikago->setPdGains(jointPgain, jointDgain);
    laikago->setPdTarget(jointNominalConfig, jointVelocityTarget);
    laikago->setName("laikago");

    /// launch raisim server
    raisim::RaisimServer server(&world);
    server.launchServer();

    for (int i = 0; i < 10000000; i++) {
        RS_TIMED_LOOP(int(world.getTimeStep() * 1e6))
        server.integrateWorldThreadSafe();
    }

    server.killServer();
}