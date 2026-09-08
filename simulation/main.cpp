#include <mujoco/mujoco.h>

#include <Eigen/Dense>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>

#include "convex_mpc/params.hpp"
#include "convex_mpc/robot_controller.hpp"
#include "convex_mpc/robot_model.hpp"
#include "convex_mpc/robot_state.hpp"
#include "convex_mpc/utils.hpp"

#ifdef MPC_USE_VIEWER
#include <GLFW/glfw3.h>
#endif

#include <unistd.h>

using namespace ConvexMPC;

namespace {

// Scene XML lookup: prefer the copy next to the executable (works when the
// build tree is mounted at a different path, e.g. inside the devcontainer),
// fall back to the source-tree path baked in at compile time.
std::string default_scene_path() {
    char exe[4096];
    const ssize_t n = readlink("/proc/self/exe", exe, sizeof(exe) - 1);
    if (n > 0) {
        exe[n] = '\0';
        std::string dir(exe);
        dir = dir.substr(0, dir.find_last_of('/'));
        const std::string local = dir + "/rsc/anymal_c/scene.xml";
        if (access(local.c_str(), R_OK) == 0) return local;
    }
    return ANYMAL_SCENE_XML;
}

constexpr double kControlDt = 0.001;  // simulation / leg-control timestep [s]
constexpr double kMpcDt = 0.02;       // MPC re-plan & discretization step [s] (40 Hz)
constexpr size_t kMpcHorizon = 16;    // 32 * 25 ms = 0.8 s = two trot gait cycles
constexpr double kStanceDuration = 0.2;
constexpr double kSwingDuration = 0.2;

const char* kFootSites[LEG_NUM] = {"LF_FOOT", "RF_FOOT", "LH_FOOT", "RH_FOOT"};

// Rigid-body inertia of the whole robot about its CoM at the current pose,
// expressed in the world frame (== body frame when computed at the nominal,
// unrotated standing pose).
Eigen::Matrix3d compute_composite_inertia(const mjModel* m, const mjData* d, int base_body_id) {
    const Eigen::Vector3d com(d->subtree_com + 3 * base_body_id);
    Eigen::Matrix3d inertia = Eigen::Matrix3d::Zero();
    for (int body = base_body_id; body < m->nbody; body++) {
        const double mass = m->body_mass[body];
        if (mass <= 0) continue;
        const Eigen::Map<const Eigen::Matrix<double, 3, 3, Eigen::RowMajor>> R(d->ximat + 9 * body);
        const Eigen::Vector3d principal_inertia(m->body_inertia + 3 * body);
        const Eigen::Vector3d r = Eigen::Vector3d(d->xipos + 3 * body) - com;
        inertia += R * principal_inertia.asDiagonal() * R.transpose() +
                   mass * (r.dot(r) * Eigen::Matrix3d::Identity() - r * r.transpose());
    }
    return inertia;
}

constexpr double kSettleDuration = 0.7;      // joint-PD standing before the gait starts [s]
constexpr double kLinearCmdSlewRate = 0.6;   // [m/s^2]
constexpr double kAngularCmdSlewRate = 1.5;  // [rad/s^2]

struct SimContext {
    mjModel* m = nullptr;
    mjData* d = nullptr;
    int base_body_id = -1;
    std::array<int, LEG_NUM> foot_site_id{};

    RobotState robot_state;
    RobotController* controller = nullptr;
    Eigen::Vector3d cmd_vel_target = Eigen::Vector3d::Zero();
    Eigen::Vector3d cmd_vel = Eigen::Vector3d::Zero();  // slew-rate-limited command actually applied
    Eigen::VectorXd settle_qpos;                        // joint configuration held during settling

    std::array<Eigen::Vector3d, LEG_NUM> grf{};
    long step = 0;
    long settle_steps = 0;
    int mpc_every = 1;
    double yaw_continuous = 0;
    bool yaw_initialized = false;
};

// Ramp the applied command toward the target with a slew-rate limit, so velocity
// commands come in gradually instead of as a step.
void update_command(SimContext& ctx) {
    for (int axis = 0; axis < 3; axis++) {
        const double rate = (axis == 2 ? kAngularCmdSlewRate : kLinearCmdSlewRate) * kControlDt;
        const double error = ctx.cmd_vel_target[axis] - ctx.cmd_vel[axis];
        ctx.cmd_vel[axis] += std::clamp(error, -rate, rate);
    }
}

// Runs one control + physics step (leg control at kControlDt, MPC at kMpcDt).
void control_and_step(SimContext& ctx) {
    mjModel* m = ctx.m;
    mjData* d = ctx.d;

    // --- state extraction -------------------------------------------------
    const Eigen::Vector3d position(d->qpos);
    const Eigen::Vector4d quat(d->qpos + 3);  // MuJoCo quaternion order: (w, x, y, z)
    const Eigen::Matrix3d R = Eigen::Quaterniond(quat[0], quat[1], quat[2], quat[3]).toRotationMatrix();

    Eigen::Vector3d euler = utils::quaternion_to_euler(quat);
    if (!ctx.yaw_initialized) {
        ctx.yaw_continuous = euler[2];
        ctx.yaw_initialized = true;
    }
    ctx.yaw_continuous = utils::unwrap_angle(ctx.yaw_continuous, euler[2]);
    euler[2] = ctx.yaw_continuous;

    const Eigen::Vector3d linear_velocity(d->qvel);         // world frame
    const Eigen::Vector3d angular_velocity_b(d->qvel + 3);  // MuJoCo free joint: body frame
    const Eigen::Vector3d angular_velocity = R * angular_velocity_b;

    ctx.robot_state.updateState(euler, position, angular_velocity, linear_velocity);

    std::array<Eigen::Vector3d, LEG_NUM> foot_positions_abs, foot_velocity_w;
    std::array<Eigen::Matrix3d, LEG_NUM> foot_jacobian_w;  // world-frame, leg-joint columns
    for (int leg = 0; leg < LEG_NUM; leg++) {
        const int site = ctx.foot_site_id[leg];
        foot_positions_abs[leg] = Eigen::Vector3d(d->site_xpos + 3 * site) - position;

        Eigen::Matrix<mjtNum, 3, Eigen::Dynamic, Eigen::RowMajor> jac(3, m->nv);
        mj_jacSite(m, d, jac.data(), nullptr, site);
        foot_jacobian_w[leg] = jac.block<3, 3>(0, 6 + 3 * leg);

        const Eigen::Vector3d qd_leg(d->qvel + 6 + 3 * leg);
        foot_velocity_w[leg] = foot_jacobian_w[leg] * qd_leg;  // joint contribution, world frame
    }
    ctx.robot_state.updateFootPosition(foot_positions_abs);
    ctx.robot_state.updateFootVelocity(foot_velocity_w);

    // --- settling phase: hold the standing pose with joint PD until contacts and
    // --- state have settled, then hand over to the MPC cleanly.
    if (ctx.step < ctx.settle_steps) {
        for (int j = 0; j < 12; j++) {
            const double q_err = ctx.settle_qpos[7 + j] - d->qpos[7 + j];
            d->ctrl[j] = 100.0 * q_err - 5.0 * d->qvel[6 + j] + d->qfrc_bias[6 + j];
        }
        mj_step(m, d);
        ctx.step++;
        return;
    }

    // --- control ----------------------------------------------------------
    update_command(ctx);
    if (ctx.step % ctx.mpc_every == 0) {
        ctx.grf = ctx.controller->computeGRF(ctx.robot_state, ctx.cmd_vel);
#ifdef MPC_DEBUG_LOG
        if (d->time < 2.0) {
            double contact_fz = 0;  // actual total vertical contact force from the simulator
            for (int c = 0; c < d->ncon; c++) {
                mjtNum wrench[6];
                mj_contactForce(m, d, c, wrench);
                mjtNum normal_world[3] = {d->contact[c].frame[0], d->contact[c].frame[1], d->contact[c].frame[2]};
                contact_fz += wrench[0] * normal_world[2];
            }
            std::printf(
                "dbg t=%.3f contacts=%d%d%d%d rpy=[%5.2f %5.2f %5.2f] fz=[%6.1f %6.1f %6.1f %6.1f] "
                "footz=[%5.3f %5.3f %5.3f %5.3f] real_fz=%6.1f\n",
                d->time,
                ctx.robot_state.contact_state(0),
                ctx.robot_state.contact_state(1),
                ctx.robot_state.contact_state(2),
                ctx.robot_state.contact_state(3),
                euler[0],
                euler[1],
                euler[2],
                ctx.grf[0][2],
                ctx.grf[1][2],
                ctx.grf[2][2],
                ctx.grf[3][2],
                foot_positions_abs[0][2] + position[2],
                foot_positions_abs[1][2] + position[2],
                foot_positions_abs[2][2] + position[2],
                foot_positions_abs[3][2] + position[2],
                contact_fz);
        }
#endif
    }
    const std::array<Eigen::Vector3d, LEG_NUM> swing_force_w = ctx.controller->computeSwingForce(ctx.robot_state, ctx.cmd_vel);

    for (int leg = 0; leg < LEG_NUM; leg++) {
        // Stance: tau = -J^T f  (f = ground reaction force on the robot, world frame).
        // Swing: world-frame PD force at the foot (Di Carlo et al., eq. 1).
        Eigen::Vector3d tau = ctx.robot_state.contact_state(leg)
                                  ? Eigen::Vector3d(-foot_jacobian_w[leg].transpose() * ctx.grf[leg])
                                  : Eigen::Vector3d(foot_jacobian_w[leg].transpose() * swing_force_w[leg]);
        // Gravity/Coriolis compensation of the leg itself (eq. 2). ANYmal's legs are a
        // large mass fraction, so stance legs need this too: it makes the realized
        // contact force match the commanded one, which the single-rigid-body model
        // (total mass) assumes.
        tau += Eigen::Vector3d(d->qfrc_bias + 6 + 3 * leg);
        for (int j = 0; j < 3; j++) d->ctrl[3 * leg + j] = tau[j];
    }

    mj_step(m, d);
    ctx.step++;
}

void print_status(const SimContext& ctx) {
    const Eigen::Vector3d euler = ctx.robot_state.euler_angle();
    const Eigen::Vector3d p = ctx.robot_state.position();
    const Eigen::Vector3d v = ctx.robot_state.linear_velocity();
    double fz_total = 0;
    for (int leg = 0; leg < LEG_NUM; leg++) fz_total += ctx.grf[leg][2];
    std::printf("t=%6.2fs  pos=[%6.3f %6.3f %6.3f]  rpy=[%6.3f %6.3f %6.3f]  vel=[%5.2f %5.2f %5.2f]  sum_fz=%6.1f\n",
                ctx.d->time,
                p[0],
                p[1],
                p[2],
                euler[0],
                euler[1],
                euler[2],
                v[0],
                v[1],
                v[2],
                fz_total);
}

}  // namespace

int main(int argc, char** argv) {
    std::string xml_path = default_scene_path();
    bool headless = false;
    long max_steps = -1;  // headless default set below
    Eigen::Vector3d cmd_vel = Eigen::Vector3d::Zero();

    bool stand_only = false;
    for (int i = 1; i < argc; i++) {
        if (!std::strcmp(argv[i], "--headless"))
            headless = true;
        else if (!std::strcmp(argv[i], "--stand"))
            stand_only = true;
        else if (!std::strcmp(argv[i], "--steps") && i + 1 < argc)
            max_steps = std::atol(argv[++i]);
        else if (!std::strcmp(argv[i], "--vx") && i + 1 < argc)
            cmd_vel[0] = std::atof(argv[++i]);
        else if (!std::strcmp(argv[i], "--vy") && i + 1 < argc)
            cmd_vel[1] = std::atof(argv[++i]);
        else if (!std::strcmp(argv[i], "--wz") && i + 1 < argc)
            cmd_vel[2] = std::atof(argv[++i]);
        else if (!std::strcmp(argv[i], "--xml") && i + 1 < argc)
            xml_path = argv[++i];
        else {
            std::printf("usage: %s [--headless] [--steps N] [--vx V] [--vy V] [--wz W] [--xml PATH]\n", argv[0]);
            return 1;
        }
    }
#ifndef MPC_USE_VIEWER
    headless = true;
#endif
    if (max_steps < 0) max_steps = headless ? long(10.0 / kControlDt) : -1;

    char error[1000] = "";
    mjModel* m = mj_loadXML(xml_path.c_str(), nullptr, error, sizeof(error));
    if (!m) {
        std::printf("failed to load %s: %s\n", xml_path.c_str(), error);
        return 1;
    }
    m->opt.timestep = kControlDt;
    mjData* d = mj_makeData(m);
    mj_resetDataKeyframe(m, d, mj_name2id(m, mjOBJ_KEY, "standing"));
    mj_forward(m, d);

    SimContext ctx;
    ctx.m = m;
    ctx.d = d;
    ctx.base_body_id = mj_name2id(m, mjOBJ_BODY, "base");
    for (int leg = 0; leg < LEG_NUM; leg++) {
        ctx.foot_site_id[leg] = mj_name2id(m, mjOBJ_SITE, kFootSites[leg]);
        if (ctx.foot_site_id[leg] < 0) {
            std::printf("missing site %s in model\n", kFootSites[leg]);
            return 1;
        }
    }
    ctx.cmd_vel_target = cmd_vel;
    ctx.mpc_every = int(kMpcDt / kControlDt + 0.5);
    ctx.settle_steps = long(kSettleDuration / kControlDt);
    ctx.settle_qpos = Eigen::Map<Eigen::VectorXd>(d->qpos, m->nq);

    // Single-rigid-body parameters extracted from the simulation model at the nominal pose.
    const double mass = m->body_subtreemass[ctx.base_body_id];
    const Eigen::Matrix3d inertia = compute_composite_inertia(m, d, ctx.base_body_id);
    std::printf("mpc_locomotion built %s %s\n", __DATE__, __TIME__);
    std::printf("robot mass: %.3f kg\ncomposite inertia [kg m^2]:\n", mass);
    std::printf("  [%7.4f %7.4f %7.4f; %7.4f %7.4f %7.4f; %7.4f %7.4f %7.4f]\n",
                inertia(0, 0),
                inertia(0, 1),
                inertia(0, 2),
                inertia(1, 0),
                inertia(1, 1),
                inertia(1, 2),
                inertia(2, 0),
                inertia(2, 1),
                inertia(2, 2));

    RobotModel robot_model(inertia, mass, params::gravity, params::mu, kMpcDt, params::f_min, 700);

    // Nominal state (defines target height and nominal footholds for swing legs).
    const Eigen::Vector3d init_position(d->qpos);
    const Eigen::Vector4d init_quat(d->qpos + 3);
    Eigen::Vector3d nominal_position = Eigen::Vector3d(d->qpos);
    nominal_position[2] = 0.5;

    std::array<Eigen::Vector3d, LEG_NUM> init_foot_positions;
    for (int leg = 0; leg < LEG_NUM; leg++)
        init_foot_positions[leg] = Eigen::Vector3d(d->site_xpos + 3 * ctx.foot_site_id[leg]) - init_position;
    RobotState nominal_state(utils::quaternion_to_euler(init_quat),
                             nominal_position,
                             Eigen::Vector3d::Zero(),
                             Eigen::Vector3d::Zero(),
                             init_foot_positions);

    Eigen::VectorXd q_weights(MPC_STATE_DIM), r_weights(MPC_INPUT_DIM);
    q_weights << 50, 150, 30, 25, 25, 100, 1, 3, 2, 30, 30, 10, 0;
    r_weights << 3e-4, 3e-4, 1e-4, 3e-4, 3e-4, 1e-4, 3e-4, 3e-4, 1e-4, 3e-4, 3e-4, 1e-4;

    // --stand keeps all four feet in permanent stance (useful to debug force control).
    const double stance_duration = stand_only ? 1e9 : kStanceDuration;
    RobotController controller(
        robot_model, nominal_state, q_weights, r_weights, 500, 40, kMpcHorizon, kControlDt, stance_duration, kSwingDuration);
    ctx.controller = &controller;

    ctx.robot_state = nominal_state;
    if (stand_only)
        ctx.robot_state.updateContactState({true, true, true, true});
    else
        ctx.robot_state.updateContactState({true, false, false, true});  // trot: LF+RH stance first

#ifdef MPC_USE_VIEWER
    if (!headless) {
        if (!glfwInit()) {
            std::printf("glfw init failed, falling back to headless mode\n");
            headless = true;
        } else {
            GLFWwindow* window = glfwCreateWindow(1440, 900, "Convex MPC - ANYmal C", nullptr, nullptr);
            if (!window) {
                std::printf("glfw window creation failed, falling back to headless mode\n");
                glfwTerminate();
                headless = true;
            } else {
                glfwMakeContextCurrent(window);
                glfwSwapInterval(1);

                mjvCamera cam;
                mjvOption opt;
                mjvScene scn;
                mjrContext con;
                mjv_defaultCamera(&cam);
                mjv_defaultOption(&opt);
                mjv_defaultScene(&scn);
                mjr_defaultContext(&con);
                mjv_makeScene(m, &scn, 2000);
                mjr_makeContext(m, &con, mjFONTSCALE_150);
                cam.type = mjCAMERA_TRACKING;
                cam.trackbodyid = ctx.base_body_id;
                cam.distance = 3.0;
                cam.elevation = -20;

                long log_every = long(1.0 / kControlDt);
                double wall_last = glfwGetTime(), sim_last = d->time;
                while (!glfwWindowShouldClose(window) && (max_steps < 0 || ctx.step < max_steps)) {
                    const mjtNum frame_start = d->time;
                    while (d->time - frame_start < 1.0 / 60.0) {
                        control_and_step(ctx);
                        if (ctx.step % log_every == 0) {
                            print_status(ctx);
                            const double wall_now = glfwGetTime();
                            std::printf("  realtime factor: %.2fx\n", (d->time - sim_last) / (wall_now - wall_last));
                            wall_last = wall_now;
                            sim_last = d->time;
                        }
                    }

                    mjrRect viewport = {0, 0, 0, 0};
                    glfwGetFramebufferSize(window, &viewport.width, &viewport.height);
                    mjv_updateScene(m, d, &opt, nullptr, &cam, mjCAT_ALL, &scn);

                    // Ghost: single-rigid-body pose predicted by the MPC at the end of the horizon.
                    if (scn.ngeom < scn.maxgeom) {
                        const Eigen::Vector<double, MPC_STATE_DIM> pred = controller.mpc_result();
                        const Eigen::Matrix3d R_pred = utils::euler_to_matrix(pred.segment<3>(0));
                        mjtNum size[3] = {0.33, 0.15, 0.09};
                        mjtNum pos[3] = {pred[3], pred[4], pred[5]};
                        mjtNum mat[9];
                        for (int r = 0; r < 3; r++)
                            for (int c = 0; c < 3; c++) mat[3 * r + c] = R_pred(r, c);
                        float rgba[4] = {0.3f, 0.7f, 1.0f, 0.35f};
                        mjv_initGeom(scn.geoms + scn.ngeom, mjGEOM_BOX, size, pos, mat, rgba);
                        scn.ngeom++;
                    }

                    mjr_render(viewport, &scn, &con);
                    glfwSwapBuffers(window);
                    glfwPollEvents();
                }

                mjr_freeContext(&con);
                mjv_freeScene(&scn);
                glfwDestroyWindow(window);
                glfwTerminate();
            }
        }
    }
#endif

    int exit_code = 0;
    if (headless) {
        std::printf("running headless for %ld steps (%.1f s)\n", max_steps, max_steps * kControlDt);
        const long log_every = long(1.0 / kControlDt);
        while (ctx.step < max_steps) {
            control_and_step(ctx);
            if (ctx.step % log_every == 0) print_status(ctx);
        }
        const Eigen::Vector3d euler = ctx.robot_state.euler_angle();
        const double height = ctx.robot_state.position()[2];
        const bool stable = height > 0.3 && height < 0.8 && std::abs(euler[0]) < 0.5 && std::abs(euler[1]) < 0.5;
        std::printf("%s (height %.3f m, roll %.3f, pitch %.3f)\n", stable ? "STABLE" : "FALLEN", height, euler[0], euler[1]);
        exit_code = stable ? 0 : 1;
    }

    mj_deleteData(d);
    mj_deleteModel(m);
    return exit_code;
}
