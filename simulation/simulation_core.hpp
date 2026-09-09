#ifndef CONVEX_MPC_SIMULATION_CORE_HPP
#define CONVEX_MPC_SIMULATION_CORE_HPP

#include <array>
#include <memory>
#include <string>
#include <vector>

#include <mujoco/mujoco.h>
#include <Eigen/Dense>

#include "convex_mpc/params.hpp"
#include "convex_mpc/robot_controller.hpp"
#include "convex_mpc/robot_model.hpp"
#include "convex_mpc/robot_state.hpp"

namespace ConvexMPC {

struct SimulationSnapshot {
    double time = 0;
    std::array<double, 3> position{};
    std::array<double, 4> quaternion{};
    std::array<double, 3> euler{};
    std::array<double, LEG_NUM * 3> foot_positions{};
    std::array<double, LEG_NUM * 3> ground_reaction_forces{};
};

struct VisualGeometrySnapshot {
    std::string mesh_name;
    std::array<double, 3> position{};
    std::array<double, 9> rotation{};
    std::array<double, 3> scale{};
};

class SimulationCore {
public:
    explicit SimulationCore(const std::string& xml_path, bool stand_only = false);
    ~SimulationCore();

    SimulationCore(const SimulationCore&) = delete;
    SimulationCore& operator=(const SimulationCore&) = delete;

    void set_velocity_command(const Eigen::Vector3d& command);
    void set_body_height(double height);
    void set_mpc_tuning(double position_scale, double velocity_scale, double force_scale);
    void step(int count = 1);
    SimulationSnapshot snapshot() const;
    std::vector<VisualGeometrySnapshot> visual_geometries() const;

private:
    void control_and_step();
    void update_command();
    Eigen::Matrix3d compute_composite_inertia() const;

    mjModel* model_ = nullptr;
    mjData* data_ = nullptr;
    int base_body_id_ = -1;
    std::array<int, LEG_NUM> foot_site_ids_{};
    std::unique_ptr<RobotController> controller_;
    RobotState robot_state_;
    Eigen::Vector3d command_target_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d command_ = Eigen::Vector3d::Zero();
    Eigen::VectorXd settle_qpos_;
    Eigen::Vector<double, MPC_STATE_DIM> base_q_weights_;
    Eigen::Vector<double, MPC_INPUT_DIM> base_r_weights_;
    Eigen::Vector<double, MPC_STATE_DIM> gait_q_weights_;
    Eigen::Vector<double, MPC_INPUT_DIM> gait_r_weights_;
    std::array<Eigen::Vector3d, LEG_NUM> ground_reaction_forces_{};
    long step_count_ = 0;
    long settle_steps_ = 0;
    int mpc_every_ = 1;
    double max_linear_speed_ = 0.6;
    double max_yaw_rate_ = 0.8;
    double yaw_continuous_ = 0;
    bool yaw_initialized_ = false;
};

}  // namespace ConvexMPC

#endif
