#ifndef ROBOT_PARAM_HPP
#define ROBOT_PARAM_HPP
namespace ConvexMPC {
// mpc
#define MPC_STATE_DIM 13
#define MPC_INPUT_DIM 12
#define MPC_CONSTRAINT_DIM 20
#define MPC_HORIZON 10

#define LEG_NUM 4
#define LEG_DOF 3

class params {
public:
    static constexpr double f_min = 0;
    static constexpr double f_max = 200;
    static constexpr double mu = 0.6;
    static constexpr double mass = 43;
    static constexpr double dt = 0.01;
    static constexpr double gravity = -9.81;
    static constexpr double kp = 150;
    static constexpr double kd = 10;
};

}  // namespace ConvexMPC
#endif  // ROBOT_PARAM_HPP