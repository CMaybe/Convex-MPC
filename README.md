# Dynamic Locomotion in the MIT Cheetah 3 Through Convex Model-Predictive Control

## Overview

This project implements dynamic locomotion for the MIT Cheetah 3 robot using Convex Model-Predictive Control (MPC). The work is inspired by the paper **"Dynamic Locomotion in the MIT Cheetah 3 Through Convex Model-Predictive Control"**. The goal is to enable the Cheetah 3 robot to perform stable and dynamic movements using MPC in a simulation environment.

This project uses **Raisim** for robot simulation and **qpOASES** for solving the optimization problem within MPC.

### Citation

```
@INPROCEEDINGS{8593885,
  author={Bledt, Gerardo and Powell, Matthew J. and Katz, Benjamin and Di Carlo, Jared and Wensing, Patrick M. and Kim, Sangbae},
  booktitle={2018 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)}, 
  title={MIT Cheetah 3: Design and Control of a Robust, Dynamic Quadruped Robot}, 
  year={2018},
  volume={},
  number={},
  pages={2245-2252},
  keywords={Legged locomotion;Actuators;Torque;Force;Knee;Robot sensing systems},
  doi={10.1109/IROS.2018.8593885}}
```
## Objective

The objective of this project is to replicate and implement the results and methodologies proposed in the paper, focusing on:

- Model-predictive control (MPC) for dynamic locomotion
- Simulation of the robot’s movement in various environments
- Optimization of control policies using convex optimization
- Implementation of robust locomotion strategies that allow the robot to adapt to dynamic terrain