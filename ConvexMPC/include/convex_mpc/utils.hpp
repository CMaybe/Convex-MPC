#ifndef UTILS_HPP
#define UTILS_HPP

#include <Eigen/Dense>

namespace ConvexMPC {
    static Eigen::Vector3d quaternion_to_euler(const Eigen::Vector4d& q);
    static Eigen::Matrix3d skew(const Eigen::Vector3d& vec);
}  // namespace ConvexMPC

#endif