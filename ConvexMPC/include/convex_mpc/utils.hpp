#ifndef UTILS_HPP
#define UTILS_HPP

#include <Eigen/Dense>

namespace ConvexMPC {
static Eigen::Vector3d quaternion_to_euler(const Eigen::Quaterniond& q);
static Eigen::Quaterniond euler_to_quaternion(const Eigen::Ref<const Eigen::Vector3d>& v);
static Eigen::Matrix3d euler_to_matrix(const Eigen::Ref<const Eigen::Vector3d>& v);
static Eigen::Matrix3d vector_to_skew(const Eigen::Ref<const Eigen::Vector3d>& v);
}  // namespace ConvexMPC

#endif