#ifndef UTILS_HPP
#define UTILS_HPP

#include <Eigen/Dense>

namespace ConvexMPC {

inline Eigen::Vector3d quaternion_to_euler(const Eigen::Quaterniond& q) {
    return q.toRotationMatrix().eulerAngles(2, 1, 0);
}

inline Eigen::Quaterniond euler_to_quaternion(const Eigen::Ref<const Eigen::Vector3d>& v) {
    return Eigen::AngleAxisd(v.z(), Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(v.y(), Eigen::Vector3d::UnitY()) *
           Eigen::AngleAxisd(v.x(), Eigen::Vector3d::UnitX());
}

inline Eigen::Matrix3d euler_to_matrix(const Eigen::Ref<const Eigen::Vector3d>& v) {
    return euler_to_quaternion(v).toRotationMatrix();
}

inline Eigen::Matrix3d vector_to_skew(const Eigen::Ref<const Eigen::Vector3d>& v) {
    Eigen::Matrix3d skew;
    skew << 0, -v.z(), v.y(), v.z(), 0, -v.x(), -v.y(), v.x(), 0;
    return skew;
}

}  // namespace ConvexMPC

#endif