#include "convex_mpc/utils.hpp"

namespace ConvexMPC {
Eigen::Vector3d quaternion_to_euler(const Eigen::Quaterniond& q) {
    Eigen::Vector3d euler = q.toRotationMatrix().eulerAngles(2, 1, 0);
    return euler;
}

Eigen::Quaterniond euler_to_quaternion(const Eigen::Ref<const Eigen::Vector3d>& v) {
    Eigen::Quaterniond q;
    q = Eigen::AngleAxisd(v.z(), Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(v.y(), Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(v.x(), Eigen::Vector3d::UnitX());

    return q;
}
Eigen::Matrix3d euler_to_matrix(const Eigen::Ref<const Eigen::Vector3d>& v) {
    Eigen::Quaterniond q;
    q = Eigen::AngleAxisd(v.z(), Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(v.y(), Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(v.x(), Eigen::Vector3d::UnitX());

    return q.toRotationMatrix();
}
Eigen::Matrix3d ConvexMPC::vector_to_skew(const Eigen::Ref<const Eigen::Vector3d>& v) {
    Eigen::Matrix3d skew;
    // clang-format off
    skew <<  0,   -v.z(),  v.y(),
            v.z(),   0,   -v.x(),
           -v.y(),  v.x(),   0;
    // clang-format on
    return skew;
}

}  // namespace ConvexMPC
