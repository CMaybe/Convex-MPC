#include "convex_mpc/utils.hpp"

namespace ConvexMPC {
Eigen::Vector3d quaternion_to_euler(const Eigen::Vector4d& q) {
    Eigen::Vector3d eulerAngles;  // [roll, pitch, yaw]

    double norm = std::sqrt(q(0) * q(0) + q(1) * q(1) + q(2) * q(2) + q(3) * q(3));
    double qw = q(0) / norm;
    double qx = q(1) / norm;
    double qy = q(2) / norm;
    double qz = q(3) / norm;

    double sinr_cosp = 2 * (qw * qx + qy * qz);
    double cosr_cosp = 1 - 2 * (qx * qx + qy * qy);
    eulerAngles[0] = std::atan2(sinr_cosp, cosr_cosp);

    double sinp = 2 * (qw * qy - qz * qx);
    if (std::abs(sinp) >= 1) {
        eulerAngles[1] = std::copysign(M_PI / 2, sinp);
    } else {
        eulerAngles[1] = std::asin(sinp);
    }

    double siny_cosp = 2 * (qw * qz + qx * qy);
    double cosy_cosp = 1 - 2 * (qy * qy + qz * qz);
    eulerAngles[2] = std::atan2(siny_cosp, cosy_cosp);

    return eulerAngles;
}

Eigen::Matrix3d ConvexMPC::skew(const Eigen::Vector3d& vec) {
    Eigen::Matrix3d rst;
    rst.setZero();
    rst << 0, -vec(2), vec(1), vec(2), 0, -vec(0), -vec(1), vec(0), 0;
    return rst;
}

}  // namespace ConvexMPC
