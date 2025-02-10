#ifndef UTILS_HPP
#define UTILS_HPP

#include <Eigen/Dense>

namespace ConvexMPC {

namespace utils {
inline Eigen::Vector3d quaternion_to_euler(const Eigen::Quaterniond& q) { return q.toRotationMatrix().eulerAngles(2, 1, 0); }
inline Eigen::Vector3d quaternion_to_euler(const Eigen::Ref<const Eigen::Vector4d>& q) {
    Eigen::Vector3d eulerAngles;  // [roll, pitch, yaw]

    // Normalize the quaternion to ensure valid rotation
    double norm = std::sqrt(q(0) * q(0) + q(1) * q(1) + q(2) * q(2) + q(3) * q(3));
    double qw = q(0) / norm;
    double qx = q(1) / norm;
    double qy = q(2) / norm;
    double qz = q(3) / norm;

    // Roll (X-axis rotation)
    double sinr_cosp = 2 * (qw * qx + qy * qz);
    double cosr_cosp = 1 - 2 * (qx * qx + qy * qy);
    eulerAngles[0] = std::atan2(sinr_cosp, cosr_cosp);

    // Pitch (Y-axis rotation)
    double sinp = 2 * (qw * qy - qz * qx);
    if (std::abs(sinp) >= 1) {
        // Use 90 degrees if out of range (for numerical stability)
        eulerAngles[1] = std::copysign(M_PI / 2, sinp);
    } else {
        eulerAngles[1] = std::asin(sinp);
    }

    // Yaw (Z-axis rotation)
    double siny_cosp = 2 * (qw * qz + qx * qy);
    double cosy_cosp = 1 - 2 * (qy * qy + qz * qz);
    eulerAngles[2] = std::atan2(siny_cosp, cosy_cosp);

    return eulerAngles;  // Returns [roll, pitch, yaw]
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

inline double bezier_curve(const double& s, const std::vector<double>& P) {
    std::vector<double> coefficients{1, 4, 6, 4, 1};
    int order = P.size() - 1;
    double result = 0;
    for (int i = 0; i <= order; i++) {
        result += coefficients[i] * std::pow(s, i) * std::pow(1 - s, order - i) * P[i];
    }
    return result;
}

};  // namespace utils

}  // namespace ConvexMPC

#endif