#include <Eigen/Geometry>
#include <Eigen/Core>
#include <math.h>

Eigen::Quaterniond euler2Quaternion(double roll, double pitch, double yaw) {
    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitZ());
    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitX());

    Eigen::Quaterniond q = rollAngle * yawAngle * pitchAngle;
    //roll = roll * 0.5;
    //pitch = pitch * 0.5;
    //yaw = yaw * 0.5;
    //double cosPhi = cos(roll);
    //double sinPhi = sin(roll);
    //double cosTheta = cos(pitch);
    //double sinTheta = sin(pitch);
    //double cosPsi = cos(yaw);
    //double sinPsi = sin(yaw);
    //Eigen::Quaterniond q;
    //q << sinPhi * cosTheta * cosPsi - cosPhi * sinTheta * sinPsi, cosPhi * sinTheta * cosPsi + sinPhi * cosTheta * sinPsi, cosPhi * cosTheta * sinPsi - sinPhi * sinTheta * cosPsi, cosPhi * cosTheta * cosPsi + sinPhi * sinTheta * sinPsi; 
    return q;
}

Eigen::Quaterniond QuatMult(Eigen::Quaterniond q1, Eigen::Quaterniond q2) {
    Eigen::Quaterniond q;
    q.w() = q1.w() * q2.w() - q1.x() * q2.x() - q1.y() * q2.y() - q1.z() * q2.z(); 
    q.x() = q1.x() * q2.w() + q1.w() * q2.x() + q1.y() * q2.z() - q1.z() * q2.y(); 
    q.y() = q1.y() * q2.w() + q1.w() * q2.y() + q1.z() * q2.x() - q1.x() * q2.z(); 
    q.z() = q1.z() * q2.w() + q1.w() * q2.z() + q1.x() * q2.y() - q1.y() * q2.x();
    return q;
}

Eigen::Quaterniond RotVector2Quat(Eigen::Vector3d rotVec) {
    Eigen::Quaterniond q;
    double vecLength;
    vecLength = rotVec.squaredNorm();
    if (vecLength < 1e-6) {
        q.w() = 1;
        q.x() = 0;
        q.y() = 0;
        q.z() = 0;
    } else {
        q.w() = cos(0.5 * vecLength);
        q.x() = rotVec[0] / vecLength * sin(0.5 * vecLength);
        q.y() = rotVec[1] / vecLength * sin(0.5 * vecLength);
        q.z() = rotVec[2] / vecLength * sin(0.5 * vecLength);
    }
}
