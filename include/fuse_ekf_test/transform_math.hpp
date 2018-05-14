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

Eigen::Matrix<double, 24, 24> calcF24(Eigen::Vector3d del_Ang_Cov, Eigen::Vector3d del_Vel_Cov, Eigen::Vector3d del_Ang_bias, Eigen::Vector3d del_Vel_bias, double dt, Eigen::Quaterniond q) {
    double t2 = del_Ang_bias[0] * 0.5;
    double t3 = del_Ang_bias[2] * 0.5;
    double t4 = del_Ang_bias[1] * 0.5;
    double t8 = del_Ang_Cov[1] * 0.5;
    double t5 = t4 - t8;
    double t6 = q.z() * 0.5;
    double t7 = q.y() * 0.5;
    double t9 = del_Ang_Cov[2] * 0.5;
    double t10 = del_Ang_Cov[0] * 0.5;
    double t11 = - t2 + t10;
    double t12 = q.x() * 0.5;
    double t13 = - t3 + t9;
    double t14 = - t4 + t8;
    double t15 = del_Vel_Cov[0] - del_Vel_bias[0];
    double t16 = del_Vel_Cov[1] - del_Vel_bias[1];
    double t17 = del_Vel_Cov[2] - del_Vel_bias[2];
    double t18 = q.x() * t17 * 2.0;
    double t19 = q.x() * t16 * 2.0;
    double t20 = q.w() * t17 * 2.0;
    double t21 = q.x() * t15 * 2.0;
    double t22 = q.y() * t16 * 2.0;
    double t23 = q.z() * t17 * 2.0;
    double t24 = t21 + t22 + t23;
    double t25 = q.w() * t15 * 2.0;
    double t26 = q.y() * t17 * 2.0;
    double t37 = q.z() * t16 * 2.0;
    double t27 = t25 + t26 - t37;
    double t28 = q.w() * q.z() * 2.0;
    double t29 = q.w() * q.w();
    double t30 = q.x() * q.x();
    double t31 = q.y() * q.y();
    double t32 = q.z() * q.z();
    double t33 = q.y() * t15 * 2.0;
    double t34 = q.z() * t15 * 2.0;
    double t35 = q.w() * t16 * 2.0;
    double t36 = - t18 + t34 + t35;
    double t38 = q.w() * q.x() * 2.0;
    Eigen::Matrix<double, 24, 24> F_;
    F_ << 1.0, t11, t14, t13, t27, t36, t19 + t20 -t33, 0.0,
          0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,            0.0,
          0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,            0.0, //1
          del_Ang_Cov[0] * (-0.5) + t2, 1.0, t3 - t9, t14, t24, - t19 - t20 + t33, t36, 0.0,
          0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
          0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, //2
          t5, t13, 1.0, t2 - t10, t19 + t20 - q.y() * t15 * 2.0, t24, - t25 - t26 + t37, 0.0,
          0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
          0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, //3
          del_Ang_Cov[2] * (-0.5) + t3, t5, t11, 1.0, t18 - q.w() * t16 * 2.0 - q.z() * t15 * 2.0, t27, t24, 0.0,
          0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
          0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, //4
          0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, dt,
          0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
          0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, //5
          0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
          dt, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
          0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, //6
          0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
          0.0, dt, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
          0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, //7
          0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0,
          0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
          0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, //8
          0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
          1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
          0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, //9 
          0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
          0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
          0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, //10
          t12, q.w() * (-0.5), - t6, t7, 0.0, 0.0, 0.0, 0.0,
          0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
          0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, //11 
          t7, t6, q.w() * (-0.5), - t12, 0.0, 0.0, 0.0, 0.0,
          0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
          0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, //12
          t6, - t7, t12, q.w() * (-0.5), 0.0, 0.0, 0.0, 0.0,
          0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 
          0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, //13
          0.0, 0.0, 0.0, 0.0, - t29 - t30 + t31 + t32, - t28 - q.x() * q.y() * 2.0, q.w() * q.y() * 2.0 - q.x() * q.z() * 2.0, 0.0,
          0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
          0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, //14
          0.0, 0.0, 0.0, 0.0, t28 - q.x() * q.y() * 2.0, - t29 + t30 - t31 + t32, - t38 - q.y() * q.z() * 2.0, 0.0,
          0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
          0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, //15
          0.0, 0.0, 0.0, 0.0, q.w() * q.y() * - 2.0 - q.x() * q.z() * 2.0, t38 - q.y() * q.z() * 2.0, - t29 + t30 + t31 - t32, 0.0,
          0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0,
          0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, //16
          0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
          0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
          1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, //17
          0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
          0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
          0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, //18
          0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
          0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
          0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, //19
          0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
          0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
          0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, //20
          0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
          0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
          0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, //21 
          0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
          0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
          0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, //22
          0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
          0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
          0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, //23
          0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
          0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
          0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0; //24
    
    return F_;
}

Eigen::Matrix<double, 24, 24> calcQ24(double da_Var, double dv_Var, Eigen::Quaterniond q) {
    
}
