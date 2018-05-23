#include <Eigen/Geometry>
#include <Eigen/Core>
#include <math.h>

/*Eigen::Quaterniond euler2Quaternion(double roll, double pitch, double yaw) {
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
} */

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

/*Eigen::Vector3d Quat2Euler(Eigen::Quaterniond q) {
    double roll;
    double pitch;
    double yaw;
    roll = atan2(2.0 * (q.y() * q.z() + q.w() * q.x()), 1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y()));
    pitch = asin(2.0 * (q.w() * q.y() - q.x() * q.z()));
    yaw = atan2(2.0 * (q.w() * q.z() + q.x() * q.y()), 1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
    Eigen::Vector3d Euler;
    Euler[2] = yaw;
    Euler[1] = pitch;
    Euler[0] = roll;
    return Euler;
} */

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

Eigen::Matrix<double, 24, 24> calcQ24(Eigen::Vector3d da_Var, Eigen::Vector3d dv_Var, Eigen::Quaterniond q) {
    double daxVar = da_Var[0];
    double dayVar = da_Var[1];
    double dazVar = da_Var[2];
    double dvxVar = dv_Var[0];
    double dvyVar = dv_Var[1];
    double dvzVar = dv_Var[2];
    double q0 = q.w();
    double q1 = q.x();
    double q2 = q.y();
    double q3 = q.z();
    
    double t2 = dayVar * q2 * q3 * 0.25;
    double t3 = t2 - daxVar * q0 * q1 * 0.25 - dazVar * q2 * q3 * 0.25;
    double t4 = q3 * q3;
    double t5 = q2 * q2;
    double t6 = dazVar * q1 * q3 * 0.25;
    double t7 = t6 - daxVar * q1 * q3 * 0.25 - dayVar * q0 * q2 * 0.25;
    double t8 = daxVar * q0 * q3 * 0.25;
    double t9 = t8 - dayVar * q0 * q3 * 0.25 - dazVar * q1 * q2 * 0.25;
    double t10 = q0 * q0;
    double t11 = q1 * q1;
    double t12 = daxVar * q1 * q2 * 0.25;
    double t13 = t12 - dayVar * q1 * q2 * 0.25 - dazVar * q0 * q3 * 0.25;
    double t14 = dazVar * q0 * q2 * 0.25;
    double t15 = t14 - daxVar * q0 * q2 * 0.25 - dayVar * q1 * q3 * 0.25;
    double t16 = dayVar * q0 * q1 * 0.25;
    double t17 = t16 - daxVar * q2 * q3 * 0.25 - dazVar * q0 * q1 * 0.25;
    double t21 = q0 * q3 * 2.0;
    double t22 = q1 * q2 * 2.0;
    double t18 = t21 - t22;
    double t23 = q0 * q2 * 2.0;
    double t24 = q1 * q3 * 2.0;
    double t19 = t23 + t24;
    double t20 = t4 + t5 - t10 - t11;
    double t25 = q0 * q1 * 2.0;
    double t26 = t21 + t22;
    double t32 = t4 - t5 - t10 + t11;
    double t27 = dvyVar * t18 * t32;
    double t28 = q2 * q3 * 2.0;
    double t29 = t25 - t28;
    double t30 = t4 - t5 - t10 + t11;
    double t31 = t25 + t28;
    double t33 = t4 - t5 + t10 - t11;
    double t34 = t23 - t24;
    double t35 = dvxVar * t34 * (t4 + t5 - t10 - t11);
    double t36 = dvzVar * t19 * t33;
    double t37 = t35 + t36 - dvyVar * t18 * t31;
    double t38 = - dvxVar * t26 * t34 - dvyVar * t31 * t32 - dvzVar * t29 * t33;
    
    Eigen::Matrix<double, 24, 24> Q_ = Eigen::MatrixXd::Zero(24, 24);
    Q_(0, 0) = daxVar * t11 * 0.25 + dayVar * t5 * 0.25 + dazVar * t4 * 0.25;
    Q_(0, 1) = t3;
    Q_(0, 2) = t7;
    Q_(0, 3) = t13;
    Q_(1, 0) = t3; 
    Q_(1, 1) = daxVar * t10 * 0.25 + dayVar * t4 * 0.25 + dazVar * t5 * 0.25;
    Q_(1, 2) = t9;
    Q_(1, 3) = t15;
    Q_(2, 0) = t7; 
    Q_(2, 1) = t9;
    Q_(2, 2) = daxVar * t4 * 0.25 + dayVar * t10 * 0.25 + dazVar * t11 * 0.25;
    Q_(2, 3) = t17; 
    Q_(3, 0) = t13;
    Q_(3, 1) = t15;
    Q_(3, 2) = t17;
    Q_(3, 3) = daxVar * t5 * 0.25 + dayVar * t11 * 0.25 + dazVar * t10 * 0.25;
    Q_(4, 4) = dvxVar * t20 * t20 + dvyVar * t18 * t18 + dvzVar * t19 * t19;
    Q_(4, 5) = t27 - dvxVar * t20 * t26 - dvzVar * t19 * t29;
    Q_(4, 6) = t37;
    Q_(5, 4) = t27 - dvzVar * t19 * (t25 - q2 * q3 * 2.0) - dvxVar * t20 * t26;
    Q_(5, 5) = dvxVar * t26 * t26 + dvyVar * t30 * t30 + dvzVar * t29 * t29;
    Q_(5, 6) = t38;
    Q_(6, 4) = t37;
    Q_(6, 5) = t38;
    Q_(6, 6) = dvxVar * t34 * t34 + dvyVar * t31 * t31 + dvzVar * t33 * t33;


    /*Q_ <<  daxVar * t11 * 0.25 + dayVar * t5 * 0.25 + dazVar * t4 * 0.25, t3, t7, t13, 0.0, 0.0, 0.0, 0.0, 
           0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
           0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, // 1 
           t3, daxVar * t10 * 0.25 + dayVar * t4 * 0.25 + dazVar * t5 * 0.25, t9, t15, 0.0, 0.0, 0.0, 0.0, 
           0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
           0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, // 2
           t7, t9, daxVar * t4 * 0.25 + dayVar * t10 * 0.25 + dazVar * t11 * 0.25, t17, 0.0, 0.0, 0.0, 0.0,
           0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
           0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, // 3
           t13, t15, t17, daxVar * t5 * 0.25 + dayVar * t11 * 0.25 + dazVar * t10 * 0.25, 0.0, 0.0, 0.0, 0.0, 
           0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
           0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, // 4
           0.0, 0.0, 0.0, 0.0, dvxVar * t20 * t20 + dvyVar * t18 * t18 + dvzVar * t19 * t19, t27 - dvxVar * t20 * t26 - dvzVar * t19 * t29, t37, 0.0,
           0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
           0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, // 5
           0.0, 0.0, 0.0, 0.0, t27 - dvzVar * t19 * (t25 - q2 * q3 * 2.0) - dvxVar * t20 * t26, dvxVar * t26 * t26 + dvyVar * t30 * t30 + dvzVar * t29 * t29, t38, 0.0,
           0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
           0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, // 6
           0.0, 0.0, 0.0, 0.0, t37, t38, dvxVar * t34 * t34 + dvyVar * t31 * t31 + dvzVar * t33 * t33, 0.0,
           0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
           0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, // 7
           0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
           0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
           0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, // 8
           0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
           0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
           0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, // 9
           0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
           0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
           0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, // 10
           0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
           0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
           0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, // 11
           0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
           0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
           0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, // 12
           0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
           0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
           0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, // 13
           0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
           0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
           0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, // 14
           0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
           0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
           0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, // 15
           0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
           0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
           0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, // 16
           0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
           0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
           0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, // 17
           0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
           0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
           0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, // 18
           0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
           0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
           0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, // 19
           0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
           0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
           0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, // 20
           0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
           0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, // 21
           0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
           0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
           0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, // 22
           0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
           0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
           0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, // 23
           0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
           0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
           0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0; // 24 */
    return Q_;
}


Eigen::Matrix<double, 1, 24> calcH_VELX(Eigen::Quaterniond q, Eigen::Vector3d velNED) {
    Eigen::Matrix<double, 1, 24> H_x;
    double q0 = q.w();
    double q1 = q.x();
    double q2 = q.y();
    double q3 = q.z();
    double vn = velNED[0];
    double ve = velNED[1];
    double vd = velNED[2];

    H_x << q2 * vd * (-2.0) + q3 * ve * 2.0 + q0 * vn * 2.0, q3 * vd * 2.0 + q2 * ve * 2.0 + q1 * vn * 2.0, q0 * vd * (-2.0) + q1 * ve * 2.0 - q2 * vn * 2.0, q1 * vd * 2.0 + q0 * ve * 2.0 - q3 * vn * 2.0, q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3, q0 * q3 * 2.0 + q1 * q2 * 2.0, q0 * q2 * (-2.0) + q1 * q3 * 2.0, 0.0, 
           0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 
           0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

    return H_x;
}

Eigen::Matrix<double, 1, 24> calcH_VELY(Eigen::Quaterniond q, Eigen::Vector3d velNED) {
    Eigen::Matrix<double, 1, 24> H_y;
    double q0 = q.w();
    double q1 = q.x();
    double q2 = q.y();
    double q3 = q.z();
    double vn = velNED[0];
    double ve = velNED[1];
    double vd = velNED[2];

    H_y << q1 * vd * 2.0 + q0 * ve * 2.0 - q3 * vn * 2.0, q0 * vd * 2.0 - q1 * ve * 2.0 + q2 * vn * 2.0, q3 * vd * 2.0 + q2 * ve * 2.0 + q1 * vn * 2.0, q2 * vd * 2.0 - q3 * ve * 2.0 - q0 * vn * 2.0, + q0 * q3 * (-2.0) + q1 * q2 * 2.0,q0 * q0 - q1 * q1 + q2 * q2 - q3 * q3, q0 * q1 * 2.0 + q2 * q3 * 2.0, 0.0,
           0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
           0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

    return H_y;
}

Eigen::Matrix<double, 1, 24> calcH_VELZ(Eigen::Quaterniond q, Eigen::Vector3d velNED) {
    Eigen::Matrix<double, 1, 24> H_z;
    double q0 = q.w();
    double q1 = q.x();
    double q2 = q.y();
    double q3 = q.z();
    double vn = velNED[0];
    double ve = velNED[1];
    double vd = velNED[2];

    H_z << q0 * vd * 2.0 - q1 * ve * 2.0 + q2 * vn * 2.0, + q1 * vd * (-2.0) - q0 * ve * 2.0 + q3 * vn * 2.0, q2 * vd * (-2.0) + q3 * ve * 2.0 + q0 * vn * 2.0, q3 * vd * 2.0 + q2 * ve * 2.0 + q1 * vn * 2.0, q0 * q2 * 2.0 + q1 * q3 * 2.0, q0 * q1 * (-2.0) + q2 * q3 * 2.0, q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3, 0.0,
           0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
           0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

    return H_z;
}
