/*************************************************************************
	> File Name: Node.cpp
	> Author: 
	> Mail: 
	> Created Time: 2018年04月16日 星期一 16时00分52秒
 ************************************************************************/
#include <iostream>
#include <fuse_ekf_test/Node.hpp>
#include <fuse_ekf_test/transform_math.hpp>


using namespace Eigen;
using namespace std;

namespace fuse_ekf_test {
    Node::Node(const ros::NodeHandle &nh): nh_(nh), 
    sync_(TimeSyncPolicy(kROSQueueSize), subImu_, subField_),
    calibState_(Uncalibrated), init_(false){

        
        //init states vector (4x1 quaternion, 3x1 velocity, 3x1 position, 3x1 delAng bias, 3x1 delVel bias, 3x1 mag_world)
        states_.Zero();
        q_init_angaxi = AngleAxisd(0, Vector3d (0, 0, 0));
        q_ = Quaterniond (q_init_angaxi);
        Tbn = q_.normalized().toRotationMatrix();
        magWorld_ = Vector3d::Zero();

        Covariances = MatrixXd::Zero(24, 24);
        cov_pos = Matrix3d::Zero();
        cov_vel = Matrix3d::Zero();
        dt_Cov = 0.0;
        dt_Int_Cov = 0.0;
        //cout << "q_:" << q_.coeffs() << endl;
        //cout << "q_0:" << q_.w() << endl;
        states_.segment(0,4) << q_.w(), q_.x(), q_.y(), q_.z();
        //cout << "states_:" << states_ << endl;

        // variables used to control dead-reckoning timeout
        last_dirft_constrain_time = - Node::control_param.velDriftTimeLim;
        last_synthetic_velocity_fusion_time = 0;

        //variables for prediction
        prevDelAng = Vector3d::Zero();
        prevDelVel = Vector3d::Zero();
        correctedDelAng = Vector3d::Zero();
        correctedDelVel = Vector3d::Zero();

        //subscribe ~visual ~imu and ~magnetic_field
        subVisual_ = nh_.subscribe("visual_odom", 40, &Node::visual_odom_cb, this); 
        // subLaser_ = nh_.subscribe("laser_data", 20, &Node::laser_data_cb, this);
        subImu_.subscribe(nh_, "imu", kROSQueueSize);
        subField_.subscribe(nh_, "magnetic_field", kROSQueueSize);
        sync_.registerCallback(boost::bind(&Node::imu_mag_cb, this, _1, _2));
    }

    void Node::visual_odom_cb(const nav_msgs::OdometryConstPtr& visMsg) {
        ROS_INFO_ONCE("[ VISUAL ] DATA RECEIVED !");

        
        //transform measured data
        Vector3d posm(0.0, 0.0, 0.0);
        Vector3d velm(0.0, 0.0, 0.0);
        posm << visMsg->pose.pose.position.x, visMsg->pose.pose.position.y, visMsg->pose.pose.position.z;
        velm << visMsg->twist.twist.linear.x, visMsg->twist.twist.linear.y, visMsg->twist.twist.linear.z;
        vis_Stamp = visMsg->header.stamp; 
        Vector3d posBody_ = Node::aligment_param.CamToBody * posm;
        Vector3d velBody_ = Node::aligment_param.CamToBody * velm;
        Vector3d velWorld_ = Tbn * velBody_;
        Vector3d posWorld_ = Tbn * posBody_;

        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                cov_pos(i,j) = visMsg->pose.covariance[i * 6 + j];
                // cov_vel(i,j) = visMsg->twist.covariance[i * 6 + j]; 

            }
        }
        
        Vector3d cov_vel_diag;
        cov_vel_diag << 5.0, 5.0, 1.0;
        cov_vel = cov_vel_diag.asDiagonal();
        cov_vel = cov_vel.transpose().eval() * cov_vel.eval();

        if (!Node::control_param.visualInitStates) {
            ROS_INFO("[ VISUAL ] STATES INIT !");
            InitStates_Visual(posWorld_, velWorld_);
            vis_prevStamp = vis_Stamp;
            return;
        }
        
        // convert quality metric to measurements to velocity
        // double quality = viso_data.qual; cam or body?????
        double quality = 0.5;
        double bodyVelError = Node::fusion_param.bodyVelErrorMin * quality + Node::fusion_param.bodyVelErrorMax * (1 - quality);
        double worldPosError = Node::fusion_param.worldPosErrorMin * quality + Node::fusion_param.worldPosErrorMax * (1 - quality);
        FuseBodyVel(velBody_, velWorld_, bodyVelError);
        FusePosition(posWorld_, worldPosError);


        vis_prevStamp = vis_Stamp;
    }

    void Node::laser_data_cb(const sensor_msgs::LaserScanConstPtr& laserMsg) {
        ROS_INFO_ONCE("[ LASER ] DATA RECEIVED !");

        float height(0.0);
        //height = laserMsg->ranges[];
        laser_Stamp = laserMsg->header.stamp;
        
        if (!Node::control_param.laserInitStates) {
            ROS_INFO("[ LASER ] STATES INIT !");
            InitStates_Laser(height);
            laser_prevStamp = laser_Stamp;
            return;
        }

        laser_prevStamp = laser_Stamp;
    }

    void Node::imu_mag_cb(const sensor_msgs::ImuConstPtr& imuMsg, const sensor_msgs::MagneticFieldConstPtr& magMsg) {
        ROS_INFO_ONCE("[ Imu_Mag ] DATA RECEIVED !");

        // transform measured data
        Vector3d wm(0.0, 0.0, 0.0); // measured angular rate
        Vector3d am(0.0, 0.0, 0.0); // measured acceleration
        Vector3d mm(0.0, 0.0, 0.0); // measured magnetic field
        wm << imuMsg->angular_velocity.x, imuMsg->angular_velocity.y, imuMsg->angular_velocity.z;
        am << imuMsg->linear_acceleration.x, imuMsg->linear_acceleration.y, imuMsg->linear_acceleration.z;
        mm << magMsg->magnetic_field.x, magMsg->magnetic_field.y, magMsg->magnetic_field.z;
        imu_Stamp = imuMsg->header.stamp;
        // measured covariances
        Matrix3d wCov = Matrix3d::Zero(); 
        Matrix3d aCov = Matrix3d::Zero();
        Matrix3d mCov = Matrix3d::Zero();
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                wCov(i,j) = imuMsg->angular_velocity_covariance[i * 3 + j];
                aCov(i,j) = imuMsg->linear_acceleration_covariance[i * 3 + j];
                mCov(i,j) = magMsg->magnetic_field_covariance[i * 3 + j];
            }
        }
        
        //correct magnetic field
        mm = mm.eval() - Node::fusion_param.magBias_;
        for (int i = 0; i < 3; i++) {
            mm[i] /= Node::fusion_param.magScale_[i];
        }

        // init states using imu
        if (!Node::control_param.imuInitStates) {
            ROS_INFO("[ Imu_Mag ] STATES INIT !");
            InitStates_Imu(wm, am, mm);
            imu_prevStamp = imu_Stamp;
            return;
        }

        if (!Node::control_param.covInit) {
            if (Node::control_param.imuInitStates & Node::control_param.visualInitStates &
                Node::control_param.laserInitStates ) {
                ROS_INFO("[ Covariances ] COV INIT !");
                double delta_t_imu;
                delta_t_imu = (imu_Stamp - imu_prevStamp).toSec();
                InitCovariance(delta_t_imu);    
            } else {
                ROS_INFO("[ Covariances ] Imu or VisualOdom is not inited !"); 
                imu_prevStamp = imu_Stamp;
                return;
            }
            
        }

        if (Node::control_param.imuInitStates & Node::control_param.visualInitStates &
            Node::control_param.laserInitStates & Node::control_param.covInit) {
            ROS_INFO_ONCE("[ ALL INIT DONE ] IMU VIS LASER COV init done !");

            PredictStates(wm, am, mm);
        } else {
            ROS_INFO("[ Predict error ] sensor init error !");
            imu_prevStamp = imu_Stamp;
            return;
        }

        // constrain states
        ConstrainStates();

        dt_Cov += (imu_Stamp - imu_prevStamp).toSec();
        delAng_Cov += correctedDelAng;
        delVel_Cov += correctedDelVel;
        if (dt_Cov > 0.01) {
            PredictCovariance();
            delAng_Cov = Vector3d::Zero();
            delVel_Cov = Vector3d::Zero();
            dt_Int_Cov += dt_Cov;
            dt_Cov = 0;
        }

        imu_prevStamp = imu_Stamp;
    } 

    void Node::InitStates_Imu(Vector3d measured_wm, Vector3d measured_am, Vector3d measured_mm) {
        // init quat using acc
        double lengthAccel = sqrt(measured_am.dot(measured_am));
        if (lengthAccel > 5 && lengthAccel < 14) {
            double tiltAng;
            Vector3d tiltUnitVec;
            Vector3d tiltUnitVec_;
            tiltAng = atan2(sqrt(measured_am[0] * measured_am[0] + measured_am[1] * measured_am[1]), -measured_am[2]);
            if (tiltAng > 1e-3) {
                tiltUnitVec = measured_am.cross(Vector3d (0, 0, -1));
                tiltUnitVec_ = tiltUnitVec / sqrt(tiltUnitVec.dot(tiltUnitVec));
                tiltUnitVec = tiltUnitVec_;
                AngleAxisd accInitQ(tiltAng,tiltUnitVec);
                q_ = Quaterniond(accInitQ); 
            }
        }
        states_.segment(0,4) << q_.w(), q_.x(), q_.y(), q_.z();
            
        // add a roll pitch yaw mislignment
        Quaterniond align_err_q = euler2Quaternion(Node::control_param.rollAlignErr, Node::control_param.pitchAlignErr, Node::control_param.yawAlignErr);
        q_ = QuatMult(align_err_q, q_);
        states_.segment(0,4) << q_.w(), q_.x(), q_.y(), q_.z();

        // init quat using mag
        Vector3d euler_except_yaw = q_.toRotationMatrix().eulerAngles(2, 1, 0); // yaw pitch roll
        euler_except_yaw[2] = 0.0;
        Quaterniond quat_except_yaw = euler2Quaternion(euler_except_yaw[2], euler_except_yaw[1], euler_except_yaw[0]);
        Matrix3d rotation_except_yaw = quat_except_yaw.toRotationMatrix();
        Vector3d mm_NED = rotation_except_yaw * measured_mm;
        euler_except_yaw[2] =  Node::fusion_param.magDeclDeg * (M_PI / 180) - atan2(mm_NED[1], mm_NED[0]);
        q_ = euler2Quaternion(euler_except_yaw[2], euler_except_yaw[1], euler_except_yaw[0]);
        states_.segment(0,4) << q_.w(), q_.x(), q_.y(), q_.z();
           
        Tbn = q_.normalized().toRotationMatrix();
        magWorld_ = Tbn * measured_mm;
        states_.segment(16,3) << magWorld_[0], magWorld_[1], magWorld_[2]; 


        Node::control_param.imuInitStates = true;

    }

    void Node::InitCovariance(double delta_t) {
        // states errors : q, vel, pos, angel_bias(dt), vel_bias(dt), magNED, magXYZ, wind 
        VectorXd Cov_diag(24);
        Cov_diag = VectorXd::Zero(24);
        Matrix<double, 24, 24> Covariances_sqrt;
        Cov_diag.segment(0,4) << aligment_param.quatErr * 1, aligment_param.quatErr * 1, aligment_param.quatErr * 1, aligment_param.quatErr * 1;
        Cov_diag.segment(10,3) << aligment_param.delAngBiasErr * delta_t, aligment_param.delAngBiasErr * delta_t, aligment_param.delAngBiasErr * delta_t;
        Cov_diag.segment(13,3) << aligment_param.delVelBiasErr * delta_t, aligment_param.delVelBiasErr * delta_t, aligment_param.delVelBiasErr * delta_t;
        Cov_diag.segment(16,3) << aligment_param.magErrNED, aligment_param.magErrNED, aligment_param.magErrNED;
        Cov_diag.segment(19,3) << aligment_param.magErrXYZ, aligment_param.magErrXYZ, aligment_param.magErrXYZ;
        Cov_diag.segment(22,2) << aligment_param.windErrNE, aligment_param.windErrNE, aligment_param.windErrNE;
        Covariances_sqrt = Cov_diag.asDiagonal();
        Covariances = Covariances_sqrt.transpose().eval() * Covariances_sqrt.eval();
        Covariances.block<3, 3>(4, 4) = cov_vel;
        Covariances.block<3, 3>(7, 7) = cov_pos;

        Node::control_param.covInit = true;
    }

    void Node::InitStates_Visual(Vector3d measured_pos, Vector3d measured_vel) {
        states_.segment(4,3) << measured_vel[0], measured_vel[1], measured_vel[2];
        states_.segment(7,3) << measured_pos[0], measured_pos[1], measured_pos[2];
        //prevPosWorld_[0] = measured_pos[0];
        //prevPosWorld_[1] = measured_pos[1];
        prevPosWorld_ << measured_pos[0], measured_pos[1], measured_pos[2];
        prevVelWorld_ << measured_vel[0], measured_vel[1], measured_vel[2];

        Node::control_param.visualInitStates = true;
    }

    void Node::InitStates_Laser(float measured_height) {
        states_.segment(9,1) << measured_height;

        Node::control_param.laserInitStates = true;
    }

    void Node::PredictStates(Eigen::Vector3d measured_wm_, Eigen::Vector3d measured_am_, Eigen::Vector3d measured_mm_) {
        double del_t;
        Vector3d delAng;
        Vector3d delVel;
        del_t = (imu_Stamp - imu_prevStamp).toSec();
        delAng = measured_wm_ * del_t;
        delVel = measured_am_ * del_t;
        if (!Node::prediction_param.prevDelAngSet) {
            prevDelAng = delAng;
        }
        if (!Node::prediction_param.prevDelVelSet) {
            prevDelVel = delVel;
        }

        //Remove sensor bias errors
        delAng_bias << states_[10], states_[11], states_[12];
        delVel_bias << states_[13], states_[14], states_[15];
        delAng = delAng - delAng_bias;
        delVel = delVel - delVel_bias;
        // correctedDelVel = delVel + 0.5 * (prevDelAng + delAng).cross(prevDelVel + delVel) + 1/6 * (prevDelAng + delAng).cross((prevDelAng + delAng).cross(prevDelVel + delVel)) + 1/12 * (prevDelAng.cross(delVel) + prevDelVel.cross(delAng));
        correctedDelVel = delVel;
        correctedDelAng = delAng - 1/12 * prevDelAng.cross(delAng);
        // save current measurements
        prevDelAng = delAng;
        prevDelVel = delVel;

        // convert the rotation vector to its equivalent quaternion
        Quaterniond deltaQuat;
        deltaQuat = RotVector2Quat(correctedDelAng);
        q_ = QuatMult(q_, deltaQuat);
        q_.normalized();
        states_.segment(0,4) << q_.w(), q_.x(), q_.y(), q_.z();
        Tbn = q_.normalized().toRotationMatrix();

        // transform body delta velocities to delta velocities in the nav frame
        Vector3d delVelNav;
        Vector3d g_vec;
        g_vec << 0, 0, gravity;
        delVelNav = Tbn * correctedDelVel + g_vec * del_t;
        Vector3d prevVel;
        prevVel << states_[4], states_[5], states_[6];
        states_.segment(4,3) << states_[4] + delVelNav[0], states_[5] + delVelNav[1], states_[6] + delVelNav[2];

        // integrate the velocity vector to get the position using trapezoidal
        states_.segment(7,3) << states_[7] + 0.5 * del_t * (prevVel[0] + states_[4]),
                                states_[8] + 0.5 * del_t * (prevVel[1] + states_[5]),
                                states_[9] + 0.5 * del_t * (prevVel[2] + states_[6]);
    }

    void Node::ConstrainStates() {
        double del_t;
        del_t = (imu_Stamp - imu_prevStamp).toSec();
        double limit = 5.0 * M_PI / 180 * del_t;
        for (int i = 10; i < 13; i++) {
            if (states_[i] > limit) 
                states_[i] = limit;
            else if (states_[i] < - limit)
                states_[i] = - limit;
        }
    }

    void Node::PredictCovariance() {
        double dAngBiasSigma;
        double dVelBiasSigma;
        double magSigmaNED;
        double magSigmaXYZ;
        Matrix<double, 24, 1> processNoiseVariance_diag;
        Matrix<double, 24, 24> processNoiseVariance;
        Matrix<double, 24, 24> processNoiseVariance_squa;
        dAngBiasSigma = dt_Cov * dt_Cov * Node::prediction_param.dAngBiasPnoise;
        dVelBiasSigma = dt_Cov * dt_Cov * Node::prediction_param.dVelBiasPnoise;
        magSigmaNED = dt_Cov * Node::prediction_param.magPnoiseNED;
        magSigmaXYZ = dt_Cov * Node::prediction_param.magPnoiseXYZ;
        processNoiseVariance_diag << 0.0, 0.0, 0.0, 0.0, 
                                     0.0, 0.0, 0.0, 
                                     0.0, 0.0, 0.0,
                                     dAngBiasSigma, dAngBiasSigma, dAngBiasSigma,
                                     dVelBiasSigma, dVelBiasSigma, dVelBiasSigma,
                                     magSigmaNED, magSigmaNED, magSigmaNED,
                                     magSigmaXYZ, magSigmaXYZ, magSigmaXYZ,
                                     0.0, 0.0;
        processNoiseVariance = processNoiseVariance_diag.asDiagonal();
        processNoiseVariance_squa = processNoiseVariance.transpose().eval() * processNoiseVariance.eval();
        
        Matrix<double, 24, 24> F;
        Matrix<double, 24, 24> Q;

        Vector3d dA = delAng_Cov;
        Vector3d dA_b = delAng_bias;
        Vector3d dV = delVel_Cov;
        Vector3d dV_b = delVel_bias;
        double dt = dt_Cov;
        F = calcF24(dA, dV, dA_b, dV_b, dt, q_);
        
        double daxVar = pow(dt_Cov * Node::prediction_param.angRateNoise, static_cast<double>(2.0));
        double dvxVar = pow(dt_Cov * Node::prediction_param.accelNoise, static_cast<double>(2.0));
        Vector3d daVar;
        daVar << daxVar, daxVar, daxVar;
        Vector3d dvVar;
        dvVar << dvxVar, dvxVar, dvxVar;
        Q = calcQ24(daVar, dvVar, q_);

        Matrix<double, 24, 24> P_;
        P_ = F * Covariances * F.transpose().eval() + Q;
        P_= P_.eval() + processNoiseVariance_squa;
        Covariances = 0.5 * (P_ + P_.transpose());

        for (int i = 0; i < 24; i++) {
            if(Covariances(i, i) < 0)
                Covariances(i, i) = 0;
        }

    }

    void Node::FuseBodyVel(Vector3d measured_vel_, Vector3d velWorld,  double bodyVelError_) {
        Vector3d innovation = Vector3d::Zero(); // Body velocity innovation (m/s)
        Vector3d varInnov = Vector3d::Zero(); // NED velocity innovation variance ((m/s)^2)
        Matrix<double, 3, 24> H = MatrixXd::Zero(3, 24);
        Vector3d relVelBodyPred = Tbn.transpose() * prevVelWorld_;
        for (int i = 0; i < 3; i++) {
            if (i == 0)
                H.row(0) = calcH_VELX(q_, velWorld); 
            else if (i == 1)
                H.row(1) = calcH_VELY(q_, velWorld);
            else if (i == 2)
                H.row(2) = calcH_VELZ(q_, velWorld);
        
            varInnov[i] = H.row(i) * Covariances * H.row(i).transpose().eval() + bodyVelError_ * bodyVelError_;
            innovation[i] = relVelBodyPred[i] - measured_vel_[i];
        }

        for (int i = 0; i < 3; i++) {
            if (innovation[i] * innovation[i] / (varInnov[i] * Node::fusion_param.bodyVelGate * Node::fusion_param.bodyVelGate) > 1.0)
                return;
        }

        Matrix<double, 24, 1> Kfusion;
        for (int i = 0; i < 3; i++) {
            Kfusion = (Covariances * H.row(i).transpose().eval()) / varInnov[i];
            states_ = states_.eval() - Kfusion * innovation[i];
            q_.w() = states_[0];
            q_.x() = states_[1];
            q_.y() = states_[2];
            q_.z() = states_[3];
            q_.normalized();
            states_.segment(0, 4) << q_.w(), q_.x(), q_.y(), q_.z();
            Covariances = Covariances.eval() - (Kfusion * H.row(i) * Covariances).eval();
            Covariances = 0.5 * (Covariances.eval() + Covariances.transpose().eval());

            for (int j = 0; j < 24; j++) {
                if (Covariances(j, j) < 0)
                    Covariances(j, j) = 0;
            }
        }

        prevVelWorld_ << states_[4], states_[5], states_[6];
    }

    void Node::FusePosition(Vector3d measured_pos_, double worldPosError_) {
        Vector3d innovation = Vector3d::Zero(); // NED position innovation (m)
        Vector3d varInnov = Vector3d::Zero(); // NED position innovation variance ((m)^2)
        Matrix<double, 3, 24> H = MatrixXd::Zero(3, 24);
        int stateIndex;

        for (int i = 0; i < 3; i++ ) {
            stateIndex = i + 7;
            innovation[i] = prevPosWorld_[i] - measured_pos_[i];
            H(i, stateIndex) = 1;
            varInnov[i] = (H.row(i) * Covariances * H.row(i).transpose().eval() + worldPosError_ * worldPosError_);

        }

        for (int i = 0; i < 3; i++ ) {
            if (innovation[i] * innovation[i] / (varInnov[i] * Node::fusion_param.worldPosGate * Node::fusion_param.worldPosGate) > 1.0)
                return;
        }

        Matrix<double, 24, 1> Kfusion; 
        for (int i = 0; i < 3; i++ ) {
            Kfusion = (Covariances * H.row(i).transpose().eval()) / varInnov[i];
            states_ = states_.eval() - Kfusion * innovation[i];
            Covariances = Covariances.eval() - (Kfusion * H.row(i) * Covariances).eval();
            Covariances = 0.5 * (Covariances.eval() + Covariances.transpose().eval());

            for (int j = 0; j < 24; j++) {
                if (Covariances(j, j) < 0)
                    Covariances(j, j) = 0;
            }
        }

        prevPosWorld_ << states_[7], states_[8], states_[9];
    }
}
