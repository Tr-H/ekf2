/*************************************************************************
	> File Name: Node.cpp
	> Author: 
	> Mail: 
	> Created Time: 2018年04月16日 星期一 16时00分52秒
 ************************************************************************/
#include <iostream>
#include <fuse_ekf_test/Node.hpp>
#include <fuse_ekf_test/transform_math.hpp>
#include "geometry_math_type.h"

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
        get_dcm_from_q(Tbn, q_);
        //Tbn = q_.normalized().toRotationMatrix();
        magWorld_ = Vector3d::Zero();

        // rviz
        marker.header.frame_id = "base_link";
        marker.header.stamp = ros::Time();
        marker.ns = "fuse_ekf_test";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::ARROW;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = 1.0;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;

        // predict covariance
        Covariances = MatrixXd::Zero(24, 24);
        //cov_pos = Matrix3d::Zero();
        //cov_vel = Matrix3d::Zero();
        _ang_rate_mag_filt = 0.0; 
        _accel_mag_filt = 0.0; 
        _accel_bias_inhibit = false;
        _bad_vert_accel_detected = false;	

        dt_Cov = 0.0;
        dt_Int_Cov = 0.0;
        //cout << "q_:" << q_.coeffs() << endl;
        states_.segment(0,4) << q_.w(), q_.x(), q_.y(), q_.z();

        //variables for prediction
        prevDelAng = Vector3d::Zero();
        prevDelVel = Vector3d::Zero();
        correctedDelAng = Vector3d::Zero();
        correctedDelVel = Vector3d::Zero();

        prevPosWorld_ = Vector3d::Zero();
        vis_prevStamp = ros::Time::now();
        
        cam_to_body = AngleAxisd(M_PI_2, Vector3d (0, 1, 0));
        q_cam_to_body = Quaterniond (cam_to_body);
        get_dcm_from_q(R_cam_to_body, q_cam_to_body);

        //subscribe ~visual ~imu and ~magnetic_field
        subVisual_ = nh_.subscribe("/svo/pose_imu", 40, &Node::visual_odom_cb, this); 
        // subLaser_ = nh_.subscribe("laser_data", 20, &Node::laser_data_cb, this);
        subImu_.subscribe(nh_, "/mavros/imu/data_raw", kROSQueueSize);
        subField_.subscribe(nh_, "/mavros/imu/mag", kROSQueueSize);
        sync_.registerCallback(boost::bind(&Node::imu_mag_cb, this, _1, _2));
        //rosbag_imu = nh_.subscribe("/sync/imu/imu", 40, &Node::imu_mag_cb, this);

        rviz_pub = nh_.advertise<visualization_msgs::Marker> ("visualization_marker", 0);

#ifdef USE_LOGGER
            start_logger(ros::Time::now());
#endif
    }

    void Node::imu_mag_cb(const sensor_msgs::ImuConstPtr& imuMsg , const sensor_msgs::MagneticFieldConstPtr& magMsg) {
        ROS_INFO_ONCE("[ Imu_Mag ] DATA RECEIVED !");
        // transform measured data
        Vector3d wm(0.0, 0.0, 0.0); // measured angular rate
        Vector3d am(0.0, 0.0, 0.0); // measured acceleration
        Vector3d mm(0.0, 0.0, 0.0); // measured magnetic field
        wm << imuMsg->angular_velocity.x, imuMsg->angular_velocity.y, imuMsg->angular_velocity.z;
        am << imuMsg-> linear_acceleration.x, imuMsg->linear_acceleration.y, imuMsg->linear_acceleration.z;
        //mm << magMsg->magnetic_field.x, magMsg->magnetic_field.y, magMsg->magnetic_field.z;
        imu_Stamp = imuMsg->header.stamp;
        // measured covariances
        //Matrix3d wCov = Matrix3d::Zero(); 
        //Matrix3d aCov = Matrix3d::Zero();
        //Matrix3d mCov = Matrix3d::Zero();
        //for (int i = 0; i < 3; i++) {
        //    for (int j = 0; j < 3; j++) {
        //        wCov(i,j) = imuMsg->angular_velocity_covariance[i * 3 + j];
        //        aCov(i,j) = imuMsg->linear_acceleration_covariance[i * 3 + j];
        //        mCov(i,j) = magMsg->magnetic_field_covariance[i * 3 + j];
        //    }
        //}
        
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
                initializeCovariance();
                //InitCovariance();    
            } else {
                ROS_INFO_ONCE("[ Covariances ] VisualOdom is not inited !"); 
                imu_prevStamp = imu_Stamp;
                return;
            }
        }

        if (Node::control_param.imuInitStates & Node::control_param.visualInitStates &
            Node::control_param.laserInitStates & Node::control_param.covInit) {
            ROS_INFO_ONCE("[ ALL INIT DONE ] IMU VIS LASER COV init done !");
        } else {
            imu_prevStamp = imu_Stamp;
            return;
        }

        PredictStates(wm, am, mm);
        // constrain states
        ConstrainStates();

        dt_Cov += (imu_Stamp - imu_prevStamp).toSec();
        delAng_Cov += correctedDelAng;
        delVel_Cov += correctedDelVel;
        if (dt_Cov > 0.01) {
            // PredictCovariance();
            predictCov(delAng_Cov, delVel_Cov, delAng_bias, delVel_bias, dt_Cov, q_);
            delAng_Cov = Vector3d::Zero();
            delVel_Cov = Vector3d::Zero();
            dt_Int_Cov += dt_Cov;
            dt_Cov = 0;
        }
        imu_prevStamp = imu_Stamp;
    } 


    void Node::visual_odom_cb(const geometry_msgs::PoseWithCovarianceStampedConstPtr& visMsg) {
        ROS_INFO_ONCE("[ VISUAL ] DATA RECEIVED !");
        //transform measured data
        Vector3d posm(0.0, 0.0, 0.0);
        Vector3d velm(0.0, 0.0, 0.0);
        posm << visMsg->pose.pose.position.x, visMsg->pose.pose.position.y, visMsg->pose.pose.position.z;
        posm = R_cam_to_body * posm.eval();

        Quaterniond q_measured_yaw;
        Quaterniond q_measured_yaw_;
        Eigen::Matrix3d R_measured_yaw;
        Eigen::Matrix3d R_measured_yaw_;
        q_measured_yaw.w() = visMsg->pose.pose.orientation.w;
        q_measured_yaw.x() = visMsg->pose.pose.orientation.z;
        q_measured_yaw.y() = visMsg->pose.pose.orientation.y;
        q_measured_yaw.z() = - visMsg->pose.pose.orientation.x;
        //get_dcm_from_q(R_measured_yaw, q_measured_yaw);
        //R_measured_yaw_ = R_cam_to_body * R_measured_yaw.eval();
        //get_q_from_dcm(q_measured_yaw_, R_measured_yaw_);

        Eigen::Vector3d euler_test;
        get_euler_from_q(euler_test, q_measured_yaw);
        cout << "euler: " << euler_test[0] << " , " << euler_test[1] << " , " << euler_test[2] << endl;

        Vector3d posBody_ = Node::aligment_param.CamToBody * posm;
        get_dcm_from_q(Tbn, q_);
        Vector3d posWorld_ = Tbn * posBody_;
        // velm << visMsg->twist.twist.linear.x, visMsg->twist.twist.linear.y, visMsg->twist.twist.linear.z;
        vis_Stamp = visMsg->header.stamp; 
        Vector3d velWorld_ = (posWorld_ - prevPosWorld_) / (vis_Stamp - vis_prevStamp).toSec();
        Vector3d velBody_ = Tbn.transpose() * velWorld_; 

        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                cov_pos(i,j) = visMsg->pose.covariance[i * 6 + j];
                // cov_vel(i,j) = visMsg->twist.covariance[i * 6 + j]; 
            }
        }
        
        //Vector3d cov_vel_diag;
        //cov_vel_diag << 1.0, 1.0, 1.0;       // 5 5 1
        //cov_vel = cov_vel_diag.asDiagonal();
        //cov_vel = cov_vel.transpose().eval() * cov_vel.eval();

        if (!Node::control_param.visualInitStates) {
            ROS_INFO("[ VISUAL ] STATES INIT !");
            InitStates_Visual(posWorld_, velWorld_);
            vis_prevStamp = vis_Stamp;
            return;
        }

        if (!Node::control_param.covInit) {
            ROS_INFO_ONCE("[ Covariances ] Imu is not inited");
            return;
        }
        
        // convert quality metric to measurements to velocity
        // double quality = viso_data.qual; cam or body?????
        double quality = 0.1;
        double bodyVelError = Node::fusion_param.bodyVelErrorMin * quality + Node::fusion_param.bodyVelErrorMax * (1 - quality);
        double worldPosError = Node::fusion_param.worldPosErrorMin * quality + Node::fusion_param.worldPosErrorMax * (1 - quality);
        //FuseBodyVel(velBody_, velWorld_, bodyVelError);
        //FusePosition(posWorld_, worldPosError);

        fuseVelPosHeight(velWorld_, posWorld_);
        fuseYaw(q_measured_yaw);

        prevVelWorld_ = velWorld_;
        prevPosWorld_ = posWorld_;

#ifdef USE_LOGGER
        Vector3d euler_;
        get_euler_from_q(euler_, q_);
        if (ekf_logger.is_open()) {
            ekf_logger << vis_Stamp.toNSec() << ",";
            ekf_logger << euler_[0] << ",";
            ekf_logger << euler_[1] << ",";
            ekf_logger << euler_[2] << ",";
            ekf_logger << states_[4]<< ",";
            ekf_logger << states_[5] << ",";
            ekf_logger << states_[6] << ",";
            ekf_logger << states_[7] << ",";
            ekf_logger << states_[8] << ",";
            ekf_logger << states_[9] << ",";
            ekf_logger << states_[16] << ",";
            ekf_logger << states_[17] << ",";
            ekf_logger << states_[18] << endl; 
        }
#endif

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

    void Node::InitStates_Imu(Vector3d measured_wm, Vector3d measured_am, Vector3d measured_mm) {
        // init quat using acc
        double lengthAccel = measured_am.norm();
        if (lengthAccel > 5 && lengthAccel < 14) {
            double tiltAng;
            Vector3d tiltUnitVec;
            Vector3d tiltUnitVec_;
            tiltAng = atan2(sqrt(measured_am[0] * measured_am[0] + measured_am[1] * measured_am[1]), -measured_am[2]);
            if (tiltAng > 1e-3) {
                tiltUnitVec = measured_am.cross(Vector3d (0, 0, -1));
                tiltUnitVec_ = tiltUnitVec.normalized();
                tiltUnitVec = tiltUnitVec_;
                AngleAxisd accInitQ(tiltAng,tiltUnitVec);
                q_ = Quaterniond(accInitQ); 
            }
        }
        states_.segment(0,4) << q_.w(), q_.x(), q_.y(), q_.z();
            
        // add a roll pitch yaw mislignment
        Quaterniond align_err_q;
        Vector3d AlignErr;
        AlignErr << Node::control_param.rollAlignErr, Node::control_param.pitchAlignErr, Node::control_param.yawAlignErr;
        get_q_from_euler(align_err_q, AlignErr);
        q_ = QuatMult(align_err_q, q_);
        states_.segment(0,4) << q_.w(), q_.x(), q_.y(), q_.z();

        // init quat using mag
        Vector3d euler_except_yaw;
        get_euler_from_q(euler_except_yaw, q_); // roll pitch yaw
        // Vector3d euler_except_yaw = q_.toRotationMatrix().eulerAngles(2, 1, 0); // yaw pitch roll
        euler_except_yaw[2] = 0.0;
        Quaterniond quat_except_yaw;
        get_q_from_euler(quat_except_yaw, euler_except_yaw);
        // cout << "q_except_yaw" << quat_except_yaw.toRotationMatrix().eulerAngles(2,1,0).transpose() << endl;

        Matrix3d rotation_except_yaw ;
        get_dcm_from_q(rotation_except_yaw, quat_except_yaw);
        Vector3d mm_NED = rotation_except_yaw * measured_mm;
        //euler_except_yaw[2] =  Node::fusion_param.magDeclDeg * (M_PI / 180) - atan2f(mm_NED[1], mm_NED[0]);
        euler_except_yaw[2] = 0.0; //atan2f(mm_NED[1], mm_NED[0]);
        get_q_from_euler(q_, euler_except_yaw);
        states_.segment(0,4) << q_.w(), q_.x(), q_.y(), q_.z();
        q_ = q_.normalized();   
        get_dcm_from_q(Tbn, q_);
        //cout << "init att rpy: " << euler_except_yaw[0] << " , " << euler_except_yaw[1] << " , " << euler_except_yaw[2] << endl;
        magWorld_ = Tbn * measured_mm;
        states_.segment(16,3) << magWorld_[0], magWorld_[1], magWorld_[2]; 

        Node::control_param.imuInitStates = true;
    } 

    /*void Node::InitCovariance() {
        // states errors : q, vel, pos, angel_bias(dt), vel_bias(dt), magNED, magXYZ, wind 
        double delta_t = 0.001f * 8.0f;
        VectorXd Cov_diag(24);
        Cov_diag = VectorXd::Zero(24);
        Matrix<double, 24, 24> Covariances_sqrt;
        Cov_diag.segment(0,4) << aligment_param.quatErr * 1, aligment_param.quatErr * 1, aligment_param.quatErr * 1, aligment_param.quatErr * 1;
        Cov_diag.segment(10,3) << aligment_param.delAngBiasErr * delta_t, aligment_param.delAngBiasErr * delta_t, aligment_param.delAngBiasErr * delta_t;
        Cov_diag.segment(13,3) << aligment_param.delVelBiasErr * delta_t, aligment_param.delVelBiasErr * delta_t, aligment_param.delVelBiasErr * delta_t;
        Cov_diag.segment(16,3) << aligment_param.magErrNED, aligment_param.magErrNED, aligment_param.magErrNED;
        Cov_diag.segment(19,3) << aligment_param.magErrXYZ, aligment_param.magErrXYZ, aligment_param.magErrXYZ;
        Cov_diag.segment(22,2) << aligment_param.windErrNE, aligment_param.windErrNE;
        Covariances_sqrt = Cov_diag.asDiagonal();
        Covariances = Covariances_sqrt.transpose().eval() * Covariances_sqrt.eval();
        //Covariances.block<3, 3>(4, 4) = cov_vel;
        //Covariances.block<3, 3>(7, 7) = cov_pos;

        Node::control_param.covInit = true;
    } */

    void Node::initializeCovariance() {
        float P[24][24] = {0.0f};
        float dt = 0.001f * 8.0f;
        Vector3f rot_vec_var;
        rot_vec_var(2) = rot_vec_var(1) = rot_vec_var(0) = sq(Node::aligment_param.quatErr);

        initialiseQuatCovariances(rot_vec_var);

        // velocity
        P[4][4] = sq(fmaxf(Node::aligment_param.zedVelNoise, 0.01f));
        P[5][5] = P[4][4];
        P[6][6] = sq(1.5f) * P[4][4];

    	// position
    	P[7][7] = sq(fmaxf(Node::aligment_param.zedPosNoise, 0.01f));
    	P[8][8] = P[7][7];
        if (Node::control_param.SinglePointFusion) {
            P[9][9] = sq(fmaxf(Node::aligment_param.singlePointNoise, 0.01f));
        } else if (Node::control_param.zedHeightFusion) {
            P[9][9] = sq(fmaxf(Node::aligment_param.zedPosNoise, 0.01f));
        }
        // gyro bias
        P[10][10] = sq(Node::aligment_param.switch_on_gyro_bias * dt);
        P[11][11] = P[10][10];
        P[12][12] = P[10][10];

        // accel bias
        _prev_dvel_bias_var(0) = P[13][13] = sq(Node::aligment_param.switch_on_accel_bias * dt);
        _prev_dvel_bias_var(1) = P[14][14] = P[13][13];
        _prev_dvel_bias_var(2) = P[15][15] = P[13][13];

        // record IMU bias state covariance reset time - used to prevent resets being performed too often
        _last_imu_bias_cov_reset_us = imu_Stamp.toNSec() * 1e3f;

        // variances for optional states

        // earth frame and body frame magnetic field
        // set to observation variance
        for (uint8_t index = 16; index <= 21; index ++) {
            P[index][index] = sq(Node::aligment_param.mag_noise);
        }

        // wind
        P[22][22] = sq(Node::aligment_param.windErrNE);
        P[23][23] = sq(Node::aligment_param.windErrNE);
        for (unsigned i = 4; i < 24; i++) {
            for (unsigned j = 4; j < 24; j++) {
                Covariances(i, j) = P[i][j];
            }
        }

        Node::control_param.covInit = true;
    }
    
    void Node::initialiseQuatCovariances(Vector3f &rot_vec_var) {
        float P[24][24];
        float q0, q1, q2, q3;
        if (states_[0] >= 0.0f) {
            q0 = states_[0];
            q1 = states_[1];
            q2 = states_[2];
            q3 = states_[3];
        } else {
            q0 = -states_[0];
            q1 = -states_[1];
            q2 = -states_[2];
            q3 = -states_[3];
        }
        float delta = 2.0f*acosf(q0);
        float scaler = (delta/sinf(delta*0.5f));
        float rotX = scaler*q1;
        float rotY = scaler*q2;
        float rotZ = scaler*q3;

        // autocode generated using matlab symbolic toolbox
        float t2 = rotX*rotX;
        float t4 = rotY*rotY;
        float t5 = rotZ*rotZ;
        float t6 = t2+t4+t5;
        if (t6 > 1e-9f) {
            float t7 = sqrtf(t6);
            float t8 = t7*0.5f;
            float t3 = sinf(t8);
            float t9 = t3*t3;
            float t10 = 1.0f/t6;
            float t11 = 1.0f/sqrtf(t6);
            float t12 = cosf(t8);
            float t13 = 1.0f/powf(t6,1.5f);
            float t14 = t3*t11;
            float t15 = rotX*rotY*t3*t13;
            float t16 = rotX*rotZ*t3*t13;
            float t17 = rotY*rotZ*t3*t13;
            float t18 = t2*t10*t12*0.5f;
            float t27 = t2*t3*t13;
            float t19 = t14+t18-t27;
            float t23 = rotX*rotY*t10*t12*0.5f;
            float t28 = t15-t23;
            float t20 = rotY*rot_vec_var(1)*t3*t11*t28*0.5f;
            float t25 = rotX*rotZ*t10*t12*0.5f;
            float t31 = t16-t25;
            float t21 = rotZ*rot_vec_var(2)*t3*t11*t31*0.5f;
            float t22 = t20+t21-rotX*rot_vec_var(0)*t3*t11*t19*0.5f;
            float t24 = t15-t23;
            float t26 = t16-t25;
            float t29 = t4*t10*t12*0.5f;
            float t34 = t3*t4*t13;
            float t30 = t14+t29-t34;
            float t32 = t5*t10*t12*0.5f;
            float t40 = t3*t5*t13;
            float t33 = t14+t32-t40;
            float t36 = rotY*rotZ*t10*t12*0.5f;
            float t39 = t17-t36;
            float t35 = rotZ*rot_vec_var(2)*t3*t11*t39*0.5f;
            float t37 = t15-t23;
            float t38 = t17-t36;
            float t41 = rot_vec_var(0)*(t15-t23)*(t16-t25);
            float t42 = t41-rot_vec_var(1)*t30*t39-rot_vec_var(2)*t33*t39;
            float t43 = t16-t25;
            float t44 = t17-t36;

            // zero all the quaternion covariances
            zeroRows(P,0,3);
            zeroCols(P,0,3);

            // Update the quaternion internal covariances using auto-code generated using matlab symbolic toolbox
            P[0][0] = rot_vec_var(0)*t2*t9*t10*0.25f+rot_vec_var(1)*t4*t9*t10*0.25f+rot_vec_var(2)*t5*t9*t10*0.25f;
            P[0][1] = t22;
            P[0][2] = t35+rotX*rot_vec_var(0)*t3*t11*(t15-rotX*rotY*t10*t12*0.5f)*0.5f-rotY*rot_vec_var(1)*t3*t11*t30*0.5f;
            P[0][3] = rotX*rot_vec_var(0)*t3*t11*(t16-rotX*rotZ*t10*t12*0.5f)*0.5f+rotY*rot_vec_var(1)*t3*t11*(t17-rotY*rotZ*t10*t12*0.5f)*0.5f-rotZ*rot_vec_var(2)*t3*t11*t33*0.5f;
            P[1][0] = t22;
            P[1][1] = rot_vec_var(0)*(t19*t19)+rot_vec_var(1)*(t24*t24)+rot_vec_var(2)*(t26*t26);
            P[1][2] = rot_vec_var(2)*(t16-t25)*(t17-rotY*rotZ*t10*t12*0.5f)-rot_vec_var(0)*t19*t28-rot_vec_var(1)*t28*t30;
            P[1][3] = rot_vec_var(1)*(t15-t23)*(t17-rotY*rotZ*t10*t12*0.5f)-rot_vec_var(0)*t19*t31-rot_vec_var(2)*t31*t33;
            P[2][0] = t35-rotY*rot_vec_var(1)*t3*t11*t30*0.5f+rotX*rot_vec_var(0)*t3*t11*(t15-t23)*0.5f;
            P[2][1] = rot_vec_var(2)*(t16-t25)*(t17-t36)-rot_vec_var(0)*t19*t28-rot_vec_var(1)*t28*t30;
            P[2][2] = rot_vec_var(1)*(t30*t30)+rot_vec_var(0)*(t37*t37)+rot_vec_var(2)*(t38*t38);
            P[2][3] = t42;
            P[3][0] = rotZ*rot_vec_var(2)*t3*t11*t33*(-0.5f)+rotX*rot_vec_var(0)*t3*t11*(t16-t25)*0.5f+rotY*rot_vec_var(1)*t3*t11*(t17-t36)*0.5f;
            P[3][1] = rot_vec_var(1)*(t15-t23)*(t17-t36)-rot_vec_var(0)*t19*t31-rot_vec_var(2)*t31*t33;
            P[3][2] = t42;
            P[3][3] = rot_vec_var(2)*(t33*t33)+rot_vec_var(0)*(t43*t43)+rot_vec_var(1)*(t44*t44);

        } else {
            // the equations are badly conditioned so use a small angle approximation
            P[0][0] = 0.0f;
            P[0][1] = 0.0f;
            P[0][2] = 0.0f;
            P[0][3] = 0.0f;
            P[1][0] = 0.0f;
            P[1][1] = 0.25f * rot_vec_var(0);
            P[1][2] = 0.0f;
            P[1][3] = 0.0f;
            P[2][0] = 0.0f;
            P[2][1] = 0.0f;
            P[2][2] = 0.25f * rot_vec_var(1);
            P[2][3] = 0.0f;
            P[3][0] = 0.0f;
            P[3][1] = 0.0f;
            P[3][2] = 0.0f;
            P[3][3] = 0.25f * rot_vec_var(2);
        }

        for (unsigned i = 0; i < 3; i++) {
            for (unsigned j = 0; j < 3; j++) {
                Covariances(i, j) = P[i][j];
            }
        }
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
        //cout << "delAng " << delAng.transpose() << endl;
        //cout << "delVel " << delVel.transpose() << endl;

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
        //correctedDelVel = delVel + 0.5 * (prevDelAng + delAng).cross(prevDelVel + delVel) + 1/6 * (prevDelAng + delAng).cross((prevDelAng + delAng).cross(prevDelVel + delVel)) + 1/12 * (prevDelAng.cross(delVel) + prevDelVel.cross(delAng));
        correctedDelVel = delVel;
        correctedDelAng = delAng - 1/12 * prevDelAng.cross(delAng);
        // save current measurements
        prevDelAng = delAng;
        prevDelVel = delVel;

        // convert the rotation vector to its equivalent quaternion
        Quaterniond deltaQuat;
        deltaQuat = RotVector2Quat(correctedDelAng);
        
        //Vector3d test;
        //get_euler_from_q(test, deltaQuat);
        //cout << "deltaQuat" << test << endl;
        q_.w() = states_[0];
        q_.x() = states_[1];
        q_.y() = states_[2];
        q_.z() = states_[3];
        q_ = q_.normalized();
        get_dcm_from_q(Tbn, q_);

        q_ = QuatMult(q_, deltaQuat);
        q_ = q_.normalized();
        states_.segment(0,4) << q_.w(), q_.x(), q_.y(), q_.z();
        get_dcm_from_q(Tbn, q_);
        //cout << "a" << Tbn * measured_am_ << endl;
        //Vector3d test2;
        //get_euler_from_q(test2, q_);
        //cout << "q_1" << test2 << endl;
        marker.pose.orientation.w = q_.w();
        marker.pose.orientation.x = q_.x();
        marker.pose.orientation.y = q_.y();
        marker.pose.orientation.z = q_.z();
        rviz_pub.publish(marker);

        // transform body delta velocities to delta velocities in the nav frame
        Vector3d delVelNav;
        Vector3d g_vec;
        g_vec << 0, 0, gravity;
        delVelNav = Tbn * correctedDelVel + g_vec * del_t;
        Vector3d prevVel;
        prevVel << states_[4], states_[5], states_[6];
        states_.segment(4,3) << states_[4] + delVelNav[0], states_[5] + delVelNav[1], states_[6] + delVelNav[2];
        //cout << "states_4-6_predict" << states_.segment(4, 3) << endl;

        //cout << "velmea:" << states_.segment(4, 3) << endl;
        // integrate the velocity vector to get the position using trapezoidal
        states_.segment(7,3) << states_[7] + 0.5 * del_t * (prevVel[0] + states_[4]),
                                states_[8] + 0.5 * del_t * (prevVel[1] + states_[5]),
                                states_[9] + 0.5 * del_t * (prevVel[2] + states_[6]);
        //cout << "posmea:" << states_.segment(7, 3) << endl;
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

    /* void Node::PredictCovariance() {
    
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
        cout << Covariances << endl;

        for (int i = 0; i < 24; i++) {
            if(Covariances(i, i) < 0)
                Covariances(i, i) = 0;
        }

    } */
    
    void Node::predictCov(Eigen::Vector3d del_Ang_Cov, Eigen::Vector3d del_Vel_Cov, Eigen::Vector3d del_Ang_bias, Eigen::Vector3d del_Vel_bias, double dt, Eigen::Quaterniond q) {
        double q0 = q.w();
        double q1 = q.x();
        double q2 = q.y();
        double q3 = q.z();

        double dax = del_Ang_Cov[0];
        double day = del_Ang_Cov[1];
        double daz = del_Ang_Cov[2];

        double dvx = del_Vel_Cov[0];
        double dvy = del_Vel_Cov[1];
        double dvz = del_Vel_Cov[2];

        double dax_b = delAng_bias[0];
        double day_b = delAng_bias[1];
        double daz_b = delAng_bias[2];

        double dvx_b = delVel_bias[0];
        double dvy_b = delVel_bias[1];
        double dvz_b = delVel_bias[2]; 

        if (dt > 0.016)
            dt = 0.016;

        double dAngBiasSigma = dt * dt * Node::prediction_param.dAngBiasPnoise;
        double dVelBiasSigma = dt * dt * Node::prediction_param.dVelBiasPnoise;
        float alpha = 1.0f - constrain((dt / Node::prediction_param.acc_bias_learn_tc), 0.0, 1.0);
        _ang_rate_mag_filt = fmaxf(del_Ang_Cov.norm(), alpha * _ang_rate_mag_filt);
	    _accel_mag_filt = fmaxf(del_Vel_Cov.norm(), alpha * _accel_mag_filt);

        if (_ang_rate_mag_filt > dt * Node::prediction_param.acc_bias_learn_gyr_lim
    			|| _accel_mag_filt > dt * Node::prediction_param.acc_bias_learn_acc_lim
    			|| _bad_vert_accel_detected) {
    		// store the bias state variances to be reinstated later
    		if (!_accel_bias_inhibit) {
    			_prev_dvel_bias_var[0] = Covariances(13, 13);
    			_prev_dvel_bias_var[1] = Covariances(14, 14);
    			_prev_dvel_bias_var[2] = Covariances(15, 15);
    		}
    		_accel_bias_inhibit = true;
    	} else {
    		if (_accel_bias_inhibit) {
    			// reinstate the bias state variances
    			Covariances(13, 13) = _prev_dvel_bias_var[0];
    			Covariances(14, 14) = _prev_dvel_bias_var[1];
    			Covariances(15, 15) = _prev_dvel_bias_var[2];
    		} else {
    			// store the bias state variances to be reinstated later
    			_prev_dvel_bias_var[0] = Covariances(13, 13);
    			_prev_dvel_bias_var[1] = Covariances(14, 14);
    			_prev_dvel_bias_var[2] = Covariances(15, 15);
    		}
    		_accel_bias_inhibit = false;
    	}
        
        // Don't continue to grow the earth field variances if they are becoming too large or we are not doing 3-axis fusion as this can make the covariance matrix badly conditioned
    	float mag_I_sig;
    	if (Node::prediction_param.mag_3D && (Covariances(16, 16) + Covariances(17, 17) + Covariances(18, 18)) < 0.1f) {
    		mag_I_sig = dt * constrain(Node::prediction_param.magPnoiseNED, 0.0f, 1.0f);

    	} else {
    		mag_I_sig = 0.0f;
    	}

    	// Don't continue to grow the body field variances if they is becoming too large or we are not doing 3-axis fusion as this can make the covariance matrix badly conditioned
    	float mag_B_sig;
    	if (Node::prediction_param.mag_3D && (Covariances(19, 19) + Covariances(20, 20) + Covariances(21, 21)) < 0.1f) {
    		mag_B_sig = dt * constrain(Node::prediction_param.magPnoiseXYZ, 0.0f, 1.0f);

    	} else {
    		mag_B_sig = 0.0f;
    	}

    	float wind_vel_sig;
    	// Don't continue to grow wind velocity state variances if they are becoming too large or we are not using wind velocity states as this can make the covariance matrix badly conditioned
    	if (Node::prediction_param.wind && (Covariances(22, 22) + Covariances(23, 23)) < 2.0f * (Node::prediction_param.initial_wind_uncertainty * Node::prediction_param.initial_wind_uncertainty)) {
    		wind_vel_sig = dt * constrain(Node::prediction_param.windVelPnoise, 0.0f, 1.0f);

    	} else {
    		wind_vel_sig = 0.0f;
    	}

        Eigen::Matrix<double, 24, 1> process_noise;
    	// Construct the process noise variance diagonal for those states with a stationary process model
    	// These are kinematic states and their error growth is controlled separately by the IMU noise variances
    	for (unsigned i = 0; i <= 9; i++) {
    		process_noise[i] = 0.0f;
    	}
    	// delta angle bias states
    	process_noise[12] = process_noise[11] = process_noise[10] = sq(dAngBiasSigma);
    	// delta_velocity bias states
    	process_noise[15] = process_noise[14] = process_noise[13] = sq(dVelBiasSigma);
    	// earth frame magnetic field states
    	process_noise[18] = process_noise[17] = process_noise[16] = sq(mag_I_sig);
    	// body frame magnetic field states
    	process_noise[21] = process_noise[20] = process_noise[19] = sq(mag_B_sig);
    	// wind velocity states
    	process_noise[23] = process_noise[22] = sq(wind_vel_sig);

    	// assign IMU noise variances
    	// inputs to the system are 3 delta angles and 3 delta velocities
    	float daxVar, dayVar, dazVar;
    	float dvxVar, dvyVar, dvzVar;
    	float gyro_noise = constrain(Node::prediction_param.angRateNoise, 0.0f, 1.0f);
    	daxVar = dayVar = dazVar = sq(dt * gyro_noise);
    	float accel_noise = constrain(Node::prediction_param.accelNoise, 0.0f, 1.0f);
    	if (_bad_vert_accel_detected) {
    		// Increase accelerometer process noise if bad accel data is detected. Measurement errors due to
    		// vibration induced clipping commonly reach an equivalent 0.5g offset.
    		accel_noise = 4.9f; // #define  BADACC_BIAS_PNOISE;
    	}
    	dvxVar = dvyVar = dvzVar = sq(dt * accel_noise);
    	// predict the covariance
        float P[24][24];
        for (int i = 0; i < 24; i++) {
            for (int j = 0; j < 24; j++) {
                P[i][j] = Covariances(i, j);
            }
        }
    	// intermediate calculations
    	float SF[21];
    	SF[0] = dvz - dvz_b;
    	SF[1] = dvy - dvy_b;
    	SF[2] = dvx - dvx_b;
    	SF[3] = 2*q1*SF[2] + 2*q2*SF[1] + 2*q3*SF[0];
    	SF[4] = 2*q0*SF[1] - 2*q1*SF[0] + 2*q3*SF[2];
    	SF[5] = 2*q0*SF[2] + 2*q2*SF[0] - 2*q3*SF[1];
    	SF[6] = day/2 - day_b/2;
    	SF[7] = daz/2 - daz_b/2;
    	SF[8] = dax/2 - dax_b/2;
    	SF[9] = dax_b/2 - dax/2;
    	SF[10] = daz_b/2 - daz/2;
    	SF[11] = day_b/2 - day/2;
    	SF[12] = 2*q1*SF[1];
    	SF[13] = 2*q0*SF[0];
    	SF[14] = q1/2;
    	SF[15] = q2/2;
    	SF[16] = q3/2;
    	SF[17] = sq(q3);
    	SF[18] = sq(q2);
    	SF[19] = sq(q1);
    	SF[20] = sq(q0);

    	float SG[8];
    	SG[0] = q0/2;
    	SG[1] = sq(q3);
    	SG[2] = sq(q2);
    	SG[3] = sq(q1);
    	SG[4] = sq(q0);
    	SG[5] = 2*q2*q3;
    	SG[6] = 2*q1*q3;
    	SG[7] = 2*q1*q2;

    	float SQ[11];
    	SQ[0] = dvzVar*(SG[5] - 2*q0*q1)*(SG[1] - SG[2] - SG[3] + SG[4]) - dvyVar*(SG[5] + 2*q0*q1)*(SG[1] - SG[2] + SG[3] - SG[4]) + dvxVar*(SG[6] - 2*q0*q2)*(SG[7] + 2*q0*q3);
    	SQ[1] = dvzVar*(SG[6] + 2*q0*q2)*(SG[1] - SG[2] - SG[3] + SG[4]) - dvxVar*(SG[6] - 2*q0*q2)*(SG[1] + SG[2] - SG[3] - SG[4]) + dvyVar*(SG[5] + 2*q0*q1)*(SG[7] - 2*q0*q3);
    	SQ[2] = dvzVar*(SG[5] - 2*q0*q1)*(SG[6] + 2*q0*q2) - dvyVar*(SG[7] - 2*q0*q3)*(SG[1] - SG[2] + SG[3] - SG[4]) - dvxVar*(SG[7] + 2*q0*q3)*(SG[1] + SG[2] - SG[3] - SG[4]);
    	SQ[3] = (dayVar*q1*SG[0])/2 - (dazVar*q1*SG[0])/2 - (daxVar*q2*q3)/4;
    	SQ[4] = (dazVar*q2*SG[0])/2 - (daxVar*q2*SG[0])/2 - (dayVar*q1*q3)/4;
    	SQ[5] = (daxVar*q3*SG[0])/2 - (dayVar*q3*SG[0])/2 - (dazVar*q1*q2)/4;
    	SQ[6] = (daxVar*q1*q2)/4 - (dazVar*q3*SG[0])/2 - (dayVar*q1*q2)/4;
    	SQ[7] = (dazVar*q1*q3)/4 - (daxVar*q1*q3)/4 - (dayVar*q2*SG[0])/2;
    	SQ[8] = (dayVar*q2*q3)/4 - (daxVar*q1*SG[0])/2 - (dazVar*q2*q3)/4;
    	SQ[9] = sq(SG[0]);
    	SQ[10] = sq(q1);

    	float SPP[11] = {};
    	SPP[0] = SF[12] + SF[13] - 2*q2*SF[2];
    	SPP[1] = SF[17] - SF[18] - SF[19] + SF[20];
    	SPP[2] = SF[17] - SF[18] + SF[19] - SF[20];
    	SPP[3] = SF[17] + SF[18] - SF[19] - SF[20];
    	SPP[4] = 2*q0*q2 - 2*q1*q3;
    	SPP[5] = 2*q0*q1 - 2*q2*q3;
    	SPP[6] = 2*q0*q3 - 2*q1*q2;
    	SPP[7] = 2*q0*q1 + 2*q2*q3;
    	SPP[8] = 2*q0*q3 + 2*q1*q2;
    	SPP[9] = 2*q0*q2 + 2*q1*q3;
    	SPP[10] = SF[16];

    	// covariance update
        float nextP[24][24];

        // calculate variances and upper diagonal covariances for quaternion, velocity, position and gyro bias states
        nextP[0][0] = P[0][0] + P[1][0]*SF[9] + P[2][0]*SF[11] + P[3][0]*SF[10] + P[10][0]*SF[14] + P[11][0]*SF[15] + P[12][0]*SPP[10] + (daxVar*SQ[10])/4 + SF[9]*(P[0][1] + P[1][1]*SF[9] + P[2][1]*SF[11] + P[3][1]*SF[10] + P[10][1]*SF[14] + P[11][1]*SF[15] + P[12][1]*SPP[10]) + SF[11]*(P[0][2] + P[1][2]*SF[9] + P[2][2]*SF[11] + P[3][2]*SF[10] + P[10][2]*SF[14] + P[11][2]*SF[15] + P[12][2]*SPP[10]) + SF[10]*(P[0][3] + P[1][3]*SF[9] + P[2][3]*SF[11] + P[3][3]*SF[10] + P[10][3]*SF[14] + P[11][3]*SF[15] + P[12][3]*SPP[10]) + SF[14]*(P[0][10] + P[1][10]*SF[9] + P[2][10]*SF[11] + P[3][10]*SF[10] + P[10][10]*SF[14] + P[11][10]*SF[15] + P[12][10]*SPP[10]) + SF[15]*(P[0][11] + P[1][11]*SF[9] + P[2][11]*SF[11] + P[3][11]*SF[10] + P[10][11]*SF[14] + P[11][11]*SF[15] + P[12][11]*SPP[10]) + SPP[10]*(P[0][12] + P[1][12]*SF[9] + P[2][12]*SF[11] + P[3][12]*SF[10] + P[10][12]*SF[14] + P[11][12]*SF[15] + P[12][12]*SPP[10]) + (dayVar*sq(q2))/4 + (dazVar*sq(q3))/4;
        nextP[0][1] = P[0][1] + SQ[8] + P[1][1]*SF[9] + P[2][1]*SF[11] + P[3][1]*SF[10] + P[10][1]*SF[14] + P[11][1]*SF[15] + P[12][1]*SPP[10] + SF[8]*(P[0][0] + P[1][0]*SF[9] + P[2][0]*SF[11] + P[3][0]*SF[10] + P[10][0]*SF[14] + P[11][0]*SF[15] + P[12][0]*SPP[10]) + SF[7]*(P[0][2] + P[1][2]*SF[9] + P[2][2]*SF[11] + P[3][2]*SF[10] + P[10][2]*SF[14] + P[11][2]*SF[15] + P[12][2]*SPP[10]) + SF[11]*(P[0][3] + P[1][3]*SF[9] + P[2][3]*SF[11] + P[3][3]*SF[10] + P[10][3]*SF[14] + P[11][3]*SF[15] + P[12][3]*SPP[10]) - SF[15]*(P[0][12] + P[1][12]*SF[9] + P[2][12]*SF[11] + P[3][12]*SF[10] + P[10][12]*SF[14] + P[11][12]*SF[15] + P[12][12]*SPP[10]) + SPP[10]*(P[0][11] + P[1][11]*SF[9] + P[2][11]*SF[11] + P[3][11]*SF[10] + P[10][11]*SF[14] + P[11][11]*SF[15] + P[12][11]*SPP[10]) - (q0*(P[0][10] + P[1][10]*SF[9] + P[2][10]*SF[11] + P[3][10]*SF[10] + P[10][10]*SF[14] + P[11][10]*SF[15] + P[12][10]*SPP[10]))/2;
        nextP[1][1] = P[1][1] + P[0][1]*SF[8] + P[2][1]*SF[7] + P[3][1]*SF[11] - P[12][1]*SF[15] + P[11][1]*SPP[10] + daxVar*SQ[9] - (P[10][1]*q0)/2 + SF[8]*(P[1][0] + P[0][0]*SF[8] + P[2][0]*SF[7] + P[3][0]*SF[11] - P[12][0]*SF[15] + P[11][0]*SPP[10] - (P[10][0]*q0)/2) + SF[7]*(P[1][2] + P[0][2]*SF[8] + P[2][2]*SF[7] + P[3][2]*SF[11] - P[12][2]*SF[15] + P[11][2]*SPP[10] - (P[10][2]*q0)/2) + SF[11]*(P[1][3] + P[0][3]*SF[8] + P[2][3]*SF[7] + P[3][3]*SF[11] - P[12][3]*SF[15] + P[11][3]*SPP[10] - (P[10][3]*q0)/2) - SF[15]*(P[1][12] + P[0][12]*SF[8] + P[2][12]*SF[7] + P[3][12]*SF[11] - P[12][12]*SF[15] + P[11][12]*SPP[10] - (P[10][12]*q0)/2) + SPP[10]*(P[1][11] + P[0][11]*SF[8] + P[2][11]*SF[7] + P[3][11]*SF[11] - P[12][11]*SF[15] + P[11][11]*SPP[10] - (P[10][11]*q0)/2) + (dayVar*sq(q3))/4 + (dazVar*sq(q2))/4 - (q0*(P[1][10] + P[0][10]*SF[8] + P[2][10]*SF[7] + P[3][10]*SF[11] - P[12][10]*SF[15] + P[11][10]*SPP[10] - (P[10][10]*q0)/2))/2;
        nextP[0][2] = P[0][2] + SQ[7] + P[1][2]*SF[9] + P[2][2]*SF[11] + P[3][2]*SF[10] + P[10][2]*SF[14] + P[11][2]*SF[15] + P[12][2]*SPP[10] + SF[6]*(P[0][0] + P[1][0]*SF[9] + P[2][0]*SF[11] + P[3][0]*SF[10] + P[10][0]*SF[14] + P[11][0]*SF[15] + P[12][0]*SPP[10]) + SF[10]*(P[0][1] + P[1][1]*SF[9] + P[2][1]*SF[11] + P[3][1]*SF[10] + P[10][1]*SF[14] + P[11][1]*SF[15] + P[12][1]*SPP[10]) + SF[8]*(P[0][3] + P[1][3]*SF[9] + P[2][3]*SF[11] + P[3][3]*SF[10] + P[10][3]*SF[14] + P[11][3]*SF[15] + P[12][3]*SPP[10]) + SF[14]*(P[0][12] + P[1][12]*SF[9] + P[2][12]*SF[11] + P[3][12]*SF[10] + P[10][12]*SF[14] + P[11][12]*SF[15] + P[12][12]*SPP[10]) - SPP[10]*(P[0][10] + P[1][10]*SF[9] + P[2][10]*SF[11] + P[3][10]*SF[10] + P[10][10]*SF[14] + P[11][10]*SF[15] + P[12][10]*SPP[10]) - (q0*(P[0][11] + P[1][11]*SF[9] + P[2][11]*SF[11] + P[3][11]*SF[10] + P[10][11]*SF[14] + P[11][11]*SF[15] + P[12][11]*SPP[10]))/2;
        nextP[1][2] = P[1][2] + SQ[5] + P[0][2]*SF[8] + P[2][2]*SF[7] + P[3][2]*SF[11] - P[12][2]*SF[15] + P[11][2]*SPP[10] - (P[10][2]*q0)/2 + SF[6]*(P[1][0] + P[0][0]*SF[8] + P[2][0]*SF[7] + P[3][0]*SF[11] - P[12][0]*SF[15] + P[11][0]*SPP[10] - (P[10][0]*q0)/2) + SF[10]*(P[1][1] + P[0][1]*SF[8] + P[2][1]*SF[7] + P[3][1]*SF[11] - P[12][1]*SF[15] + P[11][1]*SPP[10] - (P[10][1]*q0)/2) + SF[8]*(P[1][3] + P[0][3]*SF[8] + P[2][3]*SF[7] + P[3][3]*SF[11] - P[12][3]*SF[15] + P[11][3]*SPP[10] - (P[10][3]*q0)/2) + SF[14]*(P[1][12] + P[0][12]*SF[8] + P[2][12]*SF[7] + P[3][12]*SF[11] - P[12][12]*SF[15] + P[11][12]*SPP[10] - (P[10][12]*q0)/2) - SPP[10]*(P[1][10] + P[0][10]*SF[8] + P[2][10]*SF[7] + P[3][10]*SF[11] - P[12][10]*SF[15] + P[11][10]*SPP[10] - (P[10][10]*q0)/2) - (q0*(P[1][11] + P[0][11]*SF[8] + P[2][11]*SF[7] + P[3][11]*SF[11] - P[12][11]*SF[15] + P[11][11]*SPP[10] - (P[10][11]*q0)/2))/2;
        nextP[2][2] = P[2][2] + P[0][2]*SF[6] + P[1][2]*SF[10] + P[3][2]*SF[8] + P[12][2]*SF[14] - P[10][2]*SPP[10] + dayVar*SQ[9] + (dazVar*SQ[10])/4 - (P[11][2]*q0)/2 + SF[6]*(P[2][0] + P[0][0]*SF[6] + P[1][0]*SF[10] + P[3][0]*SF[8] + P[12][0]*SF[14] - P[10][0]*SPP[10] - (P[11][0]*q0)/2) + SF[10]*(P[2][1] + P[0][1]*SF[6] + P[1][1]*SF[10] + P[3][1]*SF[8] + P[12][1]*SF[14] - P[10][1]*SPP[10] - (P[11][1]*q0)/2) + SF[8]*(P[2][3] + P[0][3]*SF[6] + P[1][3]*SF[10] + P[3][3]*SF[8] + P[12][3]*SF[14] - P[10][3]*SPP[10] - (P[11][3]*q0)/2) + SF[14]*(P[2][12] + P[0][12]*SF[6] + P[1][12]*SF[10] + P[3][12]*SF[8] + P[12][12]*SF[14] - P[10][12]*SPP[10] - (P[11][12]*q0)/2) - SPP[10]*(P[2][10] + P[0][10]*SF[6] + P[1][10]*SF[10] + P[3][10]*SF[8] + P[12][10]*SF[14] - P[10][10]*SPP[10] - (P[11][10]*q0)/2) + (daxVar*sq(q3))/4 - (q0*(P[2][11] + P[0][11]*SF[6] + P[1][11]*SF[10] + P[3][11]*SF[8] + P[12][11]*SF[14] - P[10][11]*SPP[10] - (P[11][11]*q0)/2))/2;
        nextP[0][3] = P[0][3] + SQ[6] + P[1][3]*SF[9] + P[2][3]*SF[11] + P[3][3]*SF[10] + P[10][3]*SF[14] + P[11][3]*SF[15] + P[12][3]*SPP[10] + SF[7]*(P[0][0] + P[1][0]*SF[9] + P[2][0]*SF[11] + P[3][0]*SF[10] + P[10][0]*SF[14] + P[11][0]*SF[15] + P[12][0]*SPP[10]) + SF[6]*(P[0][1] + P[1][1]*SF[9] + P[2][1]*SF[11] + P[3][1]*SF[10] + P[10][1]*SF[14] + P[11][1]*SF[15] + P[12][1]*SPP[10]) + SF[9]*(P[0][2] + P[1][2]*SF[9] + P[2][2]*SF[11] + P[3][2]*SF[10] + P[10][2]*SF[14] + P[11][2]*SF[15] + P[12][2]*SPP[10]) + SF[15]*(P[0][10] + P[1][10]*SF[9] + P[2][10]*SF[11] + P[3][10]*SF[10] + P[10][10]*SF[14] + P[11][10]*SF[15] + P[12][10]*SPP[10]) - SF[14]*(P[0][11] + P[1][11]*SF[9] + P[2][11]*SF[11] + P[3][11]*SF[10] + P[10][11]*SF[14] + P[11][11]*SF[15] + P[12][11]*SPP[10]) - (q0*(P[0][12] + P[1][12]*SF[9] + P[2][12]*SF[11] + P[3][12]*SF[10] + P[10][12]*SF[14] + P[11][12]*SF[15] + P[12][12]*SPP[10]))/2;
        nextP[1][3] = P[1][3] + SQ[4] + P[0][3]*SF[8] + P[2][3]*SF[7] + P[3][3]*SF[11] - P[12][3]*SF[15] + P[11][3]*SPP[10] - (P[10][3]*q0)/2 + SF[7]*(P[1][0] + P[0][0]*SF[8] + P[2][0]*SF[7] + P[3][0]*SF[11] - P[12][0]*SF[15] + P[11][0]*SPP[10] - (P[10][0]*q0)/2) + SF[6]*(P[1][1] + P[0][1]*SF[8] + P[2][1]*SF[7] + P[3][1]*SF[11] - P[12][1]*SF[15] + P[11][1]*SPP[10] - (P[10][1]*q0)/2) + SF[9]*(P[1][2] + P[0][2]*SF[8] + P[2][2]*SF[7] + P[3][2]*SF[11] - P[12][2]*SF[15] + P[11][2]*SPP[10] - (P[10][2]*q0)/2) + SF[15]*(P[1][10] + P[0][10]*SF[8] + P[2][10]*SF[7] + P[3][10]*SF[11] - P[12][10]*SF[15] + P[11][10]*SPP[10] - (P[10][10]*q0)/2) - SF[14]*(P[1][11] + P[0][11]*SF[8] + P[2][11]*SF[7] + P[3][11]*SF[11] - P[12][11]*SF[15] + P[11][11]*SPP[10] - (P[10][11]*q0)/2) - (q0*(P[1][12] + P[0][12]*SF[8] + P[2][12]*SF[7] + P[3][12]*SF[11] - P[12][12]*SF[15] + P[11][12]*SPP[10] - (P[10][12]*q0)/2))/2;
        nextP[2][3] = P[2][3] + SQ[3] + P[0][3]*SF[6] + P[1][3]*SF[10] + P[3][3]*SF[8] + P[12][3]*SF[14] - P[10][3]*SPP[10] - (P[11][3]*q0)/2 + SF[7]*(P[2][0] + P[0][0]*SF[6] + P[1][0]*SF[10] + P[3][0]*SF[8] + P[12][0]*SF[14] - P[10][0]*SPP[10] - (P[11][0]*q0)/2) + SF[6]*(P[2][1] + P[0][1]*SF[6] + P[1][1]*SF[10] + P[3][1]*SF[8] + P[12][1]*SF[14] - P[10][1]*SPP[10] - (P[11][1]*q0)/2) + SF[9]*(P[2][2] + P[0][2]*SF[6] + P[1][2]*SF[10] + P[3][2]*SF[8] + P[12][2]*SF[14] - P[10][2]*SPP[10] - (P[11][2]*q0)/2) + SF[15]*(P[2][10] + P[0][10]*SF[6] + P[1][10]*SF[10] + P[3][10]*SF[8] + P[12][10]*SF[14] - P[10][10]*SPP[10] - (P[11][10]*q0)/2) - SF[14]*(P[2][11] + P[0][11]*SF[6] + P[1][11]*SF[10] + P[3][11]*SF[8] + P[12][11]*SF[14] - P[10][11]*SPP[10] - (P[11][11]*q0)/2) - (q0*(P[2][12] + P[0][12]*SF[6] + P[1][12]*SF[10] + P[3][12]*SF[8] + P[12][12]*SF[14] - P[10][12]*SPP[10] - (P[11][12]*q0)/2))/2;
        nextP[3][3] = P[3][3] + P[0][3]*SF[7] + P[1][3]*SF[6] + P[2][3]*SF[9] + P[10][3]*SF[15] - P[11][3]*SF[14] + (dayVar*SQ[10])/4 + dazVar*SQ[9] - (P[12][3]*q0)/2 + SF[7]*(P[3][0] + P[0][0]*SF[7] + P[1][0]*SF[6] + P[2][0]*SF[9] + P[10][0]*SF[15] - P[11][0]*SF[14] - (P[12][0]*q0)/2) + SF[6]*(P[3][1] + P[0][1]*SF[7] + P[1][1]*SF[6] + P[2][1]*SF[9] + P[10][1]*SF[15] - P[11][1]*SF[14] - (P[12][1]*q0)/2) + SF[9]*(P[3][2] + P[0][2]*SF[7] + P[1][2]*SF[6] + P[2][2]*SF[9] + P[10][2]*SF[15] - P[11][2]*SF[14] - (P[12][2]*q0)/2) + SF[15]*(P[3][10] + P[0][10]*SF[7] + P[1][10]*SF[6] + P[2][10]*SF[9] + P[10][10]*SF[15] - P[11][10]*SF[14] - (P[12][10]*q0)/2) - SF[14]*(P[3][11] + P[0][11]*SF[7] + P[1][11]*SF[6] + P[2][11]*SF[9] + P[10][11]*SF[15] - P[11][11]*SF[14] - (P[12][11]*q0)/2) + (daxVar*sq(q2))/4 - (q0*(P[3][12] + P[0][12]*SF[7] + P[1][12]*SF[6] + P[2][12]*SF[9] + P[10][12]*SF[15] - P[11][12]*SF[14] - (P[12][12]*q0)/2))/2;
        nextP[0][4] = P[0][4] + P[1][4]*SF[9] + P[2][4]*SF[11] + P[3][4]*SF[10] + P[10][4]*SF[14] + P[11][4]*SF[15] + P[12][4]*SPP[10] + SF[5]*(P[0][0] + P[1][0]*SF[9] + P[2][0]*SF[11] + P[3][0]*SF[10] + P[10][0]*SF[14] + P[11][0]*SF[15] + P[12][0]*SPP[10]) + SF[3]*(P[0][1] + P[1][1]*SF[9] + P[2][1]*SF[11] + P[3][1]*SF[10] + P[10][1]*SF[14] + P[11][1]*SF[15] + P[12][1]*SPP[10]) - SF[4]*(P[0][3] + P[1][3]*SF[9] + P[2][3]*SF[11] + P[3][3]*SF[10] + P[10][3]*SF[14] + P[11][3]*SF[15] + P[12][3]*SPP[10]) + SPP[0]*(P[0][2] + P[1][2]*SF[9] + P[2][2]*SF[11] + P[3][2]*SF[10] + P[10][2]*SF[14] + P[11][2]*SF[15] + P[12][2]*SPP[10]) + SPP[3]*(P[0][13] + P[1][13]*SF[9] + P[2][13]*SF[11] + P[3][13]*SF[10] + P[10][13]*SF[14] + P[11][13]*SF[15] + P[12][13]*SPP[10]) + SPP[6]*(P[0][14] + P[1][14]*SF[9] + P[2][14]*SF[11] + P[3][14]*SF[10] + P[10][14]*SF[14] + P[11][14]*SF[15] + P[12][14]*SPP[10]) - SPP[9]*(P[0][15] + P[1][15]*SF[9] + P[2][15]*SF[11] + P[3][15]*SF[10] + P[10][15]*SF[14] + P[11][15]*SF[15] + P[12][15]*SPP[10]);
        nextP[1][4] = P[1][4] + P[0][4]*SF[8] + P[2][4]*SF[7] + P[3][4]*SF[11] - P[12][4]*SF[15] + P[11][4]*SPP[10] - (P[10][4]*q0)/2 + SF[5]*(P[1][0] + P[0][0]*SF[8] + P[2][0]*SF[7] + P[3][0]*SF[11] - P[12][0]*SF[15] + P[11][0]*SPP[10] - (P[10][0]*q0)/2) + SF[3]*(P[1][1] + P[0][1]*SF[8] + P[2][1]*SF[7] + P[3][1]*SF[11] - P[12][1]*SF[15] + P[11][1]*SPP[10] - (P[10][1]*q0)/2) - SF[4]*(P[1][3] + P[0][3]*SF[8] + P[2][3]*SF[7] + P[3][3]*SF[11] - P[12][3]*SF[15] + P[11][3]*SPP[10] - (P[10][3]*q0)/2) + SPP[0]*(P[1][2] + P[0][2]*SF[8] + P[2][2]*SF[7] + P[3][2]*SF[11] - P[12][2]*SF[15] + P[11][2]*SPP[10] - (P[10][2]*q0)/2) + SPP[3]*(P[1][13] + P[0][13]*SF[8] + P[2][13]*SF[7] + P[3][13]*SF[11] - P[12][13]*SF[15] + P[11][13]*SPP[10] - (P[10][13]*q0)/2) + SPP[6]*(P[1][14] + P[0][14]*SF[8] + P[2][14]*SF[7] + P[3][14]*SF[11] - P[12][14]*SF[15] + P[11][14]*SPP[10] - (P[10][14]*q0)/2) - SPP[9]*(P[1][15] + P[0][15]*SF[8] + P[2][15]*SF[7] + P[3][15]*SF[11] - P[12][15]*SF[15] + P[11][15]*SPP[10] - (P[10][15]*q0)/2);
        nextP[2][4] = P[2][4] + P[0][4]*SF[6] + P[1][4]*SF[10] + P[3][4]*SF[8] + P[12][4]*SF[14] - P[10][4]*SPP[10] - (P[11][4]*q0)/2 + SF[5]*(P[2][0] + P[0][0]*SF[6] + P[1][0]*SF[10] + P[3][0]*SF[8] + P[12][0]*SF[14] - P[10][0]*SPP[10] - (P[11][0]*q0)/2) + SF[3]*(P[2][1] + P[0][1]*SF[6] + P[1][1]*SF[10] + P[3][1]*SF[8] + P[12][1]*SF[14] - P[10][1]*SPP[10] - (P[11][1]*q0)/2) - SF[4]*(P[2][3] + P[0][3]*SF[6] + P[1][3]*SF[10] + P[3][3]*SF[8] + P[12][3]*SF[14] - P[10][3]*SPP[10] - (P[11][3]*q0)/2) + SPP[0]*(P[2][2] + P[0][2]*SF[6] + P[1][2]*SF[10] + P[3][2]*SF[8] + P[12][2]*SF[14] - P[10][2]*SPP[10] - (P[11][2]*q0)/2) + SPP[3]*(P[2][13] + P[0][13]*SF[6] + P[1][13]*SF[10] + P[3][13]*SF[8] + P[12][13]*SF[14] - P[10][13]*SPP[10] - (P[11][13]*q0)/2) + SPP[6]*(P[2][14] + P[0][14]*SF[6] + P[1][14]*SF[10] + P[3][14]*SF[8] + P[12][14]*SF[14] - P[10][14]*SPP[10] - (P[11][14]*q0)/2) - SPP[9]*(P[2][15] + P[0][15]*SF[6] + P[1][15]*SF[10] + P[3][15]*SF[8] + P[12][15]*SF[14] - P[10][15]*SPP[10] - (P[11][15]*q0)/2);
        nextP[3][4] = P[3][4] + P[0][4]*SF[7] + P[1][4]*SF[6] + P[2][4]*SF[9] + P[10][4]*SF[15] - P[11][4]*SF[14] - (P[12][4]*q0)/2 + SF[5]*(P[3][0] + P[0][0]*SF[7] + P[1][0]*SF[6] + P[2][0]*SF[9] + P[10][0]*SF[15] - P[11][0]*SF[14] - (P[12][0]*q0)/2) + SF[3]*(P[3][1] + P[0][1]*SF[7] + P[1][1]*SF[6] + P[2][1]*SF[9] + P[10][1]*SF[15] - P[11][1]*SF[14] - (P[12][1]*q0)/2) - SF[4]*(P[3][3] + P[0][3]*SF[7] + P[1][3]*SF[6] + P[2][3]*SF[9] + P[10][3]*SF[15] - P[11][3]*SF[14] - (P[12][3]*q0)/2) + SPP[0]*(P[3][2] + P[0][2]*SF[7] + P[1][2]*SF[6] + P[2][2]*SF[9] + P[10][2]*SF[15] - P[11][2]*SF[14] - (P[12][2]*q0)/2) + SPP[3]*(P[3][13] + P[0][13]*SF[7] + P[1][13]*SF[6] + P[2][13]*SF[9] + P[10][13]*SF[15] - P[11][13]*SF[14] - (P[12][13]*q0)/2) + SPP[6]*(P[3][14] + P[0][14]*SF[7] + P[1][14]*SF[6] + P[2][14]*SF[9] + P[10][14]*SF[15] - P[11][14]*SF[14] - (P[12][14]*q0)/2) - SPP[9]*(P[3][15] + P[0][15]*SF[7] + P[1][15]*SF[6] + P[2][15]*SF[9] + P[10][15]*SF[15] - P[11][15]*SF[14] - (P[12][15]*q0)/2);
        nextP[4][4] = P[4][4] + P[0][4]*SF[5] + P[1][4]*SF[3] - P[3][4]*SF[4] + P[2][4]*SPP[0] + P[13][4]*SPP[3] + P[14][4]*SPP[6] - P[15][4]*SPP[9] + dvyVar*sq(SG[7] - 2*q0*q3) + dvzVar*sq(SG[6] + 2*q0*q2) + SF[5]*(P[4][0] + P[0][0]*SF[5] + P[1][0]*SF[3] - P[3][0]*SF[4] + P[2][0]*SPP[0] + P[13][0]*SPP[3] + P[14][0]*SPP[6] - P[15][0]*SPP[9]) + SF[3]*(P[4][1] + P[0][1]*SF[5] + P[1][1]*SF[3] - P[3][1]*SF[4] + P[2][1]*SPP[0] + P[13][1]*SPP[3] + P[14][1]*SPP[6] - P[15][1]*SPP[9]) - SF[4]*(P[4][3] + P[0][3]*SF[5] + P[1][3]*SF[3] - P[3][3]*SF[4] + P[2][3]*SPP[0] + P[13][3]*SPP[3] + P[14][3]*SPP[6] - P[15][3]*SPP[9]) + SPP[0]*(P[4][2] + P[0][2]*SF[5] + P[1][2]*SF[3] - P[3][2]*SF[4] + P[2][2]*SPP[0] + P[13][2]*SPP[3] + P[14][2]*SPP[6] - P[15][2]*SPP[9]) + SPP[3]*(P[4][13] + P[0][13]*SF[5] + P[1][13]*SF[3] - P[3][13]*SF[4] + P[2][13]*SPP[0] + P[13][13]*SPP[3] + P[14][13]*SPP[6] - P[15][13]*SPP[9]) + SPP[6]*(P[4][14] + P[0][14]*SF[5] + P[1][14]*SF[3] - P[3][14]*SF[4] + P[2][14]*SPP[0] + P[13][14]*SPP[3] + P[14][14]*SPP[6] - P[15][14]*SPP[9]) - SPP[9]*(P[4][15] + P[0][15]*SF[5] + P[1][15]*SF[3] - P[3][15]*SF[4] + P[2][15]*SPP[0] + P[13][15]*SPP[3] + P[14][15]*SPP[6] - P[15][15]*SPP[9]) + dvxVar*sq(SG[1] + SG[2] - SG[3] - SG[4]);
        nextP[0][5] = P[0][5] + P[1][5]*SF[9] + P[2][5]*SF[11] + P[3][5]*SF[10] + P[10][5]*SF[14] + P[11][5]*SF[15] + P[12][5]*SPP[10] + SF[4]*(P[0][0] + P[1][0]*SF[9] + P[2][0]*SF[11] + P[3][0]*SF[10] + P[10][0]*SF[14] + P[11][0]*SF[15] + P[12][0]*SPP[10]) + SF[3]*(P[0][2] + P[1][2]*SF[9] + P[2][2]*SF[11] + P[3][2]*SF[10] + P[10][2]*SF[14] + P[11][2]*SF[15] + P[12][2]*SPP[10]) + SF[5]*(P[0][3] + P[1][3]*SF[9] + P[2][3]*SF[11] + P[3][3]*SF[10] + P[10][3]*SF[14] + P[11][3]*SF[15] + P[12][3]*SPP[10]) - SPP[0]*(P[0][1] + P[1][1]*SF[9] + P[2][1]*SF[11] + P[3][1]*SF[10] + P[10][1]*SF[14] + P[11][1]*SF[15] + P[12][1]*SPP[10]) - SPP[8]*(P[0][13] + P[1][13]*SF[9] + P[2][13]*SF[11] + P[3][13]*SF[10] + P[10][13]*SF[14] + P[11][13]*SF[15] + P[12][13]*SPP[10]) + SPP[2]*(P[0][14] + P[1][14]*SF[9] + P[2][14]*SF[11] + P[3][14]*SF[10] + P[10][14]*SF[14] + P[11][14]*SF[15] + P[12][14]*SPP[10]) + SPP[5]*(P[0][15] + P[1][15]*SF[9] + P[2][15]*SF[11] + P[3][15]*SF[10] + P[10][15]*SF[14] + P[11][15]*SF[15] + P[12][15]*SPP[10]);
        nextP[1][5] = P[1][5] + P[0][5]*SF[8] + P[2][5]*SF[7] + P[3][5]*SF[11] - P[12][5]*SF[15] + P[11][5]*SPP[10] - (P[10][5]*q0)/2 + SF[4]*(P[1][0] + P[0][0]*SF[8] + P[2][0]*SF[7] + P[3][0]*SF[11] - P[12][0]*SF[15] + P[11][0]*SPP[10] - (P[10][0]*q0)/2) + SF[3]*(P[1][2] + P[0][2]*SF[8] + P[2][2]*SF[7] + P[3][2]*SF[11] - P[12][2]*SF[15] + P[11][2]*SPP[10] - (P[10][2]*q0)/2) + SF[5]*(P[1][3] + P[0][3]*SF[8] + P[2][3]*SF[7] + P[3][3]*SF[11] - P[12][3]*SF[15] + P[11][3]*SPP[10] - (P[10][3]*q0)/2) - SPP[0]*(P[1][1] + P[0][1]*SF[8] + P[2][1]*SF[7] + P[3][1]*SF[11] - P[12][1]*SF[15] + P[11][1]*SPP[10] - (P[10][1]*q0)/2) - SPP[8]*(P[1][13] + P[0][13]*SF[8] + P[2][13]*SF[7] + P[3][13]*SF[11] - P[12][13]*SF[15] + P[11][13]*SPP[10] - (P[10][13]*q0)/2) + SPP[2]*(P[1][14] + P[0][14]*SF[8] + P[2][14]*SF[7] + P[3][14]*SF[11] - P[12][14]*SF[15] + P[11][14]*SPP[10] - (P[10][14]*q0)/2) + SPP[5]*(P[1][15] + P[0][15]*SF[8] + P[2][15]*SF[7] + P[3][15]*SF[11] - P[12][15]*SF[15] + P[11][15]*SPP[10] - (P[10][15]*q0)/2);
        nextP[2][5] = P[2][5] + P[0][5]*SF[6] + P[1][5]*SF[10] + P[3][5]*SF[8] + P[12][5]*SF[14] - P[10][5]*SPP[10] - (P[11][5]*q0)/2 + SF[4]*(P[2][0] + P[0][0]*SF[6] + P[1][0]*SF[10] + P[3][0]*SF[8] + P[12][0]*SF[14] - P[10][0]*SPP[10] - (P[11][0]*q0)/2) + SF[3]*(P[2][2] + P[0][2]*SF[6] + P[1][2]*SF[10] + P[3][2]*SF[8] + P[12][2]*SF[14] - P[10][2]*SPP[10] - (P[11][2]*q0)/2) + SF[5]*(P[2][3] + P[0][3]*SF[6] + P[1][3]*SF[10] + P[3][3]*SF[8] + P[12][3]*SF[14] - P[10][3]*SPP[10] - (P[11][3]*q0)/2) - SPP[0]*(P[2][1] + P[0][1]*SF[6] + P[1][1]*SF[10] + P[3][1]*SF[8] + P[12][1]*SF[14] - P[10][1]*SPP[10] - (P[11][1]*q0)/2) - SPP[8]*(P[2][13] + P[0][13]*SF[6] + P[1][13]*SF[10] + P[3][13]*SF[8] + P[12][13]*SF[14] - P[10][13]*SPP[10] - (P[11][13]*q0)/2) + SPP[2]*(P[2][14] + P[0][14]*SF[6] + P[1][14]*SF[10] + P[3][14]*SF[8] + P[12][14]*SF[14] - P[10][14]*SPP[10] - (P[11][14]*q0)/2) + SPP[5]*(P[2][15] + P[0][15]*SF[6] + P[1][15]*SF[10] + P[3][15]*SF[8] + P[12][15]*SF[14] - P[10][15]*SPP[10] - (P[11][15]*q0)/2);
        nextP[3][5] = P[3][5] + P[0][5]*SF[7] + P[1][5]*SF[6] + P[2][5]*SF[9] + P[10][5]*SF[15] - P[11][5]*SF[14] - (P[12][5]*q0)/2 + SF[4]*(P[3][0] + P[0][0]*SF[7] + P[1][0]*SF[6] + P[2][0]*SF[9] + P[10][0]*SF[15] - P[11][0]*SF[14] - (P[12][0]*q0)/2) + SF[3]*(P[3][2] + P[0][2]*SF[7] + P[1][2]*SF[6] + P[2][2]*SF[9] + P[10][2]*SF[15] - P[11][2]*SF[14] - (P[12][2]*q0)/2) + SF[5]*(P[3][3] + P[0][3]*SF[7] + P[1][3]*SF[6] + P[2][3]*SF[9] + P[10][3]*SF[15] - P[11][3]*SF[14] - (P[12][3]*q0)/2) - SPP[0]*(P[3][1] + P[0][1]*SF[7] + P[1][1]*SF[6] + P[2][1]*SF[9] + P[10][1]*SF[15] - P[11][1]*SF[14] - (P[12][1]*q0)/2) - SPP[8]*(P[3][13] + P[0][13]*SF[7] + P[1][13]*SF[6] + P[2][13]*SF[9] + P[10][13]*SF[15] - P[11][13]*SF[14] - (P[12][13]*q0)/2) + SPP[2]*(P[3][14] + P[0][14]*SF[7] + P[1][14]*SF[6] + P[2][14]*SF[9] + P[10][14]*SF[15] - P[11][14]*SF[14] - (P[12][14]*q0)/2) + SPP[5]*(P[3][15] + P[0][15]*SF[7] + P[1][15]*SF[6] + P[2][15]*SF[9] + P[10][15]*SF[15] - P[11][15]*SF[14] - (P[12][15]*q0)/2);
        nextP[4][5] = P[4][5] + SQ[2] + P[0][5]*SF[5] + P[1][5]*SF[3] - P[3][5]*SF[4] + P[2][5]*SPP[0] + P[13][5]*SPP[3] + P[14][5]*SPP[6] - P[15][5]*SPP[9] + SF[4]*(P[4][0] + P[0][0]*SF[5] + P[1][0]*SF[3] - P[3][0]*SF[4] + P[2][0]*SPP[0] + P[13][0]*SPP[3] + P[14][0]*SPP[6] - P[15][0]*SPP[9]) + SF[3]*(P[4][2] + P[0][2]*SF[5] + P[1][2]*SF[3] - P[3][2]*SF[4] + P[2][2]*SPP[0] + P[13][2]*SPP[3] + P[14][2]*SPP[6] - P[15][2]*SPP[9]) + SF[5]*(P[4][3] + P[0][3]*SF[5] + P[1][3]*SF[3] - P[3][3]*SF[4] + P[2][3]*SPP[0] + P[13][3]*SPP[3] + P[14][3]*SPP[6] - P[15][3]*SPP[9]) - SPP[0]*(P[4][1] + P[0][1]*SF[5] + P[1][1]*SF[3] - P[3][1]*SF[4] + P[2][1]*SPP[0] + P[13][1]*SPP[3] + P[14][1]*SPP[6] - P[15][1]*SPP[9]) - SPP[8]*(P[4][13] + P[0][13]*SF[5] + P[1][13]*SF[3] - P[3][13]*SF[4] + P[2][13]*SPP[0] + P[13][13]*SPP[3] + P[14][13]*SPP[6] - P[15][13]*SPP[9]) + SPP[2]*(P[4][14] + P[0][14]*SF[5] + P[1][14]*SF[3] - P[3][14]*SF[4] + P[2][14]*SPP[0] + P[13][14]*SPP[3] + P[14][14]*SPP[6] - P[15][14]*SPP[9]) + SPP[5]*(P[4][15] + P[0][15]*SF[5] + P[1][15]*SF[3] - P[3][15]*SF[4] + P[2][15]*SPP[0] + P[13][15]*SPP[3] + P[14][15]*SPP[6] - P[15][15]*SPP[9]);
        nextP[5][5] = P[5][5] + P[0][5]*SF[4] + P[2][5]*SF[3] + P[3][5]*SF[5] - P[1][5]*SPP[0] - P[13][5]*SPP[8] + P[14][5]*SPP[2] + P[15][5]*SPP[5] + dvxVar*sq(SG[7] + 2*q0*q3) + dvzVar*sq(SG[5] - 2*q0*q1) + SF[4]*(P[5][0] + P[0][0]*SF[4] + P[2][0]*SF[3] + P[3][0]*SF[5] - P[1][0]*SPP[0] - P[13][0]*SPP[8] + P[14][0]*SPP[2] + P[15][0]*SPP[5]) + SF[3]*(P[5][2] + P[0][2]*SF[4] + P[2][2]*SF[3] + P[3][2]*SF[5] - P[1][2]*SPP[0] - P[13][2]*SPP[8] + P[14][2]*SPP[2] + P[15][2]*SPP[5]) + SF[5]*(P[5][3] + P[0][3]*SF[4] + P[2][3]*SF[3] + P[3][3]*SF[5] - P[1][3]*SPP[0] - P[13][3]*SPP[8] + P[14][3]*SPP[2] + P[15][3]*SPP[5]) - SPP[0]*(P[5][1] + P[0][1]*SF[4] + P[2][1]*SF[3] + P[3][1]*SF[5] - P[1][1]*SPP[0] - P[13][1]*SPP[8] + P[14][1]*SPP[2] + P[15][1]*SPP[5]) - SPP[8]*(P[5][13] + P[0][13]*SF[4] + P[2][13]*SF[3] + P[3][13]*SF[5] - P[1][13]*SPP[0] - P[13][13]*SPP[8] + P[14][13]*SPP[2] + P[15][13]*SPP[5]) + SPP[2]*(P[5][14] + P[0][14]*SF[4] + P[2][14]*SF[3] + P[3][14]*SF[5] - P[1][14]*SPP[0] - P[13][14]*SPP[8] + P[14][14]*SPP[2] + P[15][14]*SPP[5]) + SPP[5]*(P[5][15] + P[0][15]*SF[4] + P[2][15]*SF[3] + P[3][15]*SF[5] - P[1][15]*SPP[0] - P[13][15]*SPP[8] + P[14][15]*SPP[2] + P[15][15]*SPP[5]) + dvyVar*sq(SG[1] - SG[2] + SG[3] - SG[4]);
        nextP[0][6] = P[0][6] + P[1][6]*SF[9] + P[2][6]*SF[11] + P[3][6]*SF[10] + P[10][6]*SF[14] + P[11][6]*SF[15] + P[12][6]*SPP[10] + SF[4]*(P[0][1] + P[1][1]*SF[9] + P[2][1]*SF[11] + P[3][1]*SF[10] + P[10][1]*SF[14] + P[11][1]*SF[15] + P[12][1]*SPP[10]) - SF[5]*(P[0][2] + P[1][2]*SF[9] + P[2][2]*SF[11] + P[3][2]*SF[10] + P[10][2]*SF[14] + P[11][2]*SF[15] + P[12][2]*SPP[10]) + SF[3]*(P[0][3] + P[1][3]*SF[9] + P[2][3]*SF[11] + P[3][3]*SF[10] + P[10][3]*SF[14] + P[11][3]*SF[15] + P[12][3]*SPP[10]) + SPP[0]*(P[0][0] + P[1][0]*SF[9] + P[2][0]*SF[11] + P[3][0]*SF[10] + P[10][0]*SF[14] + P[11][0]*SF[15] + P[12][0]*SPP[10]) + SPP[4]*(P[0][13] + P[1][13]*SF[9] + P[2][13]*SF[11] + P[3][13]*SF[10] + P[10][13]*SF[14] + P[11][13]*SF[15] + P[12][13]*SPP[10]) - SPP[7]*(P[0][14] + P[1][14]*SF[9] + P[2][14]*SF[11] + P[3][14]*SF[10] + P[10][14]*SF[14] + P[11][14]*SF[15] + P[12][14]*SPP[10]) - SPP[1]*(P[0][15] + P[1][15]*SF[9] + P[2][15]*SF[11] + P[3][15]*SF[10] + P[10][15]*SF[14] + P[11][15]*SF[15] + P[12][15]*SPP[10]);
        nextP[1][6] = P[1][6] + P[0][6]*SF[8] + P[2][6]*SF[7] + P[3][6]*SF[11] - P[12][6]*SF[15] + P[11][6]*SPP[10] - (P[10][6]*q0)/2 + SF[4]*(P[1][1] + P[0][1]*SF[8] + P[2][1]*SF[7] + P[3][1]*SF[11] - P[12][1]*SF[15] + P[11][1]*SPP[10] - (P[10][1]*q0)/2) - SF[5]*(P[1][2] + P[0][2]*SF[8] + P[2][2]*SF[7] + P[3][2]*SF[11] - P[12][2]*SF[15] + P[11][2]*SPP[10] - (P[10][2]*q0)/2) + SF[3]*(P[1][3] + P[0][3]*SF[8] + P[2][3]*SF[7] + P[3][3]*SF[11] - P[12][3]*SF[15] + P[11][3]*SPP[10] - (P[10][3]*q0)/2) + SPP[0]*(P[1][0] + P[0][0]*SF[8] + P[2][0]*SF[7] + P[3][0]*SF[11] - P[12][0]*SF[15] + P[11][0]*SPP[10] - (P[10][0]*q0)/2) + SPP[4]*(P[1][13] + P[0][13]*SF[8] + P[2][13]*SF[7] + P[3][13]*SF[11] - P[12][13]*SF[15] + P[11][13]*SPP[10] - (P[10][13]*q0)/2) - SPP[7]*(P[1][14] + P[0][14]*SF[8] + P[2][14]*SF[7] + P[3][14]*SF[11] - P[12][14]*SF[15] + P[11][14]*SPP[10] - (P[10][14]*q0)/2) - SPP[1]*(P[1][15] + P[0][15]*SF[8] + P[2][15]*SF[7] + P[3][15]*SF[11] - P[12][15]*SF[15] + P[11][15]*SPP[10] - (P[10][15]*q0)/2);
        nextP[2][6] = P[2][6] + P[0][6]*SF[6] + P[1][6]*SF[10] + P[3][6]*SF[8] + P[12][6]*SF[14] - P[10][6]*SPP[10] - (P[11][6]*q0)/2 + SF[4]*(P[2][1] + P[0][1]*SF[6] + P[1][1]*SF[10] + P[3][1]*SF[8] + P[12][1]*SF[14] - P[10][1]*SPP[10] - (P[11][1]*q0)/2) - SF[5]*(P[2][2] + P[0][2]*SF[6] + P[1][2]*SF[10] + P[3][2]*SF[8] + P[12][2]*SF[14] - P[10][2]*SPP[10] - (P[11][2]*q0)/2) + SF[3]*(P[2][3] + P[0][3]*SF[6] + P[1][3]*SF[10] + P[3][3]*SF[8] + P[12][3]*SF[14] - P[10][3]*SPP[10] - (P[11][3]*q0)/2) + SPP[0]*(P[2][0] + P[0][0]*SF[6] + P[1][0]*SF[10] + P[3][0]*SF[8] + P[12][0]*SF[14] - P[10][0]*SPP[10] - (P[11][0]*q0)/2) + SPP[4]*(P[2][13] + P[0][13]*SF[6] + P[1][13]*SF[10] + P[3][13]*SF[8] + P[12][13]*SF[14] - P[10][13]*SPP[10] - (P[11][13]*q0)/2) - SPP[7]*(P[2][14] + P[0][14]*SF[6] + P[1][14]*SF[10] + P[3][14]*SF[8] + P[12][14]*SF[14] - P[10][14]*SPP[10] - (P[11][14]*q0)/2) - SPP[1]*(P[2][15] + P[0][15]*SF[6] + P[1][15]*SF[10] + P[3][15]*SF[8] + P[12][15]*SF[14] - P[10][15]*SPP[10] - (P[11][15]*q0)/2);
        nextP[3][6] = P[3][6] + P[0][6]*SF[7] + P[1][6]*SF[6] + P[2][6]*SF[9] + P[10][6]*SF[15] - P[11][6]*SF[14] - (P[12][6]*q0)/2 + SF[4]*(P[3][1] + P[0][1]*SF[7] + P[1][1]*SF[6] + P[2][1]*SF[9] + P[10][1]*SF[15] - P[11][1]*SF[14] - (P[12][1]*q0)/2) - SF[5]*(P[3][2] + P[0][2]*SF[7] + P[1][2]*SF[6] + P[2][2]*SF[9] + P[10][2]*SF[15] - P[11][2]*SF[14] - (P[12][2]*q0)/2) + SF[3]*(P[3][3] + P[0][3]*SF[7] + P[1][3]*SF[6] + P[2][3]*SF[9] + P[10][3]*SF[15] - P[11][3]*SF[14] - (P[12][3]*q0)/2) + SPP[0]*(P[3][0] + P[0][0]*SF[7] + P[1][0]*SF[6] + P[2][0]*SF[9] + P[10][0]*SF[15] - P[11][0]*SF[14] - (P[12][0]*q0)/2) + SPP[4]*(P[3][13] + P[0][13]*SF[7] + P[1][13]*SF[6] + P[2][13]*SF[9] + P[10][13]*SF[15] - P[11][13]*SF[14] - (P[12][13]*q0)/2) - SPP[7]*(P[3][14] + P[0][14]*SF[7] + P[1][14]*SF[6] + P[2][14]*SF[9] + P[10][14]*SF[15] - P[11][14]*SF[14] - (P[12][14]*q0)/2) - SPP[1]*(P[3][15] + P[0][15]*SF[7] + P[1][15]*SF[6] + P[2][15]*SF[9] + P[10][15]*SF[15] - P[11][15]*SF[14] - (P[12][15]*q0)/2);
        nextP[4][6] = P[4][6] + SQ[1] + P[0][6]*SF[5] + P[1][6]*SF[3] - P[3][6]*SF[4] + P[2][6]*SPP[0] + P[13][6]*SPP[3] + P[14][6]*SPP[6] - P[15][6]*SPP[9] + SF[4]*(P[4][1] + P[0][1]*SF[5] + P[1][1]*SF[3] - P[3][1]*SF[4] + P[2][1]*SPP[0] + P[13][1]*SPP[3] + P[14][1]*SPP[6] - P[15][1]*SPP[9]) - SF[5]*(P[4][2] + P[0][2]*SF[5] + P[1][2]*SF[3] - P[3][2]*SF[4] + P[2][2]*SPP[0] + P[13][2]*SPP[3] + P[14][2]*SPP[6] - P[15][2]*SPP[9]) + SF[3]*(P[4][3] + P[0][3]*SF[5] + P[1][3]*SF[3] - P[3][3]*SF[4] + P[2][3]*SPP[0] + P[13][3]*SPP[3] + P[14][3]*SPP[6] - P[15][3]*SPP[9]) + SPP[0]*(P[4][0] + P[0][0]*SF[5] + P[1][0]*SF[3] - P[3][0]*SF[4] + P[2][0]*SPP[0] + P[13][0]*SPP[3] + P[14][0]*SPP[6] - P[15][0]*SPP[9]) + SPP[4]*(P[4][13] + P[0][13]*SF[5] + P[1][13]*SF[3] - P[3][13]*SF[4] + P[2][13]*SPP[0] + P[13][13]*SPP[3] + P[14][13]*SPP[6] - P[15][13]*SPP[9]) - SPP[7]*(P[4][14] + P[0][14]*SF[5] + P[1][14]*SF[3] - P[3][14]*SF[4] + P[2][14]*SPP[0] + P[13][14]*SPP[3] + P[14][14]*SPP[6] - P[15][14]*SPP[9]) - SPP[1]*(P[4][15] + P[0][15]*SF[5] + P[1][15]*SF[3] - P[3][15]*SF[4] + P[2][15]*SPP[0] + P[13][15]*SPP[3] + P[14][15]*SPP[6] - P[15][15]*SPP[9]);
        nextP[5][6] = P[5][6] + SQ[0] + P[0][6]*SF[4] + P[2][6]*SF[3] + P[3][6]*SF[5] - P[1][6]*SPP[0] - P[13][6]*SPP[8] + P[14][6]*SPP[2] + P[15][6]*SPP[5] + SF[4]*(P[5][1] + P[0][1]*SF[4] + P[2][1]*SF[3] + P[3][1]*SF[5] - P[1][1]*SPP[0] - P[13][1]*SPP[8] + P[14][1]*SPP[2] + P[15][1]*SPP[5]) - SF[5]*(P[5][2] + P[0][2]*SF[4] + P[2][2]*SF[3] + P[3][2]*SF[5] - P[1][2]*SPP[0] - P[13][2]*SPP[8] + P[14][2]*SPP[2] + P[15][2]*SPP[5]) + SF[3]*(P[5][3] + P[0][3]*SF[4] + P[2][3]*SF[3] + P[3][3]*SF[5] - P[1][3]*SPP[0] - P[13][3]*SPP[8] + P[14][3]*SPP[2] + P[15][3]*SPP[5]) + SPP[0]*(P[5][0] + P[0][0]*SF[4] + P[2][0]*SF[3] + P[3][0]*SF[5] - P[1][0]*SPP[0] - P[13][0]*SPP[8] + P[14][0]*SPP[2] + P[15][0]*SPP[5]) + SPP[4]*(P[5][13] + P[0][13]*SF[4] + P[2][13]*SF[3] + P[3][13]*SF[5] - P[1][13]*SPP[0] - P[13][13]*SPP[8] + P[14][13]*SPP[2] + P[15][13]*SPP[5]) - SPP[7]*(P[5][14] + P[0][14]*SF[4] + P[2][14]*SF[3] + P[3][14]*SF[5] - P[1][14]*SPP[0] - P[13][14]*SPP[8] + P[14][14]*SPP[2] + P[15][14]*SPP[5]) - SPP[1]*(P[5][15] + P[0][15]*SF[4] + P[2][15]*SF[3] + P[3][15]*SF[5] - P[1][15]*SPP[0] - P[13][15]*SPP[8] + P[14][15]*SPP[2] + P[15][15]*SPP[5]);
        nextP[6][6] = P[6][6] + P[1][6]*SF[4] - P[2][6]*SF[5] + P[3][6]*SF[3] + P[0][6]*SPP[0] + P[13][6]*SPP[4] - P[14][6]*SPP[7] - P[15][6]*SPP[1] + dvxVar*sq(SG[6] - 2*q0*q2) + dvyVar*sq(SG[5] + 2*q0*q1) + SF[4]*(P[6][1] + P[1][1]*SF[4] - P[2][1]*SF[5] + P[3][1]*SF[3] + P[0][1]*SPP[0] + P[13][1]*SPP[4] - P[14][1]*SPP[7] - P[15][1]*SPP[1]) - SF[5]*(P[6][2] + P[1][2]*SF[4] - P[2][2]*SF[5] + P[3][2]*SF[3] + P[0][2]*SPP[0] + P[13][2]*SPP[4] - P[14][2]*SPP[7] - P[15][2]*SPP[1]) + SF[3]*(P[6][3] + P[1][3]*SF[4] - P[2][3]*SF[5] + P[3][3]*SF[3] + P[0][3]*SPP[0] + P[13][3]*SPP[4] - P[14][3]*SPP[7] - P[15][3]*SPP[1]) + SPP[0]*(P[6][0] + P[1][0]*SF[4] - P[2][0]*SF[5] + P[3][0]*SF[3] + P[0][0]*SPP[0] + P[13][0]*SPP[4] - P[14][0]*SPP[7] - P[15][0]*SPP[1]) + SPP[4]*(P[6][13] + P[1][13]*SF[4] - P[2][13]*SF[5] + P[3][13]*SF[3] + P[0][13]*SPP[0] + P[13][13]*SPP[4] - P[14][13]*SPP[7] - P[15][13]*SPP[1]) - SPP[7]*(P[6][14] + P[1][14]*SF[4] - P[2][14]*SF[5] + P[3][14]*SF[3] + P[0][14]*SPP[0] + P[13][14]*SPP[4] - P[14][14]*SPP[7] - P[15][14]*SPP[1]) - SPP[1]*(P[6][15] + P[1][15]*SF[4] - P[2][15]*SF[5] + P[3][15]*SF[3] + P[0][15]*SPP[0] + P[13][15]*SPP[4] - P[14][15]*SPP[7] - P[15][15]*SPP[1]) + dvzVar*sq(SG[1] - SG[2] - SG[3] + SG[4]);
        nextP[0][7] = P[0][7] + P[1][7]*SF[9] + P[2][7]*SF[11] + P[3][7]*SF[10] + P[10][7]*SF[14] + P[11][7]*SF[15] + P[12][7]*SPP[10] + dt*(P[0][4] + P[1][4]*SF[9] + P[2][4]*SF[11] + P[3][4]*SF[10] + P[10][4]*SF[14] + P[11][4]*SF[15] + P[12][4]*SPP[10]);
        nextP[1][7] = P[1][7] + P[0][7]*SF[8] + P[2][7]*SF[7] + P[3][7]*SF[11] - P[12][7]*SF[15] + P[11][7]*SPP[10] - (P[10][7]*q0)/2 + dt*(P[1][4] + P[0][4]*SF[8] + P[2][4]*SF[7] + P[3][4]*SF[11] - P[12][4]*SF[15] + P[11][4]*SPP[10] - (P[10][4]*q0)/2);
        nextP[2][7] = P[2][7] + P[0][7]*SF[6] + P[1][7]*SF[10] + P[3][7]*SF[8] + P[12][7]*SF[14] - P[10][7]*SPP[10] - (P[11][7]*q0)/2 + dt*(P[2][4] + P[0][4]*SF[6] + P[1][4]*SF[10] + P[3][4]*SF[8] + P[12][4]*SF[14] - P[10][4]*SPP[10] - (P[11][4]*q0)/2);
        nextP[3][7] = P[3][7] + P[0][7]*SF[7] + P[1][7]*SF[6] + P[2][7]*SF[9] + P[10][7]*SF[15] - P[11][7]*SF[14] - (P[12][7]*q0)/2 + dt*(P[3][4] + P[0][4]*SF[7] + P[1][4]*SF[6] + P[2][4]*SF[9] + P[10][4]*SF[15] - P[11][4]*SF[14] - (P[12][4]*q0)/2);
        nextP[4][7] = P[4][7] + P[0][7]*SF[5] + P[1][7]*SF[3] - P[3][7]*SF[4] + P[2][7]*SPP[0] + P[13][7]*SPP[3] + P[14][7]*SPP[6] - P[15][7]*SPP[9] + dt*(P[4][4] + P[0][4]*SF[5] + P[1][4]*SF[3] - P[3][4]*SF[4] + P[2][4]*SPP[0] + P[13][4]*SPP[3] + P[14][4]*SPP[6] - P[15][4]*SPP[9]);
        nextP[5][7] = P[5][7] + P[0][7]*SF[4] + P[2][7]*SF[3] + P[3][7]*SF[5] - P[1][7]*SPP[0] - P[13][7]*SPP[8] + P[14][7]*SPP[2] + P[15][7]*SPP[5] + dt*(P[5][4] + P[0][4]*SF[4] + P[2][4]*SF[3] + P[3][4]*SF[5] - P[1][4]*SPP[0] - P[13][4]*SPP[8] + P[14][4]*SPP[2] + P[15][4]*SPP[5]);
        nextP[6][7] = P[6][7] + P[1][7]*SF[4] - P[2][7]*SF[5] + P[3][7]*SF[3] + P[0][7]*SPP[0] + P[13][7]*SPP[4] - P[14][7]*SPP[7] - P[15][7]*SPP[1] + dt*(P[6][4] + P[1][4]*SF[4] - P[2][4]*SF[5] + P[3][4]*SF[3] + P[0][4]*SPP[0] + P[13][4]*SPP[4] - P[14][4]*SPP[7] - P[15][4]*SPP[1]);
        nextP[7][7] = P[7][7] + P[4][7]*dt + dt*(P[7][4] + P[4][4]*dt);
        nextP[0][8] = P[0][8] + P[1][8]*SF[9] + P[2][8]*SF[11] + P[3][8]*SF[10] + P[10][8]*SF[14] + P[11][8]*SF[15] + P[12][8]*SPP[10] + dt*(P[0][5] + P[1][5]*SF[9] + P[2][5]*SF[11] + P[3][5]*SF[10] + P[10][5]*SF[14] + P[11][5]*SF[15] + P[12][5]*SPP[10]);
        nextP[1][8] = P[1][8] + P[0][8]*SF[8] + P[2][8]*SF[7] + P[3][8]*SF[11] - P[12][8]*SF[15] + P[11][8]*SPP[10] - (P[10][8]*q0)/2 + dt*(P[1][5] + P[0][5]*SF[8] + P[2][5]*SF[7] + P[3][5]*SF[11] - P[12][5]*SF[15] + P[11][5]*SPP[10] - (P[10][5]*q0)/2);
        nextP[2][8] = P[2][8] + P[0][8]*SF[6] + P[1][8]*SF[10] + P[3][8]*SF[8] + P[12][8]*SF[14] - P[10][8]*SPP[10] - (P[11][8]*q0)/2 + dt*(P[2][5] + P[0][5]*SF[6] + P[1][5]*SF[10] + P[3][5]*SF[8] + P[12][5]*SF[14] - P[10][5]*SPP[10] - (P[11][5]*q0)/2);
        nextP[3][8] = P[3][8] + P[0][8]*SF[7] + P[1][8]*SF[6] + P[2][8]*SF[9] + P[10][8]*SF[15] - P[11][8]*SF[14] - (P[12][8]*q0)/2 + dt*(P[3][5] + P[0][5]*SF[7] + P[1][5]*SF[6] + P[2][5]*SF[9] + P[10][5]*SF[15] - P[11][5]*SF[14] - (P[12][5]*q0)/2);
        nextP[4][8] = P[4][8] + P[0][8]*SF[5] + P[1][8]*SF[3] - P[3][8]*SF[4] + P[2][8]*SPP[0] + P[13][8]*SPP[3] + P[14][8]*SPP[6] - P[15][8]*SPP[9] + dt*(P[4][5] + P[0][5]*SF[5] + P[1][5]*SF[3] - P[3][5]*SF[4] + P[2][5]*SPP[0] + P[13][5]*SPP[3] + P[14][5]*SPP[6] - P[15][5]*SPP[9]);
        nextP[5][8] = P[5][8] + P[0][8]*SF[4] + P[2][8]*SF[3] + P[3][8]*SF[5] - P[1][8]*SPP[0] - P[13][8]*SPP[8] + P[14][8]*SPP[2] + P[15][8]*SPP[5] + dt*(P[5][5] + P[0][5]*SF[4] + P[2][5]*SF[3] + P[3][5]*SF[5] - P[1][5]*SPP[0] - P[13][5]*SPP[8] + P[14][5]*SPP[2] + P[15][5]*SPP[5]);
        nextP[6][8] = P[6][8] + P[1][8]*SF[4] - P[2][8]*SF[5] + P[3][8]*SF[3] + P[0][8]*SPP[0] + P[13][8]*SPP[4] - P[14][8]*SPP[7] - P[15][8]*SPP[1] + dt*(P[6][5] + P[1][5]*SF[4] - P[2][5]*SF[5] + P[3][5]*SF[3] + P[0][5]*SPP[0] + P[13][5]*SPP[4] - P[14][5]*SPP[7] - P[15][5]*SPP[1]);
        nextP[7][8] = P[7][8] + P[4][8]*dt + dt*(P[7][5] + P[4][5]*dt);
        nextP[8][8] = P[8][8] + P[5][8]*dt + dt*(P[8][5] + P[5][5]*dt);
        nextP[0][9] = P[0][9] + P[1][9]*SF[9] + P[2][9]*SF[11] + P[3][9]*SF[10] + P[10][9]*SF[14] + P[11][9]*SF[15] + P[12][9]*SPP[10] + dt*(P[0][6] + P[1][6]*SF[9] + P[2][6]*SF[11] + P[3][6]*SF[10] + P[10][6]*SF[14] + P[11][6]*SF[15] + P[12][6]*SPP[10]);
        nextP[1][9] = P[1][9] + P[0][9]*SF[8] + P[2][9]*SF[7] + P[3][9]*SF[11] - P[12][9]*SF[15] + P[11][9]*SPP[10] - (P[10][9]*q0)/2 + dt*(P[1][6] + P[0][6]*SF[8] + P[2][6]*SF[7] + P[3][6]*SF[11] - P[12][6]*SF[15] + P[11][6]*SPP[10] - (P[10][6]*q0)/2);
        nextP[2][9] = P[2][9] + P[0][9]*SF[6] + P[1][9]*SF[10] + P[3][9]*SF[8] + P[12][9]*SF[14] - P[10][9]*SPP[10] - (P[11][9]*q0)/2 + dt*(P[2][6] + P[0][6]*SF[6] + P[1][6]*SF[10] + P[3][6]*SF[8] + P[12][6]*SF[14] - P[10][6]*SPP[10] - (P[11][6]*q0)/2);
        nextP[3][9] = P[3][9] + P[0][9]*SF[7] + P[1][9]*SF[6] + P[2][9]*SF[9] + P[10][9]*SF[15] - P[11][9]*SF[14] - (P[12][9]*q0)/2 + dt*(P[3][6] + P[0][6]*SF[7] + P[1][6]*SF[6] + P[2][6]*SF[9] + P[10][6]*SF[15] - P[11][6]*SF[14] - (P[12][6]*q0)/2);
        nextP[4][9] = P[4][9] + P[0][9]*SF[5] + P[1][9]*SF[3] - P[3][9]*SF[4] + P[2][9]*SPP[0] + P[13][9]*SPP[3] + P[14][9]*SPP[6] - P[15][9]*SPP[9] + dt*(P[4][6] + P[0][6]*SF[5] + P[1][6]*SF[3] - P[3][6]*SF[4] + P[2][6]*SPP[0] + P[13][6]*SPP[3] + P[14][6]*SPP[6] - P[15][6]*SPP[9]);
        nextP[5][9] = P[5][9] + P[0][9]*SF[4] + P[2][9]*SF[3] + P[3][9]*SF[5] - P[1][9]*SPP[0] - P[13][9]*SPP[8] + P[14][9]*SPP[2] + P[15][9]*SPP[5] + dt*(P[5][6] + P[0][6]*SF[4] + P[2][6]*SF[3] + P[3][6]*SF[5] - P[1][6]*SPP[0] - P[13][6]*SPP[8] + P[14][6]*SPP[2] + P[15][6]*SPP[5]);
        nextP[6][9] = P[6][9] + P[1][9]*SF[4] - P[2][9]*SF[5] + P[3][9]*SF[3] + P[0][9]*SPP[0] + P[13][9]*SPP[4] - P[14][9]*SPP[7] - P[15][9]*SPP[1] + dt*(P[6][6] + P[1][6]*SF[4] - P[2][6]*SF[5] + P[3][6]*SF[3] + P[0][6]*SPP[0] + P[13][6]*SPP[4] - P[14][6]*SPP[7] - P[15][6]*SPP[1]);
        nextP[7][9] = P[7][9] + P[4][9]*dt + dt*(P[7][6] + P[4][6]*dt);
        nextP[8][9] = P[8][9] + P[5][9]*dt + dt*(P[8][6] + P[5][6]*dt);
        nextP[9][9] = P[9][9] + P[6][9]*dt + dt*(P[9][6] + P[6][6]*dt);
        nextP[0][10] = P[0][10] + P[1][10]*SF[9] + P[2][10]*SF[11] + P[3][10]*SF[10] + P[10][10]*SF[14] + P[11][10]*SF[15] + P[12][10]*SPP[10];
        nextP[1][10] = P[1][10] + P[0][10]*SF[8] + P[2][10]*SF[7] + P[3][10]*SF[11] - P[12][10]*SF[15] + P[11][10]*SPP[10] - (P[10][10]*q0)/2;
        nextP[2][10] = P[2][10] + P[0][10]*SF[6] + P[1][10]*SF[10] + P[3][10]*SF[8] + P[12][10]*SF[14] - P[10][10]*SPP[10] - (P[11][10]*q0)/2;
        nextP[3][10] = P[3][10] + P[0][10]*SF[7] + P[1][10]*SF[6] + P[2][10]*SF[9] + P[10][10]*SF[15] - P[11][10]*SF[14] - (P[12][10]*q0)/2;
        nextP[4][10] = P[4][10] + P[0][10]*SF[5] + P[1][10]*SF[3] - P[3][10]*SF[4] + P[2][10]*SPP[0] + P[13][10]*SPP[3] + P[14][10]*SPP[6] - P[15][10]*SPP[9];
        nextP[5][10] = P[5][10] + P[0][10]*SF[4] + P[2][10]*SF[3] + P[3][10]*SF[5] - P[1][10]*SPP[0] - P[13][10]*SPP[8] + P[14][10]*SPP[2] + P[15][10]*SPP[5];
        nextP[6][10] = P[6][10] + P[1][10]*SF[4] - P[2][10]*SF[5] + P[3][10]*SF[3] + P[0][10]*SPP[0] + P[13][10]*SPP[4] - P[14][10]*SPP[7] - P[15][10]*SPP[1];
        nextP[7][10] = P[7][10] + P[4][10]*dt;
        nextP[8][10] = P[8][10] + P[5][10]*dt;
        nextP[9][10] = P[9][10] + P[6][10]*dt;
        nextP[10][10] = P[10][10];
        nextP[0][11] = P[0][11] + P[1][11]*SF[9] + P[2][11]*SF[11] + P[3][11]*SF[10] + P[10][11]*SF[14] + P[11][11]*SF[15] + P[12][11]*SPP[10];
        nextP[1][11] = P[1][11] + P[0][11]*SF[8] + P[2][11]*SF[7] + P[3][11]*SF[11] - P[12][11]*SF[15] + P[11][11]*SPP[10] - (P[10][11]*q0)/2;
        nextP[2][11] = P[2][11] + P[0][11]*SF[6] + P[1][11]*SF[10] + P[3][11]*SF[8] + P[12][11]*SF[14] - P[10][11]*SPP[10] - (P[11][11]*q0)/2;
        nextP[3][11] = P[3][11] + P[0][11]*SF[7] + P[1][11]*SF[6] + P[2][11]*SF[9] + P[10][11]*SF[15] - P[11][11]*SF[14] - (P[12][11]*q0)/2;
        nextP[4][11] = P[4][11] + P[0][11]*SF[5] + P[1][11]*SF[3] - P[3][11]*SF[4] + P[2][11]*SPP[0] + P[13][11]*SPP[3] + P[14][11]*SPP[6] - P[15][11]*SPP[9];
        nextP[5][11] = P[5][11] + P[0][11]*SF[4] + P[2][11]*SF[3] + P[3][11]*SF[5] - P[1][11]*SPP[0] - P[13][11]*SPP[8] + P[14][11]*SPP[2] + P[15][11]*SPP[5];
        nextP[6][11] = P[6][11] + P[1][11]*SF[4] - P[2][11]*SF[5] + P[3][11]*SF[3] + P[0][11]*SPP[0] + P[13][11]*SPP[4] - P[14][11]*SPP[7] - P[15][11]*SPP[1];
        nextP[7][11] = P[7][11] + P[4][11]*dt;
        nextP[8][11] = P[8][11] + P[5][11]*dt;
        nextP[9][11] = P[9][11] + P[6][11]*dt;
        nextP[10][11] = P[10][11];
        nextP[11][11] = P[11][11];
        nextP[0][12] = P[0][12] + P[1][12]*SF[9] + P[2][12]*SF[11] + P[3][12]*SF[10] + P[10][12]*SF[14] + P[11][12]*SF[15] + P[12][12]*SPP[10];
        nextP[1][12] = P[1][12] + P[0][12]*SF[8] + P[2][12]*SF[7] + P[3][12]*SF[11] - P[12][12]*SF[15] + P[11][12]*SPP[10] - (P[10][12]*q0)/2;
        nextP[2][12] = P[2][12] + P[0][12]*SF[6] + P[1][12]*SF[10] + P[3][12]*SF[8] + P[12][12]*SF[14] - P[10][12]*SPP[10] - (P[11][12]*q0)/2;
        nextP[3][12] = P[3][12] + P[0][12]*SF[7] + P[1][12]*SF[6] + P[2][12]*SF[9] + P[10][12]*SF[15] - P[11][12]*SF[14] - (P[12][12]*q0)/2;
        nextP[4][12] = P[4][12] + P[0][12]*SF[5] + P[1][12]*SF[3] - P[3][12]*SF[4] + P[2][12]*SPP[0] + P[13][12]*SPP[3] + P[14][12]*SPP[6] - P[15][12]*SPP[9];
        nextP[5][12] = P[5][12] + P[0][12]*SF[4] + P[2][12]*SF[3] + P[3][12]*SF[5] - P[1][12]*SPP[0] - P[13][12]*SPP[8] + P[14][12]*SPP[2] + P[15][12]*SPP[5];
        nextP[6][12] = P[6][12] + P[1][12]*SF[4] - P[2][12]*SF[5] + P[3][12]*SF[3] + P[0][12]*SPP[0] + P[13][12]*SPP[4] - P[14][12]*SPP[7] - P[15][12]*SPP[1];
        nextP[7][12] = P[7][12] + P[4][12]*dt;
        nextP[8][12] = P[8][12] + P[5][12]*dt;
        nextP[9][12] = P[9][12] + P[6][12]*dt;
        nextP[10][12] = P[10][12];
        nextP[11][12] = P[11][12];
        nextP[12][12] = P[12][12];

        // add process noise that is not from the IMU
        for (unsigned k = 0; k <= 12; k++) {
            nextP[k][k] += process_noise[k];
        }

        // Don't calculate these covariance terms if IMU delta velocity bias estimation is inhibited
        if (!(Node::fusion_param.fusion_mode & MASK_INHIBIT_ACC_BIAS) && !_accel_bias_inhibit) {

            // calculate variances and upper diagonal covariances for IMU delta velocity bias states
            nextP[0][13] = P[0][13] + P[1][13]*SF[9] + P[2][13]*SF[11] + P[3][13]*SF[10] + P[10][13]*SF[14] + P[11][13]*SF[15] + P[12][13]*SPP[10];
            nextP[1][13] = P[1][13] + P[0][13]*SF[8] + P[2][13]*SF[7] + P[3][13]*SF[11] - P[12][13]*SF[15] + P[11][13]*SPP[10] - (P[10][13]*q0)/2;
            nextP[2][13] = P[2][13] + P[0][13]*SF[6] + P[1][13]*SF[10] + P[3][13]*SF[8] + P[12][13]*SF[14] - P[10][13]*SPP[10] - (P[11][13]*q0)/2;
            nextP[3][13] = P[3][13] + P[0][13]*SF[7] + P[1][13]*SF[6] + P[2][13]*SF[9] + P[10][13]*SF[15] - P[11][13]*SF[14] - (P[12][13]*q0)/2;
            nextP[4][13] = P[4][13] + P[0][13]*SF[5] + P[1][13]*SF[3] - P[3][13]*SF[4] + P[2][13]*SPP[0] + P[13][13]*SPP[3] + P[14][13]*SPP[6] - P[15][13]*SPP[9];
            nextP[5][13] = P[5][13] + P[0][13]*SF[4] + P[2][13]*SF[3] + P[3][13]*SF[5] - P[1][13]*SPP[0] - P[13][13]*SPP[8] + P[14][13]*SPP[2] + P[15][13]*SPP[5];
            nextP[6][13] = P[6][13] + P[1][13]*SF[4] - P[2][13]*SF[5] + P[3][13]*SF[3] + P[0][13]*SPP[0] + P[13][13]*SPP[4] - P[14][13]*SPP[7] - P[15][13]*SPP[1];
            nextP[7][13] = P[7][13] + P[4][13]*dt;
            nextP[8][13] = P[8][13] + P[5][13]*dt;
            nextP[9][13] = P[9][13] + P[6][13]*dt;
            nextP[10][13] = P[10][13];
            nextP[11][13] = P[11][13];
            nextP[12][13] = P[12][13];
            nextP[13][13] = P[13][13];
            nextP[0][14] = P[0][14] + P[1][14]*SF[9] + P[2][14]*SF[11] + P[3][14]*SF[10] + P[10][14]*SF[14] + P[11][14]*SF[15] + P[12][14]*SPP[10];
            nextP[1][14] = P[1][14] + P[0][14]*SF[8] + P[2][14]*SF[7] + P[3][14]*SF[11] - P[12][14]*SF[15] + P[11][14]*SPP[10] - (P[10][14]*q0)/2;
            nextP[2][14] = P[2][14] + P[0][14]*SF[6] + P[1][14]*SF[10] + P[3][14]*SF[8] + P[12][14]*SF[14] - P[10][14]*SPP[10] - (P[11][14]*q0)/2;
            nextP[3][14] = P[3][14] + P[0][14]*SF[7] + P[1][14]*SF[6] + P[2][14]*SF[9] + P[10][14]*SF[15] - P[11][14]*SF[14] - (P[12][14]*q0)/2;
            nextP[4][14] = P[4][14] + P[0][14]*SF[5] + P[1][14]*SF[3] - P[3][14]*SF[4] + P[2][14]*SPP[0] + P[13][14]*SPP[3] + P[14][14]*SPP[6] - P[15][14]*SPP[9];
            nextP[5][14] = P[5][14] + P[0][14]*SF[4] + P[2][14]*SF[3] + P[3][14]*SF[5] - P[1][14]*SPP[0] - P[13][14]*SPP[8] + P[14][14]*SPP[2] + P[15][14]*SPP[5];
            nextP[6][14] = P[6][14] + P[1][14]*SF[4] - P[2][14]*SF[5] + P[3][14]*SF[3] + P[0][14]*SPP[0] + P[13][14]*SPP[4] - P[14][14]*SPP[7] - P[15][14]*SPP[1];
            nextP[7][14] = P[7][14] + P[4][14]*dt;
            nextP[8][14] = P[8][14] + P[5][14]*dt;
            nextP[9][14] = P[9][14] + P[6][14]*dt;
            nextP[10][14] = P[10][14];
            nextP[11][14] = P[11][14];
            nextP[12][14] = P[12][14];
            nextP[13][14] = P[13][14];
            nextP[14][14] = P[14][14];
            nextP[0][15] = P[0][15] + P[1][15]*SF[9] + P[2][15]*SF[11] + P[3][15]*SF[10] + P[10][15]*SF[14] + P[11][15]*SF[15] + P[12][15]*SPP[10];
            nextP[1][15] = P[1][15] + P[0][15]*SF[8] + P[2][15]*SF[7] + P[3][15]*SF[11] - P[12][15]*SF[15] + P[11][15]*SPP[10] - (P[10][15]*q0)/2;
            nextP[2][15] = P[2][15] + P[0][15]*SF[6] + P[1][15]*SF[10] + P[3][15]*SF[8] + P[12][15]*SF[14] - P[10][15]*SPP[10] - (P[11][15]*q0)/2;
            nextP[3][15] = P[3][15] + P[0][15]*SF[7] + P[1][15]*SF[6] + P[2][15]*SF[9] + P[10][15]*SF[15] - P[11][15]*SF[14] - (P[12][15]*q0)/2;
            nextP[4][15] = P[4][15] + P[0][15]*SF[5] + P[1][15]*SF[3] - P[3][15]*SF[4] + P[2][15]*SPP[0] + P[13][15]*SPP[3] + P[14][15]*SPP[6] - P[15][15]*SPP[9];
            nextP[5][15] = P[5][15] + P[0][15]*SF[4] + P[2][15]*SF[3] + P[3][15]*SF[5] - P[1][15]*SPP[0] - P[13][15]*SPP[8] + P[14][15]*SPP[2] + P[15][15]*SPP[5];
            nextP[6][15] = P[6][15] + P[1][15]*SF[4] - P[2][15]*SF[5] + P[3][15]*SF[3] + P[0][15]*SPP[0] + P[13][15]*SPP[4] - P[14][15]*SPP[7] - P[15][15]*SPP[1];
            nextP[7][15] = P[7][15] + P[4][15]*dt;
            nextP[8][15] = P[8][15] + P[5][15]*dt;
            nextP[9][15] = P[9][15] + P[6][15]*dt;
            nextP[10][15] = P[10][15];
            nextP[11][15] = P[11][15];
            nextP[12][15] = P[12][15];
            nextP[13][15] = P[13][15];
            nextP[14][15] = P[14][15];
            nextP[15][15] = P[15][15];
    
        	// add process noise that is not from the IMU
		    for (unsigned i = 13; i <= 15; i++) {
		    	nextP[i][i] += process_noise[i];
		    }
        } else {
            // Inhibit delta velocity bias learning by zeroing the covariance terms
    		zeroRows(nextP,13,15);
    		zeroCols(nextP,13,15);
        } 

        // Don't do covariance prediction on magnetic field states unless we are using 3-axis fusion
        if (Node::prediction_param.mag_3D) {
            // calculate variances and upper diagonal covariances for earth and body magnetic field states
            nextP[0][16] = P[0][16] + P[1][16]*SF[9] + P[2][16]*SF[11] + P[3][16]*SF[10] + P[10][16]*SF[14] + P[11][16]*SF[15] + P[12][16]*SPP[10];
            nextP[1][16] = P[1][16] + P[0][16]*SF[8] + P[2][16]*SF[7] + P[3][16]*SF[11] - P[12][16]*SF[15] + P[11][16]*SPP[10] - (P[10][16]*q0)/2;
            nextP[2][16] = P[2][16] + P[0][16]*SF[6] + P[1][16]*SF[10] + P[3][16]*SF[8] + P[12][16]*SF[14] - P[10][16]*SPP[10] - (P[11][16]*q0)/2;
            nextP[3][16] = P[3][16] + P[0][16]*SF[7] + P[1][16]*SF[6] + P[2][16]*SF[9] + P[10][16]*SF[15] - P[11][16]*SF[14] - (P[12][16]*q0)/2;
            nextP[4][16] = P[4][16] + P[0][16]*SF[5] + P[1][16]*SF[3] - P[3][16]*SF[4] + P[2][16]*SPP[0] + P[13][16]*SPP[3] + P[14][16]*SPP[6] - P[15][16]*SPP[9];
            nextP[5][16] = P[5][16] + P[0][16]*SF[4] + P[2][16]*SF[3] + P[3][16]*SF[5] - P[1][16]*SPP[0] - P[13][16]*SPP[8] + P[14][16]*SPP[2] + P[15][16]*SPP[5];
            nextP[6][16] = P[6][16] + P[1][16]*SF[4] - P[2][16]*SF[5] + P[3][16]*SF[3] + P[0][16]*SPP[0] + P[13][16]*SPP[4] - P[14][16]*SPP[7] - P[15][16]*SPP[1];
            nextP[7][16] = P[7][16] + P[4][16]*dt;
            nextP[8][16] = P[8][16] + P[5][16]*dt;
            nextP[9][16] = P[9][16] + P[6][16]*dt;
            nextP[10][16] = P[10][16];
            nextP[11][16] = P[11][16];
            nextP[12][16] = P[12][16];
            nextP[13][16] = P[13][16];
            nextP[14][16] = P[14][16];
            nextP[15][16] = P[15][16];
            nextP[16][16] = P[16][16];
            nextP[0][17] = P[0][17] + P[1][17]*SF[9] + P[2][17]*SF[11] + P[3][17]*SF[10] + P[10][17]*SF[14] + P[11][17]*SF[15] + P[12][17]*SPP[10];
            nextP[1][17] = P[1][17] + P[0][17]*SF[8] + P[2][17]*SF[7] + P[3][17]*SF[11] - P[12][17]*SF[15] + P[11][17]*SPP[10] - (P[10][17]*q0)/2;
            nextP[2][17] = P[2][17] + P[0][17]*SF[6] + P[1][17]*SF[10] + P[3][17]*SF[8] + P[12][17]*SF[14] - P[10][17]*SPP[10] - (P[11][17]*q0)/2;
            nextP[3][17] = P[3][17] + P[0][17]*SF[7] + P[1][17]*SF[6] + P[2][17]*SF[9] + P[10][17]*SF[15] - P[11][17]*SF[14] - (P[12][17]*q0)/2;
            nextP[4][17] = P[4][17] + P[0][17]*SF[5] + P[1][17]*SF[3] - P[3][17]*SF[4] + P[2][17]*SPP[0] + P[13][17]*SPP[3] + P[14][17]*SPP[6] - P[15][17]*SPP[9];
            nextP[5][17] = P[5][17] + P[0][17]*SF[4] + P[2][17]*SF[3] + P[3][17]*SF[5] - P[1][17]*SPP[0] - P[13][17]*SPP[8] + P[14][17]*SPP[2] + P[15][17]*SPP[5];
            nextP[6][17] = P[6][17] + P[1][17]*SF[4] - P[2][17]*SF[5] + P[3][17]*SF[3] + P[0][17]*SPP[0] + P[13][17]*SPP[4] - P[14][17]*SPP[7] - P[15][17]*SPP[1];
            nextP[7][17] = P[7][17] + P[4][17]*dt;
            nextP[8][17] = P[8][17] + P[5][17]*dt;
            nextP[9][17] = P[9][17] + P[6][17]*dt;
            nextP[10][17] = P[10][17];
            nextP[11][17] = P[11][17];
            nextP[12][17] = P[12][17];
            nextP[13][17] = P[13][17];
            nextP[14][17] = P[14][17];
            nextP[15][17] = P[15][17];
            nextP[16][17] = P[16][17];
            nextP[17][17] = P[17][17];
            nextP[0][18] = P[0][18] + P[1][18]*SF[9] + P[2][18]*SF[11] + P[3][18]*SF[10] + P[10][18]*SF[14] + P[11][18]*SF[15] + P[12][18]*SPP[10];
            nextP[1][18] = P[1][18] + P[0][18]*SF[8] + P[2][18]*SF[7] + P[3][18]*SF[11] - P[12][18]*SF[15] + P[11][18]*SPP[10] - (P[10][18]*q0)/2;
            nextP[2][18] = P[2][18] + P[0][18]*SF[6] + P[1][18]*SF[10] + P[3][18]*SF[8] + P[12][18]*SF[14] - P[10][18]*SPP[10] - (P[11][18]*q0)/2;
            nextP[3][18] = P[3][18] + P[0][18]*SF[7] + P[1][18]*SF[6] + P[2][18]*SF[9] + P[10][18]*SF[15] - P[11][18]*SF[14] - (P[12][18]*q0)/2;
            nextP[4][18] = P[4][18] + P[0][18]*SF[5] + P[1][18]*SF[3] - P[3][18]*SF[4] + P[2][18]*SPP[0] + P[13][18]*SPP[3] + P[14][18]*SPP[6] - P[15][18]*SPP[9];
            nextP[5][18] = P[5][18] + P[0][18]*SF[4] + P[2][18]*SF[3] + P[3][18]*SF[5] - P[1][18]*SPP[0] - P[13][18]*SPP[8] + P[14][18]*SPP[2] + P[15][18]*SPP[5];
            nextP[6][18] = P[6][18] + P[1][18]*SF[4] - P[2][18]*SF[5] + P[3][18]*SF[3] + P[0][18]*SPP[0] + P[13][18]*SPP[4] - P[14][18]*SPP[7] - P[15][18]*SPP[1];
            nextP[7][18] = P[7][18] + P[4][18]*dt;
            nextP[8][18] = P[8][18] + P[5][18]*dt;
            nextP[9][18] = P[9][18] + P[6][18]*dt;
            nextP[10][18] = P[10][18];
            nextP[11][18] = P[11][18];
            nextP[12][18] = P[12][18];
            nextP[13][18] = P[13][18];
            nextP[14][18] = P[14][18];
            nextP[15][18] = P[15][18];
            nextP[16][18] = P[16][18];
            nextP[17][18] = P[17][18];
            nextP[18][18] = P[18][18];
            nextP[0][19] = P[0][19] + P[1][19]*SF[9] + P[2][19]*SF[11] + P[3][19]*SF[10] + P[10][19]*SF[14] + P[11][19]*SF[15] + P[12][19]*SPP[10];
            nextP[1][19] = P[1][19] + P[0][19]*SF[8] + P[2][19]*SF[7] + P[3][19]*SF[11] - P[12][19]*SF[15] + P[11][19]*SPP[10] - (P[10][19]*q0)/2;
            nextP[2][19] = P[2][19] + P[0][19]*SF[6] + P[1][19]*SF[10] + P[3][19]*SF[8] + P[12][19]*SF[14] - P[10][19]*SPP[10] - (P[11][19]*q0)/2;
            nextP[3][19] = P[3][19] + P[0][19]*SF[7] + P[1][19]*SF[6] + P[2][19]*SF[9] + P[10][19]*SF[15] - P[11][19]*SF[14] - (P[12][19]*q0)/2;
            nextP[4][19] = P[4][19] + P[0][19]*SF[5] + P[1][19]*SF[3] - P[3][19]*SF[4] + P[2][19]*SPP[0] + P[13][19]*SPP[3] + P[14][19]*SPP[6] - P[15][19]*SPP[9];
            nextP[5][19] = P[5][19] + P[0][19]*SF[4] + P[2][19]*SF[3] + P[3][19]*SF[5] - P[1][19]*SPP[0] - P[13][19]*SPP[8] + P[14][19]*SPP[2] + P[15][19]*SPP[5];
            nextP[6][19] = P[6][19] + P[1][19]*SF[4] - P[2][19]*SF[5] + P[3][19]*SF[3] + P[0][19]*SPP[0] + P[13][19]*SPP[4] - P[14][19]*SPP[7] - P[15][19]*SPP[1];
            nextP[7][19] = P[7][19] + P[4][19]*dt;
            nextP[8][19] = P[8][19] + P[5][19]*dt;
            nextP[9][19] = P[9][19] + P[6][19]*dt;
            nextP[10][19] = P[10][19];
            nextP[11][19] = P[11][19];
            nextP[12][19] = P[12][19];
            nextP[13][19] = P[13][19];
            nextP[14][19] = P[14][19];
            nextP[15][19] = P[15][19];
            nextP[16][19] = P[16][19];
            nextP[17][19] = P[17][19];
            nextP[18][19] = P[18][19];
            nextP[19][19] = P[19][19];
            nextP[0][20] = P[0][20] + P[1][20]*SF[9] + P[2][20]*SF[11] + P[3][20]*SF[10] + P[10][20]*SF[14] + P[11][20]*SF[15] + P[12][20]*SPP[10];
            nextP[1][20] = P[1][20] + P[0][20]*SF[8] + P[2][20]*SF[7] + P[3][20]*SF[11] - P[12][20]*SF[15] + P[11][20]*SPP[10] - (P[10][20]*q0)/2;
            nextP[2][20] = P[2][20] + P[0][20]*SF[6] + P[1][20]*SF[10] + P[3][20]*SF[8] + P[12][20]*SF[14] - P[10][20]*SPP[10] - (P[11][20]*q0)/2;
            nextP[3][20] = P[3][20] + P[0][20]*SF[7] + P[1][20]*SF[6] + P[2][20]*SF[9] + P[10][20]*SF[15] - P[11][20]*SF[14] - (P[12][20]*q0)/2;
            nextP[4][20] = P[4][20] + P[0][20]*SF[5] + P[1][20]*SF[3] - P[3][20]*SF[4] + P[2][20]*SPP[0] + P[13][20]*SPP[3] + P[14][20]*SPP[6] - P[15][20]*SPP[9];
            nextP[5][20] = P[5][20] + P[0][20]*SF[4] + P[2][20]*SF[3] + P[3][20]*SF[5] - P[1][20]*SPP[0] - P[13][20]*SPP[8] + P[14][20]*SPP[2] + P[15][20]*SPP[5];
            nextP[6][20] = P[6][20] + P[1][20]*SF[4] - P[2][20]*SF[5] + P[3][20]*SF[3] + P[0][20]*SPP[0] + P[13][20]*SPP[4] - P[14][20]*SPP[7] - P[15][20]*SPP[1];
            nextP[7][20] = P[7][20] + P[4][20]*dt;
            nextP[8][20] = P[8][20] + P[5][20]*dt;
            nextP[9][20] = P[9][20] + P[6][20]*dt;
            nextP[10][20] = P[10][20];
            nextP[11][20] = P[11][20];
            nextP[12][20] = P[12][20];
            nextP[13][20] = P[13][20];
            nextP[14][20] = P[14][20];
            nextP[15][20] = P[15][20];
            nextP[16][20] = P[16][20];
            nextP[17][20] = P[17][20];
            nextP[18][20] = P[18][20];
            nextP[19][20] = P[19][20];
            nextP[20][20] = P[20][20];
            nextP[0][21] = P[0][21] + P[1][21]*SF[9] + P[2][21]*SF[11] + P[3][21]*SF[10] + P[10][21]*SF[14] + P[11][21]*SF[15] + P[12][21]*SPP[10];
            nextP[1][21] = P[1][21] + P[0][21]*SF[8] + P[2][21]*SF[7] + P[3][21]*SF[11] - P[12][21]*SF[15] + P[11][21]*SPP[10] - (P[10][21]*q0)/2;
            nextP[2][21] = P[2][21] + P[0][21]*SF[6] + P[1][21]*SF[10] + P[3][21]*SF[8] + P[12][21]*SF[14] - P[10][21]*SPP[10] - (P[11][21]*q0)/2;
            nextP[3][21] = P[3][21] + P[0][21]*SF[7] + P[1][21]*SF[6] + P[2][21]*SF[9] + P[10][21]*SF[15] - P[11][21]*SF[14] - (P[12][21]*q0)/2;
            nextP[4][21] = P[4][21] + P[0][21]*SF[5] + P[1][21]*SF[3] - P[3][21]*SF[4] + P[2][21]*SPP[0] + P[13][21]*SPP[3] + P[14][21]*SPP[6] - P[15][21]*SPP[9];
            nextP[5][21] = P[5][21] + P[0][21]*SF[4] + P[2][21]*SF[3] + P[3][21]*SF[5] - P[1][21]*SPP[0] - P[13][21]*SPP[8] + P[14][21]*SPP[2] + P[15][21]*SPP[5];
            nextP[6][21] = P[6][21] + P[1][21]*SF[4] - P[2][21]*SF[5] + P[3][21]*SF[3] + P[0][21]*SPP[0] + P[13][21]*SPP[4] - P[14][21]*SPP[7] - P[15][21]*SPP[1];
            nextP[7][21] = P[7][21] + P[4][21]*dt;
            nextP[8][21] = P[8][21] + P[5][21]*dt;
            nextP[9][21] = P[9][21] + P[6][21]*dt;
            nextP[10][21] = P[10][21];
            nextP[11][21] = P[11][21];
            nextP[12][21] = P[12][21];
            nextP[13][21] = P[13][21];
            nextP[14][21] = P[14][21];
            nextP[15][21] = P[15][21];
            nextP[16][21] = P[16][21];
            nextP[17][21] = P[17][21];
            nextP[18][21] = P[18][21];
            nextP[19][21] = P[19][21];
            nextP[20][21] = P[20][21];
            nextP[21][21] = P[21][21];

            // add process noise that is not from the IMU
            for (unsigned i = 16; i <= 21; i++) {
                nextP[i][i] += process_noise[i];
            }

        }

        // Don't do covariance prediction on wind states unless we are using them
        if (Node::prediction_param.wind) {

            // calculate variances and upper diagonal covariances for wind states
            nextP[0][22] = P[0][22] + P[1][22]*SF[9] + P[2][22]*SF[11] + P[3][22]*SF[10] + P[10][22]*SF[14] + P[11][22]*SF[15] + P[12][22]*SPP[10];
            nextP[1][22] = P[1][22] + P[0][22]*SF[8] + P[2][22]*SF[7] + P[3][22]*SF[11] - P[12][22]*SF[15] + P[11][22]*SPP[10] - (P[10][22]*q0)/2;
            nextP[2][22] = P[2][22] + P[0][22]*SF[6] + P[1][22]*SF[10] + P[3][22]*SF[8] + P[12][22]*SF[14] - P[10][22]*SPP[10] - (P[11][22]*q0)/2;
            nextP[3][22] = P[3][22] + P[0][22]*SF[7] + P[1][22]*SF[6] + P[2][22]*SF[9] + P[10][22]*SF[15] - P[11][22]*SF[14] - (P[12][22]*q0)/2;
            nextP[4][22] = P[4][22] + P[0][22]*SF[5] + P[1][22]*SF[3] - P[3][22]*SF[4] + P[2][22]*SPP[0] + P[13][22]*SPP[3] + P[14][22]*SPP[6] - P[15][22]*SPP[9];
            nextP[5][22] = P[5][22] + P[0][22]*SF[4] + P[2][22]*SF[3] + P[3][22]*SF[5] - P[1][22]*SPP[0] - P[13][22]*SPP[8] + P[14][22]*SPP[2] + P[15][22]*SPP[5];
            nextP[6][22] = P[6][22] + P[1][22]*SF[4] - P[2][22]*SF[5] + P[3][22]*SF[3] + P[0][22]*SPP[0] + P[13][22]*SPP[4] - P[14][22]*SPP[7] - P[15][22]*SPP[1];
            nextP[7][22] = P[7][22] + P[4][22]*dt;
            nextP[8][22] = P[8][22] + P[5][22]*dt;
            nextP[9][22] = P[9][22] + P[6][22]*dt;
            nextP[10][22] = P[10][22];
            nextP[11][22] = P[11][22];
            nextP[12][22] = P[12][22];
            nextP[13][22] = P[13][22];
            nextP[14][22] = P[14][22];
            nextP[15][22] = P[15][22];
            nextP[16][22] = P[16][22];
            nextP[17][22] = P[17][22];
            nextP[18][22] = P[18][22];
            nextP[19][22] = P[19][22];
            nextP[20][22] = P[20][22];
            nextP[21][22] = P[21][22];
            nextP[22][22] = P[22][22];
            nextP[0][23] = P[0][23] + P[1][23]*SF[9] + P[2][23]*SF[11] + P[3][23]*SF[10] + P[10][23]*SF[14] + P[11][23]*SF[15] + P[12][23]*SPP[10];
            nextP[1][23] = P[1][23] + P[0][23]*SF[8] + P[2][23]*SF[7] + P[3][23]*SF[11] - P[12][23]*SF[15] + P[11][23]*SPP[10] - (P[10][23]*q0)/2;
            nextP[2][23] = P[2][23] + P[0][23]*SF[6] + P[1][23]*SF[10] + P[3][23]*SF[8] + P[12][23]*SF[14] - P[10][23]*SPP[10] - (P[11][23]*q0)/2;
            nextP[3][23] = P[3][23] + P[0][23]*SF[7] + P[1][23]*SF[6] + P[2][23]*SF[9] + P[10][23]*SF[15] - P[11][23]*SF[14] - (P[12][23]*q0)/2;
            nextP[4][23] = P[4][23] + P[0][23]*SF[5] + P[1][23]*SF[3] - P[3][23]*SF[4] + P[2][23]*SPP[0] + P[13][23]*SPP[3] + P[14][23]*SPP[6] - P[15][23]*SPP[9];
            nextP[5][23] = P[5][23] + P[0][23]*SF[4] + P[2][23]*SF[3] + P[3][23]*SF[5] - P[1][23]*SPP[0] - P[13][23]*SPP[8] + P[14][23]*SPP[2] + P[15][23]*SPP[5];
            nextP[6][23] = P[6][23] + P[1][23]*SF[4] - P[2][23]*SF[5] + P[3][23]*SF[3] + P[0][23]*SPP[0] + P[13][23]*SPP[4] - P[14][23]*SPP[7] - P[15][23]*SPP[1];
            nextP[7][23] = P[7][23] + P[4][23]*dt;
            nextP[8][23] = P[8][23] + P[5][23]*dt;
            nextP[9][23] = P[9][23] + P[6][23]*dt;
            nextP[10][23] = P[10][23];
            nextP[11][23] = P[11][23];
            nextP[12][23] = P[12][23];
            nextP[13][23] = P[13][23];
            nextP[14][23] = P[14][23];
            nextP[15][23] = P[15][23];
            nextP[16][23] = P[16][23];
            nextP[17][23] = P[17][23];
            nextP[18][23] = P[18][23];
            nextP[19][23] = P[19][23];
            nextP[20][23] = P[20][23];
            nextP[21][23] = P[21][23];
            nextP[22][23] = P[22][23];
            nextP[23][23] = P[23][23];

            // add process noise that is not from the IMU
            for (unsigned i = 22; i <= 23; i++) {
                nextP[i][i] += process_noise[i];
            }

        }

        // stop position covariance growth if our total position variance reaches 100m
        // this can happen if we lose gps for some time
        if ((P[7][7] + P[8][8]) > 1e4f) {
            for (uint8_t i = 7; i <= 8; i++) {
                for (uint8_t j = 0; j < 24; j++) {
                    nextP[i][j] = P[i][j];
                    nextP[j][i] = P[j][i];
                }
            }
        }

        // covariance matrix is symmetrical, so copy upper half to lower half
        for (unsigned row = 1; row < 24; row++) {
            for (unsigned column = 0 ; column < row; column++) {
                P[row][column] = P[column][row] = nextP[column][row];
            }
        }

        // copy variances (diagonals)
        for (unsigned i = 0; i < 24; i++) {
            P[i][i] = nextP[i][i];
        }

        for (unsigned i = 0; i < 24; i++) {
            for (unsigned j = 0; j < 24; j++) {
                Covariances(i, j) = P[i][j];
            }
        }
    }


    /*void Node::FuseBodyVel(Vector3d measured_vel_, Vector3d velWorld,  double bodyVelError_) {
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
            //innovation[i] = relVelBodyPred[i] - measured_vel_[i];
            innovation[i] = prevVelWorld_[i] - velWorld[i];
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
            q_ = q_.normalized();
            states_.segment(0, 4) << q_.w(), q_.x(), q_.y(), q_.z();
            Covariances = Covariances.eval() - (Kfusion * H.row(i) * Covariances).eval();
            Covariances = 0.5 * (Covariances.eval() + Covariances.transpose().eval());

            for (int j = 0; j < 24; j++) {
                if (Covariances(j, j) < 0)
                    Covariances(j, j) = 0;
            }
        }
        //cout << "velWorld:" << states_.segment(4, 3) << endl;
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

        marker.pose.position.x = states_[7];
        marker.pose.position.y = states_[8];
        marker.pose.position.z = states_[9];
 
        //cout << "posWorld:" << states_.segment(7, 3) << endl; 
        prevPosWorld_ << states_[7], states_[8], states_[9];
    } */

    void Node::fuseVelPosHeight(Vector3d velWorld, Vector3d posWorld) {
        float R[6] = {}; // observation variances for [VN,VE,VD,PN,PE,PD]  Q
        float gate_size[6] = {}; // innovation consistency check gate sizes for [VN,VE,VD,PN,PE,PD] observations
        float _vel_pos_innov[6] = {};
        float _vel_pos_innov_var[6] = {};
        float _vel_pos_test_radio[6] = {};
        float P[24][24] = {};
        for (int i = 0; i < 24; i++) {
            for (int j = 0; j < 24; j++) {
                P[i][j] = Covariances(i, j);
            }
        }

        // horizontal velocity innovations
        _vel_pos_innov[0] = states_[4] - velWorld[0];
        _vel_pos_innov[1] = states_[5] - velWorld[1];
        _vel_pos_innov[2] = states_[6] - velWorld[2];
        //cout << "delta_vz : " << _vel_pos_innov[2] << " = " << states_[6] << "-" << velWorld[2] << endl;
        _vel_pos_innov[3] = states_[7] - posWorld[0];
        _vel_pos_innov[4] = states_[8] - posWorld[1];
        _vel_pos_innov[5] = states_[9] - posWorld[2];
        

        // observation variance - use receiver reported accuracy with parameter setting the minimum value
        R[0] = fmaxf(Node::aligment_param.zedVelNoise, 0.01f);
        R[0] = fmaxf(R[0], Node::fusion_param.zedVelAcc);
        R[0] = R[0] * R[0];
        R[1] = R[0];
        R[2] = fmaxf(Node::aligment_param.zedVelNoise, 0.01f);
        // use scaled horizontal speed accuracy assuming typical ratio of VDOP/HDOP
        R[2] = 1.5f * fmaxf(R[2], Node::fusion_param.zedVelAcc);
        R[2] = R[2] * R[2];
        // innovation gate size
        gate_size[0] = fmaxf(Node::fusion_param.bodyVelGate, 1.0f);
        gate_size[1] = gate_size[0];
        gate_size[2] = gate_size[0];
        // fusion position variances
        R[4] = R[3] = sq(Node::fusion_param.posObsNoiseNE);
        gate_size[4] = gate_size[3] = Node::fusion_param.posInnovGateNE;
        R[5] = fmaxf(Node::fusion_param.posErr, 0.01f);
        R[5] = R[5] * R[5];
        gate_size[5] = fmaxf(Node::fusion_param.ev_innov_gate, 1.0f);

        for (unsigned obs_index = 0; obs_index < 6; obs_index++) {
            unsigned state_index = obs_index + 4;
            // Ck*Pk*CkT+Qk  compute the innovation variance SK = HPH + R // Ck = H = 1
            _vel_pos_innov_var[obs_index] = P[state_index][state_index] + R[obs_index];
            // compute the ratio of innovation to gate size
            _vel_pos_test_radio[obs_index] = sq(_vel_pos_innov[obs_index]) / (sq(gate_size[obs_index]) * _vel_pos_innov_var[obs_index]);
        }

        // check position, velocity and height innovations
        // treat 3D velocity, 2D position and height as separate sensors
        // always pass position checks if using synthetic position measurements or yet to complete tilt alignment
        // always pass height checks if yet to complete tilt alignment
        bool vel_check_pass = (_vel_pos_test_radio[0] <= 1.0f) && (_vel_pos_test_radio[1] <= 1.0f) && (_vel_pos_test_radio[2] <= 1.0f);
        bool pos_check_pass = (_vel_pos_test_radio[3] <= 1.0f) && (_vel_pos_test_radio[4] <= 1.0f) && (_vel_pos_test_radio[5] <= 1.0f);
        float Kfusion[24] = {};
        for (unsigned obs_index = 0; obs_index < 6; obs_index++) {
            if (obs_index < 3) {
                if (!vel_check_pass) {
                    continue;
                }
            } else {
                if (!pos_check_pass) {
                    continue;
                }
            }

            unsigned state_index = obs_index + 4;
            
            //calculate kalman gain K = PHS, where S = 1/innovation variance
            for (int row = 0; row < 24; row++) {
                Kfusion[row] = P[row][state_index] / _vel_pos_innov_var[obs_index];
            }
            
            //update covariance matrix via Pnew = (I - KH) * P
            float KHP[24][24];
            for (unsigned row = 0; row < 24; row++) {
                for (unsigned column = 0; column < 24; column++) {
                    KHP[row][column] = Kfusion[row] * P[state_index][column];
                }
            }

            // if the covariance correction will result in a negative variance, then
            // the covariance marix is unhealthy and must be corrected
            bool healthy = true;
            for (int i = 0; i < 24; i++) {
                if (P[i][i] < KHP[i][i]) {
                    // zero rows and columns
                    zeroRows(P,i,i);
                    zeroCols(P,i,i);

                    healthy = false;
                }
            }  

            // only apply covariance and state corrrections if healthy
    		if (healthy) {
    			// apply the covariance corrections
    			for (unsigned row = 0; row < 24; row++) {
    				for (unsigned column = 0; column < 24; column++) {
    					P[row][column] = P[row][column] - KHP[row][column];
    				}
    			}
    			// correct the covariance marix for gross errors
                fixCovarianceErrors(P);

                for (unsigned i = 0; i < 24; i++) {
                    states_[i] = states_[i] - Kfusion[i] * _vel_pos_innov[obs_index];
                }

                q_.w() = states_[0];
                q_.x() = states_[1];
                q_.y() = states_[2];
                q_.z() = states_[3];
                q_ = q_.normalized();
                states_.segment(0, 4) << q_.w(), q_.x(), q_.y(), q_.z();
            }

        }

        for (unsigned i = 0; i < 24; i++) {
            for (unsigned j = 0; j < 24; j++) {
                Covariances(i, j) = P[i][j];
            }
        }

        marker.pose.position.x = states_[7];
        marker.pose.position.y = states_[8];
        marker.pose.position.z = states_[9];
 
        //cout << "velWorld:" << states_.segment(4, 3) << endl; 
        //prevVelWorld_ << states_[4], states_[5], states_[6];
        //cout << "posWorld:" << states_.segment(7, 3) << endl; 
        //prevPosWorld_ << states_[7], states_[8], states_[9];


    }

    void Node::fixCovarianceErrors(float (&P)[24][24]) {
        // NOTE: This limiting is a last resort and should not be relied on
        // TODO: Split covariance prediction into separate F*P*transpose(F) and Q contributions
        // and set corresponding entries in Q to zero when states exceed 50% of the limit
        // Covariance diagonal limits. Use same values for states which
        // belong to the same group (e.g. vel_x, vel_y, vel_z)
        float P_lim[8] = {};
        P_lim[0] = 1.0f;		// quaternion max var
        P_lim[1] = 1e6f;		// velocity max var
        P_lim[2] = 1e6f;		// positiion max var
        P_lim[3] = 1.0f;		// gyro bias max var
        P_lim[4] = 1.0f;		// delta velocity z bias max var
        P_lim[5] = 1.0f;		// earth mag field max var
        P_lim[6] = 1.0f;		// body mag field max var
        P_lim[7] = 1e6f;		// wind max var

        for (int i = 0; i <= 3; i++) {
            // quaternion states
            P[i][i] = constrain(P[i][i], 0.0f, P_lim[0]);
        }

        for (int i = 4; i <= 6; i++) {
            // NED velocity states
            P[i][i] = constrain(P[i][i], 0.0f, P_lim[1]);
        }

        for (int i = 7; i <= 9; i++) {
            // NED position states
            P[i][i] = constrain(P[i][i], 0.0f, P_lim[2]);
        }

        for (int i = 10; i <= 12; i++) {
            // gyro bias states
            P[i][i] = constrain(P[i][i], 0.0f, P_lim[3]);
        }

        // force symmetry on the quaternion, velocity and positon state covariances
        makeSymmetrical(P,0,12);

        // the following states are optional and are deactivaed when not required
        // by ensuring the corresponding covariance matrix values are kept at zero

        // accelerometer bias states
        
    }

    void Node::fuseYaw(Quaterniond q_measured_yaw_) {
        Vector3d euler_vis;
        get_euler_from_q(euler_vis, q_measured_yaw_);
        //cout << "euler_vis : " << euler_vis << endl;
        Vector3d euler_before_fuse;
        get_euler_from_q(euler_before_fuse, q_);
        euler_before_fuse[2] = euler_vis[2];
        get_q_from_euler(q_, euler_before_fuse);
        states_.segment(0, 4) << q_.w(), q_.x(), q_.y(), q_.z();

    }

#ifdef USE_LOGGER
    void Node::start_logger(const ros::Time &t) {
        std::string logger_file_name("/home/nuc/fuse_ekf_ws/src/ekf2/src/logger/");
        logger_file_name += "ekf_logger";
        logger_file_name += getTime_string();
        logger_file_name += ".csv";
        if (ekf_logger.is_open()) {
            ekf_logger.close();
        }
        ekf_logger.open(logger_file_name.c_str(), std::ios::out);
        if (!ekf_logger.is_open()) {
            cout << "cannot open the logger." << endl;
        } else {
            ekf_logger << "timestamp" << ",";
            ekf_logger << "roll" << ",";
            ekf_logger << "pitch" << ",";
            ekf_logger << "yaw" << ",";
            ekf_logger << "vel_x(cam)" << ",";
            ekf_logger << "vel_y(cam)" << ",";
            ekf_logger << "vei_z(cam)" << ",";
            ekf_logger << "pos_x(cam)" << ",";
            ekf_logger << "pos_y(cam)" << ",";
            ekf_logger << "pos_z(cam)" << ",";
            ekf_logger << "mag_x(NED)" << ",";
            ekf_logger << "mag_y(NED)" << ",";
            ekf_logger << "mag_z(NED)" << endl; 
        }
    }

    std::string Node::getTime_string() {
        time_t timep;
        timep = time(0);
        char tmp[64];
        strftime(tmp, sizeof(tmp), "%Y_%m_%d_%H_%M_%S", localtime(&timep));
        return tmp;
    }

#endif
}


