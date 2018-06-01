#ifndef FUSE_EKF_TEST_NODE_HPP_
#define FUSE_EKF_TEST_NODE_HPP_

#define USE_LOGGER 1

#include <ros/ros.h>
#include <math.h>
#include <fuse_ekf_test/param_list.hpp>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/LaserScan.h>
#include <message_filters/synchronizer.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/exact_time.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#ifdef USE_LOGGER
#include <string>
#include <iostream>
#include <fstream>
#include <time.h>
#endif

namespace fuse_ekf_test {
    class Node {
        public:
            Node(const ros::NodeHandle& nh);

            //callbacks
            void visual_odom_cb(const geometry_msgs::PoseWithCovarianceStampedConstPtr&); 
            void imu_mag_cb(const sensor_msgs::ImuConstPtr&, const sensor_msgs::MagneticFieldConstPtr&);
            void laser_data_cb(const sensor_msgs::LaserScanConstPtr&);

            void InitStates_Imu(Eigen::Vector3d measured_wm, Eigen::Vector3d measured_am, Eigen::Vector3d measured_mm);
            //void InitCovariance();
            void initializeCovariance(); 
            void initialiseQuatCovariances(Eigen::Vector3f &rot_vec_var);
            void InitStates_Visual(Eigen::Vector3d measured_pos, Eigen::Vector3d measured_vel);
            void InitStates_Laser(float measured_height);

            void PredictStates(Eigen::Vector3d measured_wm_, Eigen::Vector3d measured_am_, Eigen::Vector3d measured_mm_);
            void ConstrainStates();
            //void PredictCovariance();
            void predictCov(Eigen::Vector3d del_Ang_Cov, Eigen::Vector3d del_Vel_Cov, Eigen::Vector3d del_Ang_bias, Eigen::Vector3d del_Vel_bias, double dt, Eigen::Quaterniond q); 
            //void FuseBodyVel(Eigen::Vector3d measured_vel_, Eigen::Vector3d velWorld, double bodyVelError_);
            //void FusePosition(Eigen::Vector3d measured_pos_, double worldPosError_);
            void fuseVelPosHeight(Eigen::Vector3d velWorld, Eigen::Vector3d posWorld);
            void fixCovarianceErrors(float (&P)[24][24]);
            void fuseYaw(Eigen::Quaterniond);
            
#ifdef USE_LOGGER
            void start_logger(const ros::Time &t);
            std::string getTime_string();
#endif

        private:
            static const unsigned int kROSQueueSize = 200;
            ros::NodeHandle nh_;
            ros::Publisher pubImu_;
            ros::Publisher pubBias_;
            ros::Publisher rviz_pub;

            // Subscriber
            message_filters::Subscriber<sensor_msgs::Imu> subImu_;
            message_filters::Subscriber<sensor_msgs::MagneticField> subField_;
            ros::Subscriber subImuUnsync_;
            nav_msgs::Odometry vis_odom_get;
            ros::Subscriber subVisual_;
            ros::Subscriber subLaser_;
            ros::Subscriber rosbag_imu;

            // time synchronizer
            typedef message_filters::sync_policies::ExactTime<sensor_msgs::Imu,sensor_msgs::MagneticField> TimeSyncPolicy;
            message_filters::Synchronizer<TimeSyncPolicy> sync_;
            ros::Time imu_prevStamp;
            ros::Time imu_Stamp;
            ros::Time vis_prevStamp;
            ros::Time vis_Stamp;
            ros::Time laser_prevStamp;
            ros::Time laser_Stamp;

            //options
            bool enableMag_;

            //init param_list
            param::param_control control_param;
            param::param_fusion fusion_param;
            param::param_prediction prediction_param;
            param::param_alignment aligment_param;

            //state vector (4x1 quaternion, 3x1 velocity, 3x1 position, 3x1 delAng bias, 3x1 delVel bias, 3x1 mag_world)
            Eigen::Matrix<double, 24, 1> states_;
            Eigen::AngleAxisd q_init_angaxi;
            Eigen::Quaterniond q_;
            Eigen::Matrix3d Tbn;
            Eigen::Vector3d prevVelWorld_;
            Eigen::Vector3d prevPosWorld_;
            Eigen::Vector3d delAng_bias;
            Eigen::Vector3d delVel_bias;
            Eigen::Vector3d magWorld_;
            visualization_msgs::Marker marker;

            //covariance
            Eigen::Matrix<double, 24, 24> Covariances;
            Eigen::Matrix3d cov_pos;
            Eigen::Matrix3d cov_vel;
            double _ang_rate_mag_filt; 
            double _accel_mag_filt; 
            bool _accel_bias_inhibit;
            bool _bad_vert_accel_detected;	
            Eigen::Vector3d _prev_dvel_bias_var;
            
            //covariance prediction variables
            Eigen::Vector3d delAng_Cov; // delta angle vector used by the covariance prediction (rad)
            Eigen::Vector3d delVel_Cov; // delta velocity vector used by the covariance prediction (m/sec)
            double dt_Cov; // time step used by the covariance prediction (sec)
            double dt_Int_Cov; // accumulated time step of covariance predictions (sec)

            // variables used to control dead-reckoning timeout
            double last_dirft_constrain_time;
            double last_synthetic_velocity_fusion_time;
            double _last_imu_bias_cov_reset_us;

            // variables used by prediction
            Eigen::Vector3d prevDelAng;
            Eigen::Vector3d prevDelVel;
            Eigen::Vector3d correctedDelAng;
            Eigen::Vector3d correctedDelVel;
            
            enum {
                Uncalibrated = 0,
                Calibrating = 1,
                CalibrationComplete = 2,
            } calibState_;
            bool init_; // if init not complete, reinitialize 

#ifdef USE_LOGGER
            // Logger ekf_Logger
            std::ofstream ekf_logger;
#endif

    };
} // namespace fuse_ekf_test

#endif // FUSE_EKF_TEST_NODE_HPP_
