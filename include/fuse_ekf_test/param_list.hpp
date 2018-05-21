#ifndef PARAM_LIST_H_
#define PARAM_LIST_H_

#define _USE_MATH_DEFINES
#include <math.h>
#include <Eigen/Dense>

#define gravity 9.80665
namespace param {
    
    // filter control
    struct param_control {
        param_control(){
            reset_param_control();
        }
        float waitForGps; //set to 1 if the filter start should be delayed until GPS checks to pass
        float gpsSpdErrLim; //GPS use will not start if reported GPS speed error is greater than this (m/s)
        float gpsPosErrLim; //GPS use will not start if reported GPS position error is greater than this (m)
        float velDriftTimeLim; //The maximum time without observations to constrain velocity drift before a zero velocity is fused to prevent the filter diverging (sec)
        float gpsOffTime; //GPS aiding will be turned off at this time (sec)
        float gpsOnTime; //GPS aiding will be turned back on at this time (sec)
        float visoOffTime; //visual odometry aiding will be turned off at this time (sec)
        float visoOnTime; //visual odometry aiding will be turned back on at this time (sec)
        float rollAlignErr; //initial roll misalignment (rad)
        float pitchAlignErr; //initial pitch misalignment (rad)
        float yawAlignErr; //initial yaw misalignment (rad)
        bool imuInitStates;
        bool visualInitStates;
        bool laserInitStates;
        bool covInit;
        void reset_param_control(){
            waitForGps = 0; //set to 1 if the filter start should be delayed until GPS checks to pass
            gpsSpdErrLim = 1.0; //GPS use will not start if reported GPS speed error is greater than this (m/s)
            gpsPosErrLim = 5.0; //GPS use will not start if reported GPS position error is greater than this (m)
            velDriftTimeLim = 5.0; //The maximum time without observations to constrain velocity drift before a zero velocity is fused to prevent the filter diverging (sec)
            gpsOffTime = 0; //GPS aiding will be turned off at this time (sec)
            gpsOnTime = 0; //GPS aiding will be turned back on at this time (sec)
            visoOffTime = 0; //visual odometry aiding will be turned off at this time (sec)
            visoOnTime = 0; //visual odometry aiding will be turned back on at this time (sec)
            rollAlignErr = 0.0; //initial roll misalignment (rad)
            pitchAlignErr = 0.0; //initial pitch misalignment (rad)
            yawAlignErr = 0.0; 
            imuInitStates = false;
            visualInitStates = false;
            laserInitStates = true;
            covInit = false;
        }
    };
    //fusion
    struct param_fusion {
        param_fusion(){
            reset_param_fusion();
        }
        //gps fusion
        float gpsTimeDelay; //GPS measurement delay relative to IMU (sec)
        float gpsVelGate; //Size of the IMU velocity innovation consistency check gate in SD
        float gpsPosGate; //Size of the IMU velocity innovation consistency check gate in SD
        float gpsCheckTimeout; //Length of time that GPS measurements will be rejected by the filter before states are reset to the GPS velocity. (sec)
        //baro fusion
        float baroTimeDelay; //Baro measurement delay relative to IMU (sec)
        float baroHgtGate; //Size of the IMU velocity innovation consistency check gate in SD
        float baroHgtNoise; //1SD observation noise of the baro measurements (m)
        //Magnetometer measurement fusion
        float magTimeDelay; //Magnetomer time delay relative to IMU (sec)
        float magFuseMethod; //0: 3-Axis field fusion with free declination, 1: 3-Axis field fusion with constrained declination, 2: magnetic heading fusion. (None)
        float magFieldError; //Magnetic field measurement 1SD error including hard and soft iron interference. Used when magFuseMethod = 0 or 1. (gauss)
        float magHdgError; //Magnetic heading measurement 1SD error including hard and soft iron interference. Used when magFuseMethod = 2. (rad)
        float magFieldGate; //Size of the magnetic field innovation consistency check gate in SD
        float magHdgGate; //Size of the magnetic heading innovation consistency check gate in SD
        float magDeclDeg; //Magnetic declination in deg
        Eigen::Vector3d magBias_; // calibrate mag get
        Eigen::Vector3d magScale_;
        //visual odometry body frame velocity measurement fusion
        float bodyVelTimeDelay; //sensor time delay relative to IMU (sec)
        float bodyVelErrorMin; //Observation noise 1SD for the odometry sensor at the highest quality value (m/sec)
        float bodyVelErrorMax; //Observation noise 1SD for the odometry sensor at the lowest quality value (m/sec)
        float bodyVelGate; //Size of the optical flow rate innovation consistency check gate in SD
        float worldPosErrorMin;
        float worldPosErrorMax;
        float worldPosGate;
        void reset_param_fusion(){
            gpsTimeDelay = 0.1; //GPS measurement delay relative to IMU (sec)
            gpsVelGate = 5.0; //Size of the IMU velocity innovation consistency check gate in SD
            gpsPosGate = 5.0; //Size of the IMU velocity innovation consistency check gate in SD
            gpsCheckTimeout = 10.0; //Length of time that GPS measurements will be rejected by the filter before states are reset to the GPS velocity. (sec)
            //baro fusion
            baroTimeDelay = 0.05; //Baro measurement delay relative to IMU (sec)
            baroHgtGate = 5.0; //Size of the IMU velocity innovation consistency check gate in SD
            baroHgtNoise = 2.0; //1SD observation noise of the baro measurements (m)
            //Magnetometer measurement fusion
            magTimeDelay = 0.0; //Magnetomer time delay relative to IMU (sec)
            magFuseMethod = 1; //0: 3-Axis field fusion with free declination, 1: 3-Axis field fusion with constrained declination, 2: magnetic heading fusion. (None)
            magFieldError = 0.05; //Magnetic field measurement 1SD error including hard and soft iron interference. Used when magFuseMethod = 0 or 1. (gauss)
            magHdgError = 0.1745; //Magnetic heading measurement 1SD error including hard and soft iron interference. Used when magFuseMethod = 2. (rad)
            magFieldGate = 5.0; //Size of the magnetic field innovation consistency check gate in SD
            magHdgGate = 5.0; //Size of the magnetic heading innovation consistency check gate in SD
            magDeclDeg = 10.5; //Magnetic declination in deg
            //visual odometry body frame velocity measurement fusion
            bodyVelTimeDelay = 0.01; //sensor time delay relative to IMU (sec)
            bodyVelErrorMin = 0.1; //Observation noise 1SD for the odometry sensor at the highest quality value (m/sec)
            bodyVelErrorMax = 0.9; //Observation noise 1SD for the odometry sensor at the lowest quality value (m/sec)
            bodyVelGate = 5.0;
            worldPosErrorMin = 0.1;
            worldPosErrorMax = 0.9;
            worldPosGate = 5.0;
            magBias_ << 0, 0, 0;
            magScale_ << 1, 1, 1;
        }
    };


    //state prediction error growth
    struct param_prediction {
        param_prediction(){
            reset_param_prediction();
        }
        float magPnoiseNED; //Earth magnetic field 1SD rate of change. (gauss/sec)
        float magPnoiseXYZ; //Body magnetic field 1SD rate of change. (gauss/sec)
        float dAngBiasPnoise; //IMU gyro bias 1SD rate of change (rad/sec^2)
        float dVelBiasPnoise; //IMU accel bias 1SD rate of change (m/sec^3)
        float angRateNoise; //IMU gyro 1SD rate process noise (rad/sec)
        float accelNoise; //IMU accelerometer 1SD error noise including switch on bias uncertainty. (m/sec^2)
        bool prevDelAngSet;
        bool prevDelVelSet;
        void reset_param_prediction(){
            magPnoiseNED = 1e-3; //Earth magnetic field 1SD rate of change. (gauss/sec)
            magPnoiseXYZ = 1e-3; //Body magnetic field 1SD rate of change. (gauss/sec)
            dAngBiasPnoise = 0.001; //IMU gyro bias 1SD rate of change (rad/sec^2)
            dVelBiasPnoise = 0.03; //IMU accel bias 1SD rate of change (m/sec^3)
            angRateNoise = 0.015; //IMU gyro 1SD rate process noise (rad/sec)
            accelNoise = 0.35; //IMU accelerometer 1SD error noise including switch on bias uncertainty. (m/sec^2)
            prevDelAngSet = false;
            prevDelVelSet = false;
        }
    };

    //initial uncertainty
    struct param_alignment {
        param_alignment(){
            reset_param_alignment();
        }
        Eigen::Matrix3d CamToBody;
        float posErrNE; //Initial 1SD position error when aligning without GPS. (m/sec)
        float velErrNE; //Initial 1SD velocity error when aligning without GPS. (m/sec)
        float velErrD; //Initial 1SD vertical velocity error when aligning without GPS. (m/sec)
        float delAngBiasErr; //Initial 1SD rate gyro bias uncertainty. (rad/sec)
        float delVelBiasErr; //Initial 1SD accelerometer bias uncertainty. (m/sec^2)
        float quatErr; //Initial 1SD uncertainty in quaternion.
        float magErrXYZ; //Initial 1SD uncertainty in body frame XYZ magnetic field states. (gauss)
        float magErrNED; //Initial 1SD uncertainty in earth frame NED magnetic field states. (gauss)
        float hgtErr; //Initial 1SD uncertainty in height. (m)
        float windErrNE; //Initial 1SD error in wind states. (m/sec)
        void reset_param_alignment(){
            CamToBody = Eigen::Matrix3d::Identity();
            posErrNE = 10.0; //Initial 1SD position error when aligning without GPS. (m/sec)
            velErrNE = 5.0; //Initial 1SD velocity error when aligning without GPS. (m/sec)
            velErrD = 1.0; //Initial 1SD vertical velocity error when aligning without GPS. (m/sec)
            delAngBiasErr = 0.05*M_PI/180; //Initial 1SD rate gyro bias uncertainty. (rad/sec)
            delVelBiasErr = 0.07; //Initial 1SD accelerometer bias uncertainty. (m/sec^2)
            quatErr = 0.1; //Initial 1SD uncertainty in quaternion.
            magErrXYZ = 0.01; //Initial 1SD uncertainty in body frame XYZ magnetic field states. (gauss)
            magErrNED = 0.5; //Initial 1SD uncertainty in earth frame NED magnetic field states. (gauss)
            hgtErr = 0.5; //Initial 1SD uncertainty in height. (m)
            windErrNE = 5.0; //Initial 1SD error in wind states. (m/sec)
        }
    };
} // namespace param

#endif //PARAM_LIST_H_
