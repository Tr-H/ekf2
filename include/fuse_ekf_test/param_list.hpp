#ifndef PARAM_LIST_H_
#define PARAM_LIST_H_

#define _USE_MATH_DEFINES
#include <math.h>
#include <Eigen/Dense>

#define gravity 9.80665 // 9.80665
// Bit locations for fusion_mode
#define MASK_USE_GPS    (1<<0)		///< set to true to use GPS data
#define MASK_USE_OF     (1<<1)		///< set to true to use optical flow data
#define MASK_INHIBIT_ACC_BIAS (1<<2)	///< set to true to inhibit estimation of accelerometer delta velocity bias
#define MASK_USE_EVPOS	(1<<3)		///< set to true to use external vision NED position data
#define MASK_USE_EVYAW  (1<<4)		///< set to true to use exernal vision quaternion data for yaw
#define MASK_USE_DRAG  (1<<5)		///< set to true to use the multi-rotor drag model to estimate wind
#define MASK_ROTATE_EV  (1<<6)		///< set to true to if the EV observations are in a non NED reference frame and need to be rotated before being used


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
        bool SinglePointFusion;
        bool zedHeightFusion; 
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
            SinglePointFusion = false;
            zedHeightFusion = true;
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
        float zedVelAcc;
        float posObsNoiseNE;
        float posInnovGateNE;
        float posErr;
        float ev_innov_gate;
        float worldPosErrorMin;
        float worldPosErrorMax;
        float worldPosGate;
    	int32_t fusion_mode;		///< bitmasked integer that selects which aiding sources will be used

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

            zedVelAcc = 0.01f; // zed vel accuracy in m/s -han
            posObsNoiseNE = 0.0f; ///< 1-STD observtion noise used for the fusion of NE position data (m)
            posInnovGateNE = 0.0f; ///< Number of standard deviations used for the NE position fusion innovation consistency check
            posErr = 0.0f;
            ev_innov_gate = 5.0f;

            worldPosErrorMin = 0.1;
            worldPosErrorMax = 0.9;
            worldPosGate = 5.0;
            magBias_ << 0, 0, 0;
            magScale_ << 1, 1, 1;
            fusion_mode = MASK_USE_EVPOS;
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
        float windVelPnoise;
        float angRateNoise; //IMU gyro 1SD rate process noise (rad/sec)
        float accelNoise; //IMU accelerometer 1SD error noise including switch on bias uncertainty. (m/sec^2)
        bool prevDelAngSet;
        bool prevDelVelSet;
      	float acc_bias_lim;		///< maximum accel bias magnitude (m/sec**2)
    	float acc_bias_learn_acc_lim;	///< learning is disabled if the magnitude of the IMU acceleration vector is greater than this (m/sec**2)
    	float acc_bias_learn_gyr_lim;	///< learning is disabled if the magnitude of the IMU angular rate vector is greater than this (rad/sec)
    	float acc_bias_learn_tc;		///< time constant used to control the decaying envelope filters applied to the accel and gyro magnitudes (sec)
        uint32_t mag_3D;
        uint32_t wind;
        float initial_wind_uncertainty;


        void reset_param_prediction(){
            magPnoiseNED = 1e-3; //Earth magnetic field 1SD rate of change. (gauss/sec)
            magPnoiseXYZ = 1e-4; //Body magnetic field 1SD rate of change. (gauss/sec)
            dAngBiasPnoise = 0.001; //IMU gyro bias 1SD rate of change (rad/sec^2)
            dVelBiasPnoise = 0.006; //IMU accel bias 1SD rate of change (m/sec^3)
            windVelPnoise = 1e-1;
            angRateNoise = 0.015; //IMU gyro 1SD rate process noise (rad/sec)
            accelNoise = 0.35; //IMU accelerometer 1SD error noise including switch on bias uncertainty. (m/sec^2)
            prevDelAngSet = false;
            prevDelVelSet = false;
            acc_bias_lim = 0.4f;	
            acc_bias_learn_acc_lim = 25.0f;	
            acc_bias_learn_gyr_lim = 3.0f;	
            acc_bias_learn_tc = 0.5f;	
            mag_3D = 1; 
            wind = 1;
            initial_wind_uncertainty = 1.0f;
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
        float zedPosNoise; // according to GPS
        float zedVelNoise;
        float delAngBiasErr; //Initial 1SD rate gyro bias uncertainty. (rad/sec)
        float delVelBiasErr; //Initial 1SD accelerometer bias uncertainty. (m/sec^2)
        float mag_noise;
        float quatErr; //Initial 1SD uncertainty in quaternion.
        float magErrXYZ; //Initial 1SD uncertainty in body frame XYZ magnetic field states. (gauss)
        float magErrNED; //Initial 1SD uncertainty in earth frame NED magnetic field states. (gauss)
        float hgtErr; //Initial 1SD uncertainty in height. (m)
        float singlePointNoise;
        float switch_on_gyro_bias;
        float switch_on_accel_bias;
        float windErrNE; //Initial 1SD error in wind states. (m/sec)
        void reset_param_alignment(){
            //CamToBody = Eigen::Matrix3d::Identity();
            CamToBody << 0.0, -1.0, 0.0,
                         1.0, 0.0, 0.0,
                         0.0, 0.0, 1.0;
            //CamToBody << 0.0148655429818, -0.999880929698, 0.00414029679422, 0.999557249008, 0.0149672133247, 0.025715529948, -0.0257744366974, 0.00375618835797, 0.999660727178;
            posErrNE = 10.0f; //Initial 1SD position error when aligning without GPS. (m/sec)
            velErrNE = 5.0f; //Initial 1SD velocity error when aligning without GPS. (m/sec)
            velErrD = 1.0f; //Initial 1SD vertical velocity error when aligning without GPS. (m/sec)
            zedPosNoise = 0.01f; // 0.5
            zedVelNoise = 0.01f; // 0.5
            delAngBiasErr = 0.05f * M_PI/180; //Initial 1SD rate gyro bias uncertainty. (rad/sec)
            delVelBiasErr = 0.07f; //Initial 1SD accelerometer bias uncertainty. (m/sec^2)
            mag_noise = 5.0e-2f;
            quatErr = 0.1f; //Initial 1SD uncertainty in quaternion.
            magErrXYZ = 0.01f; //Initial 1SD uncertainty in body frame XYZ magnetic field states. (gauss)
            magErrNED = 0.5f; //Initial 1SD uncertainty in earth frame NED magnetic field states. (gauss)
            hgtErr = 0.5f; //Initial 1SD uncertainty in height. (m)
            singlePointNoise = 0.1f;
            switch_on_gyro_bias = 0.1f;
            switch_on_accel_bias = 0.2f;
            windErrNE = 1.0f; //Initial 1SD error in wind states. (m/sec)
        }
    };
} // namespace param

#endif //PARAM_LIST_H_
