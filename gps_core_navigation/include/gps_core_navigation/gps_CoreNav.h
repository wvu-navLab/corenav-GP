/*
 * Author: Cagri
 */

#ifndef gps_core_navigation_H
#define gps_core_navigation_H

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <fstream>
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JointState.h>
#include <hw_interface_plugin_roboteq/RoboteqData.h>
#include <tf/transform_broadcaster.h>
#include <geometry_utils/Transform3.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Twist.h>
#include <message_filters/time_synchronizer.h>
#include <gps_core_navigation/InsConst.h>
#include <gps_core_navigation/InsUtils.h>
#include <tf2/LinearMath/Quaternion.h>
// #include <gps_core_nav/GP_Input.h>
// #include <gps_core_nav/GP_Output.h>
#include <std_msgs/Float64.h>
#include <novatel_gps_msgs/NovatelXYZ.h>
#include <novatel_gps_msgs/NovatelPosition.h>
class gps_CoreNav {
public:

        typedef sensor_msgs::Imu ImuData;
        typedef nav_msgs::Odometry OdoData;
        typedef sensor_msgs::JointState JointData;
        // typedef geometry_msgs::Twist CmdData;
        typedef hw_interface_plugin_roboteq::RoboteqData EncoderLeftData;
        typedef hw_interface_plugin_roboteq::RoboteqData EncoderRightData;

        typedef novatel_gps_msgs::NovatelXYZ GPSECEFData;
        typedef novatel_gps_msgs::NovatelPosition GPSLLHData;

        typedef geometry_msgs::PoseStamped PoseData;
        typedef Eigen::VectorXd Vector;
        typedef Eigen::MatrixXd Matrix;
        typedef Eigen::Matrix<double, 2, 1> Vector2;
        typedef Eigen::Matrix<double, 3, 1> Vector3;
        typedef Eigen::Matrix<double, 4, 1> Vector4;
        typedef Eigen::Matrix<double, 6, 1> Vector6;
        typedef Eigen::Matrix<double, 9, 1> Vector9;
        typedef Eigen::Matrix<double, 13, 1> Vector13;
        typedef Eigen::Matrix<double, 15, 1> Vector15;

        typedef Eigen::Matrix<double, 3, 3> Matrix3;
        Matrix3 CbnMinus;
        Matrix3 eye3=Eigen::Matrix3d::Identity();
        Matrix3 zeros3=Eigen::Matrix3d::Zero(3,3);

        gps_CoreNav::Vector13 odo;
        gps_CoreNav::Vector6 imu;
        gps_CoreNav::Vector4 joint;
        gps_CoreNav::Vector3 cmd;
        gps_CoreNav::Vector2 encoderLeft;
        gps_CoreNav::Vector2 encoderRight;
        gps_CoreNav::Vector6 gps_ecef;
        gps_CoreNav::Vector3 gps_llh;
        gps_CoreNav();
        ~gps_CoreNav();

// Calls LoadParameters and RegisterCallbacks. Fails on failure of either.
        bool Initialize(const ros::NodeHandle& n);

// private:
// Node initialization
        bool Init(const ros::NodeHandle& n);
        bool LoadParameters(const ros::NodeHandle& n);
        bool RegisterCallbacks(const ros::NodeHandle& n);

// Publish estimated  states.
        void PublishStates(const gps_CoreNav::Vector3& states, const ros::Publisher& pub);
        void PublishStatesSlip(const gps_CoreNav::Vector3& slip_states, const ros::Publisher& slip_pub);
        void PublishStatesCN(const gps_CoreNav::Vector9& cn_states, const ros::Publisher& cn_pub_);
        void ImuCallback(const ImuData& imu_data_);
        void OdoCallback(const OdoData& odo_data_);
        void JointCallBack(const JointData& joint_data_);
        // void CmdCallBack(const CmdData& cmd_data_);
        void EncoderLeftCallBack(const EncoderLeftData& encoderLeft_data_);
        void EncoderRightCallBack(const EncoderRightData& encoderRight_data_);

        void GPSECEFCallBack(const GPSECEFData& gps_ecef_data_);
        void GPSLLHCallBack(const GPSLLHData& gps_llh_data_);
        // void GPCallBack(const gps_core_nav::GP_Output::ConstPtr& gp_data_in_);

        void Update(const gps_CoreNav::Vector13& odo,const gps_CoreNav::Vector4& joint);
        void Propagate(const gps_CoreNav::Vector6& imu, const gps_CoreNav::Vector13& odo, const gps_CoreNav::Vector3& cmd,const gps_CoreNav::Vector2& encoderLeft,const gps_CoreNav::Vector2& encoderRight,const gps_CoreNav::Vector4& joint, const gps_CoreNav::Vector6& gps_ecef, const gps_CoreNav::Vector3& gps_llh);
//NonHolonomic constraint
        void NonHolonomic(const gps_CoreNav::Vector3 vel, const gps_CoreNav::Vector3 att, const gps_CoreNav::Vector3 llh, gps_CoreNav::Vector15 errorStates, Eigen::MatrixXd P, gps_CoreNav::Vector3 omega_b_ib);
//Zero vel update
        void zupt(const gps_CoreNav::Vector3 vel, const gps_CoreNav::Vector3 att, const gps_CoreNav::Vector3 llh, gps_CoreNav::Vector15 errorStates, Eigen::MatrixXd P);
// Zero ang. update
        void zaru(const gps_CoreNav::Vector3 vel, const gps_CoreNav::Vector3 att, const gps_CoreNav::Vector3 llh, gps_CoreNav::Vector15 errorStates, Eigen::MatrixXd P, const gps_CoreNav::Vector3 omega_b_ib);

        void gps_init(const gps_CoreNav::Vector3 vel, const gps_CoreNav::Vector3 att, const gps_CoreNav::Vector3 llh, gps_CoreNav::Vector15 errorStates, Eigen::MatrixXd P, const gps_CoreNav::Vector3 omega_b_ib, gps_CoreNav::Vector3 gps_llh, gps_CoreNav::Vector6 gps_ecef);

        void writeParams(std::string path_to_param_file, const gps_CoreNav::Vector3& ins_bias_a, const gps_CoreNav::Vector3& ins_bias_g, const gps_CoreNav::Vector3& ins_xyz_, const gps_CoreNav::Vector3& ins_pos_, const gps_CoreNav::Vector3& ins_att_);
        // void writeParams(std::string path_to_param_file, const InitCoreNav::Vector3& ins_bias_a, const InitCoreNav::Vector3& ins_bias_g, const InitCoreNav::Vector3& init_ecef, const InitCoreNav::Vector3& init_llh);

        gps_CoreNav::Vector3 calc_gravity(const double latitude, const double height);
        gps_CoreNav::Matrix3 skew_symm(const gps_CoreNav::Vector3 vec);
        gps_CoreNav::Matrix3 eul_to_dcm(double phi, double theta, double psi);
        gps_CoreNav::Vector3 dcm_to_eul(gps_CoreNav::Matrix3 dcm);
        gps_CoreNav::Vector3 llh_to_enu(const double latitude, const double longitude, const double height);
        gps_CoreNav::Vector3 llh_to_xyz(const double latitude, const double longitude, const double height);

        gps_CoreNav::Matrix insErrorStateModel_LNF(double R_EPlus, double R_N, gps_CoreNav::Vector3 insLLHPlus, gps_CoreNav::Vector3 insVelPlus, double dt,gps_CoreNav::Matrix3 CbnPlus, double omega_ie, gps_CoreNav::Vector3 omega_n_in,Vector3 f_ib_b,double gravity);
        gps_CoreNav::Matrix calc_Q(double R_N, double R_E, gps_CoreNav::Vector3 insLLHPlus, double dt, gps_CoreNav::Matrix3 CbnPlus, gps_CoreNav::Vector3 f_ib_b);

        gps_CoreNav::Vector6 getImuData(const ImuData& imu_data_);
        gps_CoreNav::Vector4 getJointData(const JointData &joint_data_);
        // gps_CoreNav::Vector getCmdData(const CmdData &cmd_data_);
        gps_CoreNav::Vector2 getEncoderLeftData(const EncoderLeftData &encoderLeft_data_);
        gps_CoreNav::Vector2 getEncoderRightData(const EncoderRightData &encoderRight_data_);
        gps_CoreNav::Vector13 getOdoData(const OdoData& odo_data_);
        gps_CoreNav::Vector6 getGPSECEFData(const GPSECEFData& gps_ecef_data_);
        gps_CoreNav::Vector3 getGPSLLHData(const GPSLLHData& gps_llh_data_);


// The node's name.
        std::string name_;

// Subscriber
        ros::Subscriber imu_sub_;
        ros::Subscriber odo_sub_;
        ros::Subscriber joint_sub_;
        ros::Subscriber cmd_sub_;
        ros::Subscriber encoderLeft_sub_;
        ros::Subscriber encoderRight_sub_;
        ros::Subscriber gps_ecef_sub_;
        ros::Subscriber gps_llh_sub_;
        // ros::Subscriber gp_sub_;
        // bool new_gp_data_arrived_;
        // double gp_arrived_time_;
// Publisher.
        ros::Publisher position_pub_, velocity_pub_, attitude_pub_, enu_pub_,cn_pub_, slip_pub_, /*gp_pub,*/ stop_cmd_pub_, attitude_cov_pub_,attitude_cov_pub_p, velocity_cov_pub_,velocity_cov_pub_p, position_cov_pub_,position_cov_pub_p,pos_llh_pub_,ins_xyz_pub_;
        tf::TransformBroadcaster transformed_states_tf_broad;

        OdoData odo_data_;
        OdoData odo_data_prev_;
        ImuData imu_data_;
        JointData joint_data_;
        // CmdData cmd_data_;
        EncoderLeftData encoderLeft_data_;
        EncoderRightData encoderRight_data_;
        GPSECEFData gps_ecef_data_;
        GPSLLHData gps_llh_data_;
        std_msgs::Float64 stop_cmd_msg_;
        // gps_core_nav::GP_Input slip_msg;
        // gps_core_nav::GP_Output gp_data_;
        bool has_odo_ = false;
        bool has_joint_ = false;
        bool has_cmd_ = false;
        bool has_encoderLeft_ = false;
        bool has_encoderRight_ = false;
        bool has_gpsLLH_=false;
        bool has_gpsECEF_=false;
        bool has_imu_ = false;
        bool first_odo_ = true;
        bool first_joint_ = true;
        bool first_imu_ = true;
        bool flag = true;
        bool propagate_flag = false;
        bool update_flag = false;
        // bool gp_flag = false;
        // bool started_driving_again_flag = true;

// Most recent time stamp for publishers.
        ros::Time stamp_;

// Coordinate frames.
        std::string frame_id_out_;
        std::string frame_id_imu_;
        std::string frame_id_odo_;
        std::string frame_id_fixed_;

// update rate [hz]
        unsigned int publish_hz_;
        unsigned int sensor_pub_rate_;

// sub. topics
        std::string imu_topic_;
        std::string odo_topic_;
        std::string joint_topic_;
        std::string cmd_topic_;
        std::string encoderLeft_topic_;
        std::string encoderRight_topic_;
        std::string gps_llh_topic_;
        std::string gps_ecef_topic_;
        // std::string gp_topic_; // CAGRI: this is fine, you just need to set it with a param, like you do with the rest of these. It isn't being set to anything yet in the .cpp file
// For initialization.
        bool initialized_;

// Filter vars.
        int num_states_ = 15;
        gps_CoreNav::Vector15 error_states_; // {pos., vel, att, ba, bg}
        gps_CoreNav::Vector3 ba_;
        gps_CoreNav::Vector3 bg_;
        gps_CoreNav::Vector3 ins_att_, ins_vel_, ins_pos_, ins_enu_,slip_cn_,savePos, ins_enu_slip, ins_enu_slip3p, ins_enu_slip_3p,ins_att_cov_,ins_att_cov_p, ins_vel_cov_,ins_vel_cov_p,ins_pos_cov_,ins_pos_cov_p, ins_xyz_,ins_bias_a,ins_bias_g,ins_pos_llh_;
        gps_CoreNav::Vector4 Z_;
        gps_CoreNav::Vector9 ins_cn_;
        gps_CoreNav::Matrix P_, Q_, STM_, P_pred, P;
        Eigen::Matrix<double, 4, 4> R_;
        Eigen::Matrix<double, 3, 3> R_zupt;
        Eigen::Matrix<double, 3, 3> R_zaru;
        Eigen::Matrix<double, 4, 4> R_IP;
        // Eigen::Matrix<double, 4, 4> R_IP_1;
        // Eigen::Matrix<double, 4, 4> R_IP_2;
        Eigen::Matrix<double, 2, 2> R_holo;
        Eigen::Matrix<double, 15, 4> K_;
        Eigen::Matrix<double, 15, 4> K_pred;
        Eigen::Matrix<double, 15, 3> K_zupt;
        Eigen::Matrix<double, 15, 3> K_zaru;
        Eigen::Matrix<double, 15, 2> K_holo;
        Eigen::Matrix<double, 15, 6> K_gps;
        Eigen::Matrix<double, 4, 15> H_;
        Eigen::Matrix<double, 3, 15> H_zupt;
        Eigen::Matrix<double, 3, 15> H_zaru;
        Eigen::Matrix<double, 2, 15> H_holo;
        Eigen::Matrix<double, 2, 1> z_holo;
        Eigen::Matrix<double, 6, 15> H_gps;
        Eigen::Matrix<double, 6,6> R_gps;
        Eigen::Matrix<double, 3,3> S_p;

        gps_CoreNav::Vector3 H11_, H12_, H21_, H31_, H32_, H24_, H41_, H42_;
        double z11_, z21_, z31_, z41_;
        double rearVel_, headRate_, T_r_, s_or_, s_delta_or_,velFrontLeft_,velFrontRight_,velBackLeft_,velBackRight_;

        gps_CoreNav::Vector3 omega_b_ib_, omega_b_ib_prev_, omega_n_ie_;
        gps_CoreNav::Vector3 f_ib_b_, f_ib_b_prev_, omega_n_en_, omega_n_in_, grav_;
        gps_CoreNav::Matrix3 Omega_b_ib_, Omega_n_ie_, Omega_n_en_;

// imu noise params
        double sig_gyro_inRun_, sig_ARW_,  sig_accel_inRun_, sig_VRW_;
// filter noise params
        double position_noise_, attitude_noise_, velocity_noise_, bias_noise_;

// initial pose
        double init_x, init_y, init_z, init_vx, init_vy, init_vz, psiEst;
        double init_roll, init_pitch, init_yaw, sigma_x, sigma_y;
        double init_cov_x,init_cov_y,init_cov_z;
        double init_cov_vx,init_cov_vy,init_cov_vz;
        double init_cov_roll, init_cov_pitch, init_cov_yaw;
        double init_ecef_x,init_ecef_y,init_ecef_z;
        double init_ba_x, init_ba_y, init_ba_z, init_bg_x, init_bg_y, init_bg_z;
        double init_bias_a_x, init_bias_a_y, init_bias_a_z, init_bias_g_x, init_bias_g_y, init_bias_g_z;
        double sigma_z, sigma_vx, sigma_vy, prev_stamp_, up_time_;
        double sigma_vz, sigma_roll, sigma_pitch, sigma_yaw;
        double imu_stamp_curr_, imu_stamp_prev_, odo_stamp_curr_, odo_stamp_prev_,latitude_prev,latitude_now,longitude_prev,longitude_now;
        double joint_stamp_curr_, joint_stamp_prev_;
        double dt_odo_, dt_imu_, dt_lat,dt_lon,yawx,yawy,yawGPS;
        int count=0;
        int slip_i, i;
        double xy_errSlip;
        double odomUptCount, /*startRecording, stopRecording,*/ saveCountOdom;

};
#endif
