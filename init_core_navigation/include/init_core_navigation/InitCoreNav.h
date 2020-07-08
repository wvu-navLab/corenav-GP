#ifndef init_core_navigation_H
#define init_core_navigation_H

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <fstream>
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>
#include <geometry_utils/Transform3.h>
#include <geometry_msgs/PointStamped.h>
#include <message_filters/time_synchronizer.h>
#include <novatel_gps_msgs/NovatelXYZ.h>
#include <novatel_gps_msgs/NovatelPosition.h>

class InitCoreNav {
public:

        typedef sensor_msgs::Imu ImuData;
        typedef geometry_msgs::PoseStamped PoseData;

        typedef Eigen::VectorXd Vector;
        typedef Eigen::MatrixXd Matrix;
        typedef Eigen::Matrix<double, 3, 1> Vector3;
        typedef Eigen::Matrix<double, 6, 1> Vector6;
        typedef novatel_gps_msgs::NovatelXYZ GPSECEFData;
        typedef novatel_gps_msgs::NovatelPosition GPSLLHData;

        InitCoreNav::Vector6 imu;
        InitCoreNav::Vector3 gps_ecef;
        InitCoreNav::Vector3 gps_llh;

        InitCoreNav();
        ~InitCoreNav();

// Calls LoadParameters and RegisterCallbacks. Fails on failure of either.
        bool Initialize(const ros::NodeHandle& n);

private:
// Node initialization
        bool Init(const ros::NodeHandle& n);
        bool LoadParameters(const ros::NodeHandle& n);
        bool RegisterCallbacks(const ros::NodeHandle& n);

// Publish estimated  states.
        void PublishStates(const InitCoreNav::Vector3& states, const ros::Publisher& pub);
        void writeParams(std::string path_to_param_file, const InitCoreNav::Vector3& ins_bias_a, const InitCoreNav::Vector3& ins_bias_g, const InitCoreNav::Vector3& init_ecef, const InitCoreNav::Vector3& init_llh);
        void ImuCallback(const ImuData& imu_data_);
        void Propagate(const InitCoreNav::Vector6& imu,const InitCoreNav::Vector3& gps_ecef, const InitCoreNav::Vector3& gps_llh);
        void GPSLLHCallBack(const GPSLLHData& gps_llh_data_);
        void GPSECEFCallBack(const GPSECEFData& gps_ecef_data_);


        InitCoreNav::Vector6 getImuData(const ImuData& imu_data_);
        InitCoreNav::Vector3 getGPSECEFData(const GPSECEFData& gps_ecef_data_);
        InitCoreNav::Vector3 getGPSLLHData(const GPSLLHData& gps_llh_data_);


// The node's name.
        std::string name_;

// Subscriber
        ros::Subscriber imu_sub_;
        ros::Subscriber gps_ecef_sub_;
        ros::Subscriber gps_llh_sub_;

// Publisher.
        ros::Publisher bias_a_pub_;
        ros::Publisher bias_g_pub_, init_llh_pub_, init_ecef_pub_;
        tf::TransformBroadcaster transformed_states_tf_broad;

        GPSECEFData gps_ecef_data_;
        GPSLLHData gps_llh_data_;
        ImuData imu_data_;

        bool has_imu_ = false;
        bool first_imu_ = true;
        bool has_gpsLLH_=false;
        bool has_gpsECEF_=false;

// Most recent time stamp for publishers.
        ros::Time stamp_;

// Coordinate frames.
        std::string frame_id_out_;
        std::string frame_id_imu_;
        std::string frame_id_fixed_;

// update rate [hz]
        unsigned int publish_hz_;
        unsigned int sensor_pub_rate_;

// sub. topics
        std::string imu_topic_;
        std::string gps_llh_topic_;
        std::string gps_ecef_topic_;
// For initialization.
        bool initialized_;

// Filter vars.
        InitCoreNav::Vector3 bias_a_;
        InitCoreNav::Vector3 bias_g_;
        InitCoreNav::Vector3 ins_bias_a;
        InitCoreNav::Vector3 ins_bias_g;
        InitCoreNav::Vector3 init_llh;
        InitCoreNav::Vector3 init_ecef;

        InitCoreNav::Vector3 omega_b_ib_;
        InitCoreNav::Vector3 f_ib_b_;

// initial pose

        double init_ba_x, init_ba_y, init_ba_z, init_bg_x, init_bg_y, init_bg_z;
        double init_ecef_x,init_ecef_y,init_ecef_z;
        double init_x, init_y, init_z;
        double imu_stamp_curr_, imu_stamp_prev_;
        double dt_imu_;
        int count=0;

};
#endif
