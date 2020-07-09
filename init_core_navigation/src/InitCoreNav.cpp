#include <init_core_navigation/InitCoreNav.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_utils/GeometryUtilsROS.h>
#include <parameter_utils/ParameterUtils.h>
#include <geometry_msgs/TransformStamped.h>
#include <novatel_gps_msgs/NovatelXYZ.h>
#include <novatel_gps_msgs/NovatelPosition.h>


namespace pu = parameter_utils;
namespace gu = geometry_utils;
namespace gr = gu::ros;

InitCoreNav::InitCoreNav() : initialized_(false){
}

InitCoreNav::~InitCoreNav(){
}

bool InitCoreNav::Initialize(const ros::NodeHandle& n){
        name_ = ros::names::append(n.getNamespace(), "InitCoreNav");

        if(!LoadParameters(n)) {
                ROS_ERROR("%s: Failed to load parameters.", name_.c_str());
                return false;
        }

        if(!RegisterCallbacks(n)) {
                ROS_ERROR("%s: Failed to register callbacks.", name_.c_str());
                return false;
        }

        return true;
}


bool InitCoreNav::LoadParameters(const ros::NodeHandle& n){
        // Load frame ids.
        if(!pu::Get("frames/frame_id_out", frame_id_out_)) return false;
        if(!pu::Get("frames/frame_id_imu", frame_id_imu_)) return false;
        if(!pu::Get("frames/frame_id_fixed", frame_id_fixed_)) return false;

        // Load topics
        if(!pu::Get("imu/topic", imu_topic_)) return false;
        if(!pu::Get("gps_llh/topic", gps_llh_topic_)) return false;
        if(!pu::Get("gps_ecef/topic", gps_ecef_topic_)) return false;

        // Load update rate
        if(!pu::Get("imu/publish_hz", publish_hz_)) return false;
        if(!pu::Get("imu/sensor_pub_rate", sensor_pub_rate_)) return false;


        return true;
}


void InitCoreNav::ImuCallback(const ImuData& imu_dataAdis_){
        imu = getImuData(imu_dataAdis_);
        has_imu_ = true;
        if (first_imu_)
        {
                imu_stamp_prev_ = (imu_dataAdis_.header.stamp).toSec();
                first_imu_ = false;
        }
        else{
                imu_stamp_curr_ = (imu_dataAdis_.header.stamp).toSec();
                if (count<=200) {
                  Propagate(imu, gps_ecef, gps_llh);
                  imu_stamp_prev_  = imu_stamp_curr_;
                } else {

                  if (ros::ok()) {
                    ros::shutdown();
                  }

                }

        }
        return;
}

void InitCoreNav::GPSLLHCallBack(const GPSLLHData& gps_llh_data_){
  gps_llh =getGPSLLHData(gps_llh_data_);
  // std::cout << "llh" <<gps_llh <<'\n';
  has_gpsLLH_ = true;
  return;
}

void InitCoreNav::GPSECEFCallBack(const GPSECEFData& gps_ecef_data_){
  gps_ecef =getGPSECEFData(gps_ecef_data_);
  // std::cout << "ecef" <<gps_ecef <<'\n';
  has_gpsECEF_ = true;
  return;
}

bool InitCoreNav::RegisterCallbacks(const ros::NodeHandle& n){
        // Create a local nodehandle to manage callback subscriptions.
        ros::NodeHandle nl(n);

        bias_a_pub_ = nl.advertise<geometry_msgs::PointStamped>( "bias_a", 10, false);
        bias_g_pub_ = nl.advertise<geometry_msgs::PointStamped>( "bias_g", 10, false);

        init_ecef_pub_= nl.advertise<geometry_msgs::PointStamped>( "init_ecef", 10, false);
        init_llh_pub_= nl.advertise<geometry_msgs::PointStamped>( "init_llh", 10, false);

        imu_sub_ = nl.subscribe(imu_topic_,  10, &InitCoreNav::ImuCallback, this);
        gps_llh_sub_=nl.subscribe(gps_llh_topic_, 10, &InitCoreNav::GPSLLHCallBack, this);
        gps_ecef_sub_=nl.subscribe(gps_ecef_topic_,10, &InitCoreNav::GPSECEFCallBack, this);

        return true;
}

void InitCoreNav::Propagate(const InitCoreNav::Vector6& imu,const InitCoreNav::Vector3& gps_ecef, const InitCoreNav::Vector3& gps_llh){ // no joint
        dt_imu_ = imu_stamp_curr_ - imu_stamp_prev_;
        count++;

          omega_b_ib_(0) += imu[3];
          omega_b_ib_(1) += imu[4];
          omega_b_ib_(2) += imu[5];
          f_ib_b_(0) += imu[0];
          f_ib_b_(1) += imu[1];
          f_ib_b_(2) += imu[2];

          ins_bias_a(0)= f_ib_b_(0)/count;
          ins_bias_a(1)= f_ib_b_(1)/count;
          ins_bias_a(2)= f_ib_b_(2)/count;

          ins_bias_g(0)= omega_b_ib_(0)/count;
          ins_bias_g(1)= omega_b_ib_(1)/count;
          ins_bias_g(2)= omega_b_ib_(2)/count;

          init_ecef(0)=gps_ecef(0);
          init_ecef(1)=gps_ecef(1);
          init_ecef(2)=gps_ecef(2);
          init_llh(0)=gps_llh(0);
          init_llh(1)=gps_llh(1);
          init_llh(2)=gps_llh(2);

if (count==200) {
  ROS_INFO("Detected acceleration bias:\n %.12f\n %.12f \n %.12f ", ins_bias_a(0),ins_bias_a(1),ins_bias_a(2));
  ROS_INFO("Detected gyroscope bias:\n %.12f\n %.12f \n %.12f ", ins_bias_g(0),ins_bias_g(1),ins_bias_g(2));
  ROS_INFO("GPS LLH solution:\n %.12f\n %.12f \n %.12f ", init_llh(0),init_llh(1),init_llh(2));
  ROS_INFO("GPS ECEF solution:\n %.12f\n %.12f \n %.12f ", init_ecef(0),init_ecef(1),init_ecef(2));


        if (ins_bias_a(1)>1000)
        {
ROS_ERROR("IMU y-axis couldn't initialize, bias_y set to 0.0");
                ins_bias_a(1)=0.0;
        }

  PublishStates(ins_bias_a, bias_a_pub_);
  PublishStates(ins_bias_g, bias_g_pub_);
  PublishStates(init_llh, init_llh_pub_);
  PublishStates(init_ecef, init_ecef_pub_);
// //write params file
// std::string path =  ros::package::getPath("core_nav") + "/config/init_params.yaml";
std::string path2 =  ros::package::getPath("gps_core_nav") + "/config/init_params.yaml";
ROS_INFO("Writing calibration results to file...");
// writeParams(path, ins_bias_a, ins_bias_g, init_ecef, init_llh);
writeParams(path2, ins_bias_a, ins_bias_g, init_ecef, init_llh);
ROS_INFO("Wrote to param file: ");
// std::cout << path.c_str() << std::endl;
std::cout << path2.c_str() << std::endl;
}
// else {
//   std::cout << "%";
//   std::cout <<(count/200)*100 << '\n';
// }

        return;
}

InitCoreNav::Vector6 InitCoreNav::getImuData(const ImuData& imu_dataAdis_)
{
        InitCoreNav::Vector6 imuVec((Vector(6) << imu_dataAdis_.linear_acceleration.x,
                                 imu_dataAdis_.linear_acceleration.y,
                                 imu_dataAdis_.linear_acceleration.z,
                                 imu_dataAdis_.angular_velocity.x,
                                 imu_dataAdis_.angular_velocity.y,
                                 imu_dataAdis_.angular_velocity.z).finished());
        return imuVec;
}

InitCoreNav::Vector3 InitCoreNav::getGPSECEFData(const GPSECEFData& gps_ecef_data_)
{
        InitCoreNav::Vector3 GPSECEFVec((Vector(3) << gps_ecef_data_.x,
                                   gps_ecef_data_.y,
                                   gps_ecef_data_.z).finished());

        return GPSECEFVec;
}

InitCoreNav::Vector3 InitCoreNav::getGPSLLHData(const GPSLLHData& gps_llh_data_)
{
        InitCoreNav::Vector3 GPSLLHVec((Vector(3) <<gps_llh_data_.lat*3.14159265358979/180.0,
                                   gps_llh_data_.lon*3.14159265358979/180.0,
                                   gps_llh_data_.height).finished());

        return GPSLLHVec;
}

// Publish estimated states in global frame
void InitCoreNav::PublishStates(const InitCoreNav::Vector3& states,
                            const ros::Publisher& pub){
        // // Check for subscribers before doing any work.
        if(pub.getNumSubscribers() == 0)
                return;

        geometry_msgs::PointStamped msg;

        msg.point.x = states(0);
        msg.point.y = states(1);
        msg.point.z = states(2);
        msg.header.frame_id = frame_id_imu_;
        msg.header.stamp = ros::Time::now();

        pub.publish(msg);
}

void InitCoreNav::writeParams(std::string path_to_param_file, const InitCoreNav::Vector3& ins_bias_a, const InitCoreNav::Vector3& ins_bias_g, const InitCoreNav::Vector3& init_ecef, const InitCoreNav::Vector3& init_llh){
    // Open file
    std::ofstream paramsFile (path_to_param_file.c_str());

    // Write to file
        paramsFile << "bias_a:" << std::endl;
        paramsFile << std::fixed << std::setprecision(12) << "  x: " << (ins_bias_a(0))<< std::endl;
        paramsFile << std::fixed << std::setprecision(12) << "  y: " << (ins_bias_a(1))<< std::endl;
        paramsFile << std::fixed << std::setprecision(12) << "  z: " << (ins_bias_a(2))<< std::endl;

        paramsFile << "bias_g:" << std::endl;
        paramsFile << std::fixed << std::setprecision(12) << "  x: " << (ins_bias_g(0))<< std::endl;
        paramsFile << std::fixed << std::setprecision(12) << "  y: " << (ins_bias_g(1))<< std::endl;
        paramsFile << std::fixed << std::setprecision(12) << "  z: " << (ins_bias_g(2))<< std::endl;

        paramsFile << "init_ecef:" << std::endl;
        paramsFile << std::fixed << std::setprecision(12) << "  x: " << (init_ecef(0))<< std::endl;
        paramsFile << std::fixed << std::setprecision(12) << "  y: " << (init_ecef(1))<< std::endl;
        paramsFile << std::fixed << std::setprecision(12) << "  z: " << (init_ecef(2))<< std::endl;

        paramsFile << "init_llh:" << std::endl;
        paramsFile << std::fixed << std::setprecision(12) << "  x: " << (init_llh(0))<< std::endl;
        paramsFile << std::fixed << std::setprecision(12) << "  y: " << (init_llh(1))<< std::endl;
        paramsFile << std::fixed << std::setprecision(12) << "  z: " << (init_llh(2))<< std::endl;
        // paramsFile << std::fixed << std::setprecision(5) << "  yaw_offset: " << yaw_offset << std::endl;
        // paramsFile << "  zero_altitude: " << std::boolalpha << zero_altitude << std::endl;
        // paramsFile << "  publish_filtered_gps: " << std::boolalpha << publish_filtered_gps << std::endl;
        // paramsFile << "  use_odometry_yaw: " << std::boolalpha << use_odometry_yaw << std::endl;

    // Close file
    paramsFile.close();
}
