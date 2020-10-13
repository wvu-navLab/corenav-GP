#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <message_filters/subscriber.h>


class ImuFilter {
  public:

    // IMU data RAW vector for values reading
    std::vector<double> imu_raw_ = {0.0,0.0,0.0,0.0,0.0,0.0};

    // IMU filtered data vector
    std::vector<double> imu_filt_ = {0.0,0.0,0.0,0.0,0.0,0.0};

    // Filter Weight Parameter
    int count=0;

    // Output Publisher
    ros::Publisher imu_filt_pub_;

    // IMU RAW data callback function
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);

    // Filter Function
    void filter();

    // Publisher
    void publishFilt(const sensor_msgs::Imu::ConstPtr& msg);

};

// Callback for Filtering
void ImuFilter::imuCallback(const sensor_msgs::Imu::ConstPtr &msg) {

  count++;
  // Clear IMU Raw Container


  // Fill IMU Raw Acceleration
  imu_raw_[0] += msg->linear_acceleration.x;
  imu_raw_[1] += msg->linear_acceleration.y;
  imu_raw_[2] += msg->linear_acceleration.z;

  // Fill IMU Raw Angular Velocity
  imu_raw_[3] += msg->angular_velocity.x;
  imu_raw_[4] += msg->angular_velocity.y;
  imu_raw_[5] += msg->angular_velocity.z;

  imu_filt_[0] = imu_raw_[0] / count;
  imu_filt_[1] = imu_raw_[1] / count;
  imu_filt_[2] = imu_raw_[2] / count;


  imu_filt_[3] = imu_raw_[3] / count;
  imu_filt_[4] = imu_raw_[4] / count;
  imu_filt_[5] = imu_raw_[5] / count;

  if (count == 5) {

    // imu_filt_[0] = imu_raw_[0] / 5;
    // imu_filt_[1] = imu_raw_[1] / 5;
    // imu_filt_[2] = imu_raw_[2] / 5;
    //

    // imu_filt_[3] = imu_raw_[3] / 5;
    // imu_filt_[4] = imu_raw_[4] / 5;
    // imu_filt_[5] = imu_raw_[5] / 5;
    // imu_raw_.clear();
    publishFilt(msg);
    imu_raw_.clear();
    imu_filt_.clear();
    imu_raw_[0] = 0;
    imu_raw_[1] = 0;
    imu_raw_[2] = 0;
    imu_raw_[3] = 0;
    imu_raw_[4] = 0;
    imu_raw_[5] = 0;
    imu_filt_[0] = 0;
    imu_filt_[1] = 0;
    imu_filt_[2] = 0;
    imu_filt_[3] = 0;
    imu_filt_[4] = 0;
    imu_filt_[5] = 0;
    count = 0;
  }
}



// Publish Filtered Message
void ImuFilter::publishFilt(const sensor_msgs::Imu::ConstPtr& msg) {

  sensor_msgs::Imu msg_out;

  msg_out.header.stamp = msg->header.stamp;
  msg_out.header.frame_id ="imu_link";

  msg_out.linear_acceleration.x = imu_filt_[0];
  msg_out.linear_acceleration.y = imu_filt_[1];
  msg_out.linear_acceleration.z = imu_filt_[2];

  msg_out.angular_velocity.x = imu_filt_[3];
  msg_out.angular_velocity.y = imu_filt_[4];
  msg_out.angular_velocity.z = imu_filt_[5];


  imu_filt_pub_.publish(msg_out);

}


int main(int argc, char **argv){

  // Init ROS
  ros::init(argc, argv, "imu_filter");

  ros::NodeHandle nh;

  ImuFilter imu_filter_;

  message_filters::Subscriber<sensor_msgs::Imu> imu_raw_sub(nh,"imu_raw", 1);
  	imu_raw_sub.registerCallback(boost::bind(&ImuFilter::imuCallback, &imu_filter_,_1));

  imu_filter_.imu_filt_pub_ = nh.advertise<sensor_msgs::Imu>("imu_filtered", 1);;


  // Spin
  ros::spin();
}
