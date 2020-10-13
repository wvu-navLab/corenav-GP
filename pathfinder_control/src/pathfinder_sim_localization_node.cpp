#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

class SimLocalization
{
public:
    // Members    
    ros::NodeHandle nh;
    ros::Subscriber odom_sub;
    tf::TransformBroadcaster tf_broad;
    geometry_msgs::TransformStamped odom_trans;
    nav_msgs::Odometry odom;
    // Methods
    SimLocalization()
    {
        odom_sub = nh.subscribe("odom",1,&SimLocalization::odomCallback,this);
    }
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        odom_trans.header = msg->header;
        odom_trans.child_frame_id = msg->child_frame_id;
        odom_trans.transform.translation.x = msg->pose.pose.position.x;
        odom_trans.transform.translation.y = msg->pose.pose.position.y;
        odom_trans.transform.translation.z = msg->pose.pose.position.z;
        odom_trans.transform.rotation = msg->pose.pose.orientation;
        tf_broad.sendTransform(odom_trans);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pathfinder_sim_localization");
    SimLocalization simLocalization;
    ros::spin();
    return 0;
}
