#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>

enum DRIVING_STATE_T {_waiting_for_stop, _stopped} driving_state = _waiting_for_stop;

const double stop_duration = 5.0; // sec
const double drive_speed = 1.0; // m/s
const double stop_speed = 0.0; // m/s

double time_to_stop;
bool stop_commanded = false;

void stopCallback(const std_msgs::Float64::ConstPtr& msg);

int main(int argc, char** argv)
{
    ros::init(argc, argv, "drive_straight_with_stop_node");
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
    ros::Subscriber sub = nh.subscribe("/core_nav/core_nav/stop_cmd", 1, stopCallback);
    ros::Rate loop_rate(20);
    geometry_msgs::Twist cmd_vel_msg;
    double stopped_time;

    while(ros::ok())
    {
        switch(driving_state)
        {
            case _waiting_for_stop:
                if(stop_commanded==true)
                {
                  ROS_INFO("remaining_time = %.3f",time_to_stop - ros::Time::now().toSec());
                    if(ros::Time::now().toSec() > time_to_stop)
                    {
                        stopped_time = ros::Time::now().toSec();
                        driving_state = _stopped;
                    }
                }
                //cmd_vel_msg.linear.x = drive_speed;
                break;
            case _stopped:
                if(ros::Time::now().toSec() - stopped_time > stop_duration)
                {
                    driving_state = _waiting_for_stop;
                    stop_commanded = false;
                }
                cmd_vel_msg.linear.x = stop_speed;
                pub.publish(cmd_vel_msg); // Moved publisher here
                break;
        }
        // Publisher was here
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

void stopCallback(const std_msgs::Float64::ConstPtr& msg)
{
    time_to_stop = ros::Time::now().toSec() + msg->data;
    stop_commanded = true;
    ROS_INFO("delta time = %.3f",msg->data);
    ROS_INFO("time to stop = %.3f",time_to_stop);
}
