/*********************************************************************
 * * Software License Agreement (BSD License)
 * *
 * * Copyright (c) <YEAR>, WVU Interactive Robotics Laboratory
 * *                       https://web.statler.wvu.edu/~irl/
 * * All rights reserved.
 * *
 * *  Redistribution and use in source and binary forms, with or without
 * *  modification, are permitted provided that the following conditions
 * *  are met:
 * *
 * *   * Redistributions of source code must retain the above copyright
 * *     notice, this list of conditions and the following disclaimer.
 * *   * Redistributions in binary form must reproduce the above
 * *     copyright notice, this list of conditions and the following
 * *     disclaimer in the documentation and/or other materials provided
 * *     with the distribution.
 * *   * Neither the name of the WVU Interactive Robotics Laboratory nor
 * *     the names of its contributors may be used to endorse or promote products
 * *     derived from this software without specific prior written permission.
 * *
 * *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * *  POSSIBILITY OF SUCH DAMAGE.
 * *********************************************************************/

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <stdint.h>

#define MAX_SPEED 2

class PathfinderControl
{
public:
    // Members
    ros::NodeHandle nh;
    ros::Publisher fl_pub, bl_pub, fr_pub, br_pub;
    ros::Subscriber twist_sub;
    geometry_msgs::Twist cmd_vel;
    std_msgs::Float64 fl_motor_speed_cmd, bl_motor_speed_cmd, fr_motor_speed_cmd, br_motor_speed_cmd;
    ros::Rate loop_rate; // Hz
    ros::Time last_callback_time;
    const double timeout_period; // sec
    const double velocity_to_integer_gain; // scale factor to convert m/s velocity commands to [-1000, 1000] integer value for roboteq command
    // Methods
    PathfinderControl() // Constructor
        : timeout_period(1.0), // sec
          velocity_to_integer_gain(9.0), // m/s to [-MAX_SPEED,MAX_SPEED] scale factor
          loop_rate(20) // Hz
    {
        // TODO: make topic names parameters
        fl_pub = nh.advertise<std_msgs::Float64>("/pathfinder/front_left_velocity_controller/command", 1);
        bl_pub = nh.advertise<std_msgs::Float64>("/pathfinder/back_left_velocity_controller/command", 1);
        fr_pub = nh.advertise<std_msgs::Float64>("/pathfinder/front_right_velocity_controller/command", 1);
        br_pub = nh.advertise<std_msgs::Float64>("/pathfinder/back_right_velocity_controller/command", 1);
        twist_sub = nh.subscribe("cmd_vel_out", 1, &PathfinderControl::twistCallback, this);
        last_callback_time = ros::Time::now();
    }

    void twistCallback(const geometry_msgs::Twist::ConstPtr& _msg)
    {
        cmd_vel = *_msg;
        last_callback_time = ros::Time::now();
    }

    int coerceRoboteqOutput(int _value_in)
    {
        if(_value_in > MAX_SPEED)
        {
            return MAX_SPEED;
        }
        else if(_value_in < -MAX_SPEED)
        {
            return -MAX_SPEED;
        }
else
        {
            return _value_in;
        }
    }

    void run()
    {
        while(ros::ok())
        {
            if((ros::Time::now().toSec() - last_callback_time.toSec()) > timeout_period)
            {
                // Timeout since last message expired, set motor speeds to zero
                fl_motor_speed_cmd.data = 0;
                fr_motor_speed_cmd.data = 0;
                bl_motor_speed_cmd.data = 0;
                br_motor_speed_cmd.data = 0;
            }
            else
            {
                // Compute motor velocities, based on commanded linear and angular velocities
                fl_motor_speed_cmd.data = (int16_t)(cmd_vel.linear.x*velocity_to_integer_gain -
                    cmd_vel.angular.z*velocity_to_integer_gain);
                fr_motor_speed_cmd.data = (int16_t)(cmd_vel.linear.x*velocity_to_integer_gain +
                    cmd_vel.angular.z*velocity_to_integer_gain);
                bl_motor_speed_cmd.data = (int16_t)(cmd_vel.linear.x*velocity_to_integer_gain -
                    cmd_vel.angular.z*velocity_to_integer_gain);
                br_motor_speed_cmd.data = (int16_t)(cmd_vel.linear.x*velocity_to_integer_gain +
                    cmd_vel.angular.z*velocity_to_integer_gain);

                // Coerce motor velocity commands to be within [-1000,1000] range
                fl_motor_speed_cmd.data = coerceRoboteqOutput(fl_motor_speed_cmd.data);
                fr_motor_speed_cmd.data = coerceRoboteqOutput(fr_motor_speed_cmd.data);
                bl_motor_speed_cmd.data = coerceRoboteqOutput(bl_motor_speed_cmd.data);
                br_motor_speed_cmd.data = coerceRoboteqOutput(br_motor_speed_cmd.data);
            }
            // Publish motor outputs, sleep, and spin
            fl_pub.publish(fl_motor_speed_cmd);
            fr_pub.publish(fr_motor_speed_cmd);
            bl_pub.publish(bl_motor_speed_cmd);
            br_pub.publish(br_motor_speed_cmd);
            loop_rate.sleep();
            ros::spinOnce();
        }
    }
}; 

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "pathfinder_control_node");    
    PathfinderControl pathfinderControl;
    pathfinderControl.run();

    return 0;
}
