/*********************************************************************
 * * Software License Agreement (BSD License)
 * *
 * * Copyright (c) 2019, WVU Interactive Robotics Laboratory
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
#include <hw_interface_plugin_roboteq/ActuatorOut.h>
#include <stdint.h>

class PathfinderControl
{
public:
    // Members
    ros::NodeHandle nh;
    ros::Publisher roboteq_pub;
    ros::Subscriber twist_sub;
    geometry_msgs::Twist cmd_vel;
    hw_interface_plugin_roboteq::ActuatorOut actuator_msg_out;
    ros::Rate loop_rate; // Hz
    ros::Time last_callback_time;
    const double timeout_period; // sec
    const double velocity_to_integer_gain; // scale factor to convert m/s velocity commands to [-1000, 1000] integer value for roboteq command
    const double left_right_speed_gain; // scale factor to correct for different motor friction between two sides of robot
    // Methods
    PathfinderControl() // Constructor
        : timeout_period(1.0), // sec
          velocity_to_integer_gain(1000.0), // m/s to [-1000,1000] scale factor
          left_right_speed_gain(1.0),
          loop_rate(20) // Hz
    {
        // TODO: make topic names parameters
        roboteq_pub = nh.advertise<hw_interface_plugin_roboteq::ActuatorOut>("control/actuatorout", 1);
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
        if(_value_in > 1000)
        {
            return 1000;
        }
        else if(_value_in < -1000)
        {
            return -1000;
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
                actuator_msg_out.fl_speed_cmd = 0;
                actuator_msg_out.fr_speed_cmd = 0;
                actuator_msg_out.bl_speed_cmd = 0;
                actuator_msg_out.br_speed_cmd = 0;
            }
            else
            {
                // Compute motor velocities, based on commanded linear and angular velocities
                actuator_msg_out.fl_speed_cmd = -(int16_t)((cmd_vel.linear.x*velocity_to_integer_gain -
                    cmd_vel.angular.z*velocity_to_integer_gain)*left_right_speed_gain);
                actuator_msg_out.fr_speed_cmd = (int16_t)((cmd_vel.linear.x*velocity_to_integer_gain +
                    cmd_vel.angular.z*velocity_to_integer_gain)/left_right_speed_gain);
                actuator_msg_out.bl_speed_cmd = -(int16_t)((cmd_vel.linear.x*velocity_to_integer_gain -
                    cmd_vel.angular.z*velocity_to_integer_gain)*left_right_speed_gain);
                actuator_msg_out.br_speed_cmd = (int16_t)((cmd_vel.linear.x*velocity_to_integer_gain +
                    cmd_vel.angular.z*velocity_to_integer_gain)/left_right_speed_gain);

                // Coerce motor velocity commands to be within [-1000,1000] range
                actuator_msg_out.fl_speed_cmd = coerceRoboteqOutput(actuator_msg_out.fl_speed_cmd);
                actuator_msg_out.fr_speed_cmd = coerceRoboteqOutput(actuator_msg_out.fr_speed_cmd);
                actuator_msg_out.bl_speed_cmd = coerceRoboteqOutput(actuator_msg_out.bl_speed_cmd);
                actuator_msg_out.br_speed_cmd = coerceRoboteqOutput(actuator_msg_out.br_speed_cmd);
            }
            // Publish motor outputs, sleep, and spin
            roboteq_pub.publish(actuator_msg_out);
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
