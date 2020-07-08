/*
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 *    1. Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *
 *    2. Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *
 *    3. Neither the name of the copyright holder nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS AS IS
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Please contact the author(s) of this library if you have any questions.
 * Authors: Cagri, Ryan
 */

#include <ros/ros.h>
#include <core_navigation/CoreNav.h>

int main(int argc, char** argv){
        ros::init(argc, argv, "inertial propogation");
        ros::NodeHandle n("~");

        CoreNav coreNav;
        if(!coreNav.Initialize(n)) {
                ROS_ERROR("%s: Failed to initialize the nav. filter.",
                          ros::this_node::getName().c_str());
                return EXIT_FAILURE;
        }
        while(ros::ok())
        {
            if(coreNav.propagate_flag)
            {
                coreNav.Propagate(coreNav.imu,coreNav.odo,coreNav.cmd,coreNav.encoderLeft,coreNav.encoderRight,coreNav.joint);
                //ROS_INFO("after Propagate\n");
                coreNav.propagate_flag =false;
            }

            if(coreNav.update_flag)
            {
                coreNav.Update(coreNav.odo,coreNav.joint);
                //ROS_INFO("after Propagate\n");
                coreNav.update_flag =false;
            }
            ros::spinOnce();
        }
        //ros::spin();

        return EXIT_SUCCESS;
}
