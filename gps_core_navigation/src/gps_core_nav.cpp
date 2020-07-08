/*
 * Author: Cagri
 */

#include <ros/ros.h>
#include <gps_core_navigation/gps_CoreNav.h>

int main(int argc, char** argv){
        ros::init(argc, argv, "inertial propagation");
        ros::NodeHandle n("~");

        gps_CoreNav gps_coreNav;
        if(!gps_coreNav.Initialize(n)) {
                ROS_ERROR("%s: Failed to initialize the nav. filter.",
                          ros::this_node::getName().c_str());
                return EXIT_FAILURE;
        }
        while(ros::ok())
        {
            if(gps_coreNav.propagate_flag)
            {
                gps_coreNav.Propagate(gps_coreNav.imu,gps_coreNav.odo,gps_coreNav.cmd,gps_coreNav.encoderLeft,gps_coreNav.encoderRight,gps_coreNav.joint, gps_coreNav.gps_ecef, gps_coreNav.gps_llh);
                //ROS_INFO("after Propagate\n");
                gps_coreNav.propagate_flag =false;
            }

            if(gps_coreNav.update_flag)
            {
                gps_coreNav.Update(gps_coreNav.odo,gps_coreNav.joint);
                //ROS_INFO("after Propagate\n");
                gps_coreNav.update_flag =false;
            }
            ros::spinOnce();
        }
        //ros::spin();

        return EXIT_SUCCESS;
}
