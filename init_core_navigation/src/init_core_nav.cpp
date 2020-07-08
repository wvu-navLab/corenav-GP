
#include <ros/ros.h>
#include <init_core_navigation/InitCoreNav.h>

int main(int argc, char** argv){
        ros::init(argc, argv, "inertial propagation");
        ros::NodeHandle n("~");

        InitCoreNav InitCoreNav;
        if(!InitCoreNav.Initialize(n)) {
                ROS_ERROR("%s: Failed to initialize the nav. filter.",
                          ros::this_node::getName().c_str());
                return EXIT_FAILURE;
        }
        ros::spin();

        return EXIT_SUCCESS;
}
