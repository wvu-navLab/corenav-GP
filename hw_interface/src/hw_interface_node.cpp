#include <ros/ros.h>
#include <hw_interface/hw_interface.hpp>

#include <boost/scoped_ptr.hpp>

void mySigintHandler(int sig)
{
    // Do some custom action.
    // For example, publish a stop message to some other nodes.

    // All the default sigint handler does is call shutdown()
    ros::shutdown();
}

int main(int argc, char **argv)
{
    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info) ) {
           ros::console::notifyLoggerLevelsChanged();
        }
    std::string node_type = "hw_interface";
    ROS_INFO("hw_interface Start");
    ros::init(argc, argv, node_type, ros::init_options::NoSigintHandler);
    ROS_INFO(" - ros::init complete");
    if(ros::param::get("node_type", node_type)==false) node_type = "hw_interface";
    ros::NodeHandlePtr nh = ros::NodeHandlePtr(new ros::NodeHandle());
    ROS_INFO(" - node handle created");

    //override default ros handler
    signal(SIGINT, mySigintHandler);

    boost::scoped_ptr<hw_interface> hwInterfacePtr(new hw_interface(nh));
    ROS_DEBUG("HW_Interface Obj start");

    while(nh->ok())
    {
        //This is where monitoring of the interfaces can happen.
        //Monitoring can also be implemented into the ASIO Thread pool by posting
            //the monitoring function into the work queue
        ros::spin();
    }

    hwInterfacePtr.reset();
    ROS_DEBUG("HW_Interface Closing");
    return 0;
}
