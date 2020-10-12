#ifndef HW_INTERFACE_PLUGIN_ADIS_IMU_HPP__
#define HW_INTERFACE_PLUGIN_ADIS_IMU_HPP__

//always inlclude these
#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <hw_interface/base_interface.hpp>

//include the header of the base type you want, Serial or UDP
#include <hw_interface/base_serial_interface.hpp>
//include ros message types
#include <sensor_msgs/Imu.h>
//TODO
//#include <sensor_msgs/MagneticField.h>
//#include <sensor_msgs/FluidPressure.h>

#define CRC32_POLYNOMIAL 0xEDB88320L
#define PI 3.14159265358979
#define RAD2DEG 180.0/PI
#define DEG2RAD PI/180.0
#define IMU_RATE 125.0 // Hz

namespace hw_interface_plugin_adis_imu {

    class adis_imu_serial : public base_classes::base_serial_interface
    {
    public:
        adis_imu_serial();
        ~adis_imu_serial() {}

    protected:

        //these methods are abstract as defined by the base_serial_interface
            //they must be defined
        bool subPluginInit(ros::NodeHandlePtr nhPtr);
        void setInterfaceOptions();
        bool interfaceReadHandler(const size_t &length, int arrayStartPos, const boost::system::error_code &ec);
        bool verifyChecksum();

        std::size_t adisIMUStreamMatcher(const boost::system::error_code &error, long totalBytesInBuffer);

        long headerLen;
        long fullPacketLen;
        sensor_msgs::Imu imuMessage;
        sensor_msgs::Imu imuMessageAdis;
        //TODO
        //sensor_msgs::MagneticField magMessage
        //sensor_msgs::FluidPressure barMessage
        ros::Publisher imuPublisher;
        ros::Publisher imuPublisherAdis;
        //TODO
        //ros::Publisher magPublisher;
        //ros::Publisher barPublisher;
        double gyroScaleFactor;
        double accelScaleFactor;

    private:
        unsigned long CRC32Value_(int i);
        unsigned long CalculateBlockCRC32_(unsigned long ulCount, unsigned char *ucBuffer ); // Number of bytes in the data block, Data block
    };

}

PLUGINLIB_EXPORT_CLASS(hw_interface_plugin_adis_imu::adis_imu_serial, base_classes::base_interface)



#endif //HW_INTERFACE_PLUGIN_ADIS_IMU_HPP__

