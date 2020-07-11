#include <hw_interface_plugin_adis_imu/hw_interface_plugin_adis_imu_serial.hpp>

hw_interface_plugin_adis_imu::adis_imu_serial::adis_imu_serial()
{
    //A debug message
    ROS_INFO("ADIS Plugin Appeared!");

    //force the ROS Console level to Debug Level
    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
           ros::console::notifyLoggerLevelsChanged();
        }

    //enable automatic class metric collection.
    enableMetrics();

    // Compute IMU scale factors once for use later
    gyroScaleFactor = (PI/180)*0.025/65536; // rad/s/LSB /ADIS16488 0.02 | ADIS16495 0.025 deg/sec/lsb
    accelScaleFactor = 9.80665*(0.25e-3/65536); // m/s^2/LSB ADIS16488 0.8e-3 | ADIS16495 0.25e-3 g

    // Set first term in orientation covariance in imu message to -1 to signify no orientation data
    imuMessage.orientation_covariance[0] = -1;
    //imuMessageAdis.orientation_covariance[0] = -1;
}

//this is called each time the plugin is enabled, before anything else of the plugin is called
bool hw_interface_plugin_adis_imu::adis_imu_serial::subPluginInit(ros::NodeHandlePtr nhPtr)
{
    ROS_DEBUG_EXTRA("%s Plugin Init", pluginName.c_str());

    /*for Serial interfaces, 'deviceName' is an inherited member and must be defined.
        failing to define this variable will disable the plugin.
        Opening of the device port is handled automatically

        deviceName is the name and path of the port to be opened
            example: "/dev/ttyS0" */
    deviceName = "";
    ros::param::get(pluginName+"/deviceName", deviceName);

    // bind a functor to the streamCompletionChecker inherited member
    streamCompletionChecker = boost::bind(&hw_interface_plugin_adis_imu::adis_imu_serial::adisIMUStreamMatcher, this, _1, _2);

    // enable the completion functor if the bind succeeded
    enableCompletionFunctor = !streamCompletionChecker.empty();

    //retrieve string from the ROS Parameter Server
        //of the format '<plugin_name>/<parameter>
    std::string tempString = "";

    //place that wants to write data to the device
    if(ros::param::get(pluginName+"/subscribeToTopic", tempString))
    {
	//This will create a ros subscriber to the topic from the ROS parameter server.
	    //The rosMsgCallback method will be called whenever there is a message pending.
        //rosDataSub = nh->subscribe(tempString, 1, &roboteq_drive::rosMsgCallback, this);
    }
    else
    {
        ROS_ERROR("%s:: Could not find topic subscription name", pluginName.c_str());
    }

    //place to publish the data after reading from device
    if(ros::param::get(pluginName+"/publishToTopic", tempString))
    {
        imuPublisher = nhPtr->advertise<sensor_msgs::Imu>(tempString, 1, false);

    }
    if(ros::param::get(pluginName+"/publishToTopicAdis", tempString))
    {
        imuPublisherAdis = nhPtr->advertise<sensor_msgs::Imu>(tempString, 1, false);

    }
    else
    {
        ROS_ERROR("%s:: Could not find topic advertisement name", pluginName.c_str());
    }

    return true;
}

/*this function is called to setup the port
    typical serial port setup uses 115200 baud rate, 8 bit character size, no flow control,
      no parity, 1 stop bit. This is typical, but no all encompassing. Change this if
      the port requires different. */
void hw_interface_plugin_adis_imu::adis_imu_serial::setInterfaceOptions()
{
	int tempBaudRate = 0;
    ros::param::get(pluginName+"/baudrate", tempBaudRate);

    setOption<boost::asio::serial_port_base::baud_rate>(
                new boost::asio::serial_port_base::baud_rate(tempBaudRate));
                ROS_INFO("Set baudrate");
    //8 bits per character
    setOption<boost::asio::serial_port_base::character_size>(
    			new boost::asio::serial_port_base::character_size( 8 ));
//ROS_INFO("Set character size");
    //flow control
    setOption<boost::asio::serial_port_base::flow_control>(
    			new boost::asio::serial_port_base::flow_control(
    										boost::asio::serial_port_base::flow_control::type::none));
//ROS_INFO("Set flow control");
    //parity
    setOption<boost::asio::serial_port_base::parity>(
    			new boost::asio::serial_port_base::parity(
    										boost::asio::serial_port_base::parity::type::none));
//ROS_INFO("Set parity");
    //stop-bits
    setOption<boost::asio::serial_port_base::stop_bits>(
    			new boost::asio::serial_port_base::stop_bits(
    										boost::asio::serial_port_base::stop_bits::type::one));
//ROS_INFO("Set stop-bits");
//    ROS_INFO("%s :: Device: %s :: Baudrate %d", pluginName.c_str(), deviceName.c_str(), tempBaudRate);

}

//this is called automatically when data that passes the streamMatcher is okay
    //this function is called with a data length and a position in an inherited array member
        //named 'receivedData'
//data should be published to topics from here
bool hw_interface_plugin_adis_imu::adis_imu_serial::interfaceReadHandler(const size_t &length,int arrayStartPos, const boost::system::error_code &ec)
{
//ROS_INFO("ADIS IMU Plugin Data Handler %d",arrayStartPos);
        headerLen=3;
	
        if((receivedData[arrayStartPos]==0x41) && (receivedData[arrayStartPos+1]==0x7A) && (receivedData[arrayStartPos+2]==0x30)) // Check if message starts with Az0
        {

//ROS_INFO("Made it");
            // populate IMU message accelerations and angular velocities
	    //
      //int j;
      //for ( j=0; j<35; j++){
        //ROS_INFO("byte %d %d",j, receivedData[arrayStartPos+headerLen+j]);
      //}
      imuMessage.linear_acceleration.x = accelScaleFactor*((double)((receivedData[arrayStartPos+headerLen+13]) +
                                                                   ((receivedData[arrayStartPos+headerLen+12])<<8) +
                       				                                     ((receivedData[arrayStartPos+headerLen+11])<<16) +
                			                                             ((receivedData[arrayStartPos+headerLen+10])<<24)));


imuMessageAdis.linear_acceleration.x = imuMessage.linear_acceleration.x;
	    imuMessage.linear_acceleration.y = accelScaleFactor*((double)((receivedData[arrayStartPos+headerLen+17]) +
                                                                   ((receivedData[arrayStartPos+headerLen+16])<<8) +
                                                                   ((receivedData[arrayStartPos+headerLen+15])<<16) +
                                                                   ((receivedData[arrayStartPos+headerLen+14])<<24)));
imuMessageAdis.linear_acceleration.y = imuMessage.linear_acceleration.y;
	    imuMessage.linear_acceleration.z = accelScaleFactor*((double)((receivedData[arrayStartPos+headerLen+21]) +
                                                                   ((receivedData[arrayStartPos+headerLen+20])<<8) +
                                                                   ((receivedData[arrayStartPos+headerLen+19])<<16) +
                                                                   ((receivedData[arrayStartPos+headerLen+18])<<24)));

imuMessageAdis.linear_acceleration.z = imuMessage.linear_acceleration.z;
	    imuMessage.angular_velocity.x = gyroScaleFactor*((double)((receivedData[arrayStartPos+headerLen+25]) +
                                                               ((receivedData[arrayStartPos+headerLen+24])<<8) +
                                                               ((receivedData[arrayStartPos+headerLen+23])<<16) +
						                                                   ((receivedData[arrayStartPos+headerLen+22])<<24)));
imuMessageAdis.angular_velocity.x = imuMessage.angular_velocity.x;
	    imuMessage.angular_velocity.y = gyroScaleFactor*((double)((receivedData[arrayStartPos+headerLen+29]) +
                                                               ((receivedData[arrayStartPos+headerLen+28])<<8) +
                                                               ((receivedData[arrayStartPos+headerLen+27])<<16) +
                                                               ((receivedData[arrayStartPos+headerLen+26])<<24)));
imuMessageAdis.angular_velocity.y = imuMessage.angular_velocity.y;
	    imuMessage.angular_velocity.z = gyroScaleFactor*((double)((receivedData[arrayStartPos+headerLen+33]) +
                                                               ((receivedData[arrayStartPos+headerLen+32])<<8) +
                                                               ((receivedData[arrayStartPos+headerLen+31])<<16) +
                                                               ((receivedData[arrayStartPos+headerLen+30])<<24)));
imuMessageAdis.angular_velocity.z = imuMessage.angular_velocity.z;
	    // populate IMU message header
     std::string imu_link = "imu_link";
     imuMessage.header.frame_id = imu_link;

 union { char b[8]; double TimeStamp; };
	int cc=0;
	for (int jj=9; jj>1; jj--){
		b[cc]=receivedData[arrayStartPos+headerLen+jj];
		cc=cc+1;
	}
double param, fractpart, intpart;
 fractpart = modf (TimeStamp , &intpart);

    imuMessageAdis.header.stamp.sec = intpart;
    imuMessageAdis.header.stamp.nsec = fractpart*1.0e9;
  //ROS_INFO("IMU Time =%lf %lf %lf",TimeStamp, intpart, fractpart);


imuMessageAdis.header.seq =(double)((receivedData[arrayStartPos+headerLen+1]) + ((receivedData[arrayStartPos+headerLen])<<8));
      // publish message
      imuMessage.header.stamp = ros::Time::now();


      imuPublisher.publish(imuMessage);
      imuPublisherAdis.publish(imuMessageAdis);
      // increment sequence number for next cycle
      imuMessage.header.seq++;
      imuMessageAdis.header.seq++;
	   }
    return true;
}


//automatically called to check the checksum of the packet.
    //If un-wanted/un-needed, return true.
bool hw_interface_plugin_adis_imu::adis_imu_serial::verifyChecksum()
{
    return true;
}

std::size_t hw_interface_plugin_adis_imu::adis_imu_serial::adisIMUStreamMatcher(const boost::system::error_code &error, long totalBytesInBuffer)
{
    const long syncSeqLen = 3;
    headerLen = 0;
    fullPacketLen = 0;
    dataArrayStart = 0;
    //ROS_INFO("Before the loop here=%d %c %c %c %u %u %u",totalBytesInBuffer, receivedData[0], receivedData[1],receivedData[2], receivedData[0], receivedData[1],receivedData[2]  );
    if(receivedData!=0)
    {
      //ROS_INFO("receivedData:%d",receivedData[0]);
    }
    //ROS_INFO("receivedData:%d, totalBytesInBuffer=%d ",receivedData[0],totalBytesInBuffer);
    if(error != boost::asio::error::operation_aborted)
    {
        if(totalBytesInBuffer < syncSeqLen)
        {
            //ROS_DEBUG_EXTRA_NAME("Returning Early, %ld", syncSeqLen-totalBytesInBuffer);
            //ROS_INFO("1Return from here=%d",totalBytesInBuffer);
            return syncSeqLen-totalBytesInBuffer;
        }
        for(long i=0; i<totalBytesInBuffer; i++)
        {
            if((receivedData[i]&0xFF) == 0x41) // Find first sync char
            //if((receivedData[i]&0xFF) == 0x41)
            {
                if((totalBytesInBuffer-(i+1)) < (syncSeqLen-1)) // Not enough to find rest of rest of sync sequence, read remaining
                {
                    //ROS_INFO("2Return from here=%d %c %c %c",totalBytesInBuffer, receivedData[i], receivedData[i+1],receivedData[i+2]  );
                    return syncSeqLen-1;
                }
                else
                {
                    if((receivedData[i+1]&0xFF) == 0x7A) // Find second sync char
                    //if((receivedData[i+1]) == 0x7A)
                    {
                        if((totalBytesInBuffer-(i+1)-1) < (syncSeqLen-2)) // Not enough to find rest of rest of sync sequence, read remaining
                        {
                          //ROS_INFO("3Return from here=%x",receivedData[i]);
                           return syncSeqLen-2;
                        }
                        else
                        {
                            if((receivedData[i+2]&0xFF) == 0x30) // Find third sync char
                            //if((receivedData[i+2]) == 0x30)
                            {
                                dataArrayStart = i;
                                //OS_INFO("Array Start=%d",i);
                                if(totalBytesInBuffer<(i+35)){
                                    return 1;// not enough bytes
                                }
                                else{
			 	                           return 0;
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}
unsigned long hw_interface_plugin_adis_imu::adis_imu_serial::CRC32Value_(int i)
{
    int j;
    unsigned long ulCRC;
    ulCRC = i;
    for ( j = 8 ; j > 0; j-- )
    {
        if ( ulCRC & 1 )
            ulCRC = ( ulCRC >> 1 ) ^ CRC32_POLYNOMIAL;
        else
            ulCRC >>= 1;
    }
    return ulCRC;
}

unsigned long hw_interface_plugin_adis_imu::adis_imu_serial::CalculateBlockCRC32_(unsigned long ulCount, unsigned char *ucBuffer)
{
    unsigned long ulTemp1;
    unsigned long ulTemp2;
    unsigned long ulCRC = 0;
    while ( ulCount-- != 0 )
    {
        ulTemp1 = ( ulCRC >> 8 ) & 0x00FFFFFFL;
        ulTemp2 = CRC32Value_( ((int) ulCRC ^ *ucBuffer++ ) & 0xff );
        ulCRC = ulTemp1 ^ ulTemp2;
    }
    return ulCRC;
}
