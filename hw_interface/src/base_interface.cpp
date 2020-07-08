#include <ros/ros.h>

#include <hw_interface/base_interface.hpp>

#include <boost/accumulators/statistics/mean.hpp>

base_classes::base_interface::base_interface() :
    lastTimeMetric(ros::Time::now()),
    acc(boost::accumulators::tag::rolling_window::window_size = 150)
{
    enabled = true;
}


bool base_classes::base_interface::enableMetrics()
{
    metricsEnabled = true;
    this->lastTimeMetric = ros::Time::now();
    return metricsEnabled;
}

bool base_classes::base_interface::disableMetrics()
{
    metricsEnabled = false;
    return metricsEnabled;
}

std::string base_classes::base_interface::printMetrics(bool printSideEffect)
{
    if(metricsEnabled)
    {
        boost::scoped_ptr<char> output(new char[255]);
        deltaTime = ros::Time::now().toSec()-this->lastTimeMetric.toSec();
        acc(deltaTime);
        sprintf(output.get(), "Thread <%s>:: Interface <%15s>:: Delta Time %2.3f:: Hz %2.2f:: Avg Hz %2.2f",
                                        THREAD_ID_TO_C_STR, this->pluginName.c_str(),
                                        deltaTime, 1/deltaTime, 1/(boost::accumulators::rolling_mean(acc)));
        this->lastTimeMetric = ros::Time::now();
        if(printSideEffect)
        {
            //ROS_WARN_THROTTLE(5, "%s", output.get());
        }
		
        std::string outputString(1, *output);
        return outputString;
    }
    else
    {
        return "";
    }
}

uint16_t base_classes::base_interface::calcCRC16Block(const void * const buf, std::size_t numOfBytes)
{
    return boost::crc<16, 0x1021, 0xFFFF, 0, false, false>(buf, numOfBytes);
}

//this should be multi access safe, as members are generated on the stack each call.
//need to double check
uint32_t base_classes::base_interface::calcCRC32Block(const void * const buf, std::size_t numOfBytes)
{
    boost::crc_32_type crcComputer;
    crcComputer.process_bytes(buf, numOfBytes);
    return crcComputer.checksum();
    //return boost::crc<32, 0x1021, 0xFFFF, 0, false, false>(buf, numOfBytes);
}

std::size_t base_classes::base_interface::streamMatcherDelimAndLength(const boost::system::error_code &error, long totalBytesInBuffer,
                                                                      const int &packetLengthInBytes, const char *headerSequence,
                                                                      const char *footerSequence)
{
    ROS_DEBUG("%s:: Length and footer matcher: buffer length%lu", pluginName.c_str(), totalBytesInBuffer);
    ROS_DEBUG("packetlength %d", packetLengthInBytes);
    ROS_DEBUG("Candidate Header: %s", headerSequence);
    //ROS_DEBUG("Candidate Footer: %s", footerSequence);
    if(totalBytesInBuffer <= std::strlen(footerSequence))
    {
        ROS_DEBUG("%s:: Matcher Returning Early", pluginName.c_str());
        return packetLengthInBytes - totalBytesInBuffer;
    }
    if((packetLengthInBytes - totalBytesInBuffer) <= 0)
    {
        ROS_DEBUG("%s:: Full Length Packet Received", pluginName.c_str());
        std::printf("Contents: ");
	    for(int i = 0; i < totalBytesInBuffer; i++)
	    {
            std::printf("%c, %X | ", receivedData[i], receivedData[i]);
	    }
	    std::printf("\r\n");
        const int footerLength = std::strlen(footerSequence);
        int i = 0;
        int j = footerLength-1;
        std::cout << "Footer: ";
        for(i = 0; i < footerLength; i++)
        {
            std::printf("%c | ", receivedData[ totalBytesInBuffer - 1 - i ]);
            if(receivedData[ totalBytesInBuffer - 1 - i ] != footerSequence[j])
            {
                //THIS IS WHERE AN HSM invalid message can be sent
                std::printf("\r\n");
                ROS_ERROR("%s:: Invalid Footer\r\n", pluginName.c_str());
                return footerLength;
            }
            j--;
        }
        std::cout << std::endl;
        const int headerLength = std::strlen(headerSequence);
        std::printf("%d Header: ", headerLength);
        for(i = 0; i < headerLength; i++)
        {
            std::printf("%x, %x, %d| ", receivedData[totalBytesInBuffer - packetLengthInBytes + i], (headerSequence[i] & 0xff),
                        receivedData[totalBytesInBuffer - packetLengthInBytes + i] == headerSequence[i]);
            if(receivedData[totalBytesInBuffer - packetLengthInBytes + i] != (headerSequence[i] & 0xff))
            {
                //THIS IS WHERE AN HSM invalid message can be sent
                std::printf("\r\n");
                ROS_ERROR("%s:: Invalid Header\r\n", pluginName.c_str());
                return packetLengthInBytes;
            }
        }
        std::cout << std::endl;
        //should post something to HSM here.

        ROS_DEBUG("%s:: Header Found, Footer Found, Correct Length, Good Packet", pluginName.c_str());

        dataArrayStart = ( totalBytesInBuffer - packetLengthInBytes );
        return 0;
    }
    return packetLengthInBytes - totalBytesInBuffer;
}
