#include <ros/ros.h>

#include <hw_interface/base_serial_interface.hpp>

base_classes::base_serial_interface::base_serial_interface()
{
    ROS_DEBUG_EXTRA_SINGLE("Serial Plugin Initialized");
    interfaceType = base_classes::Serial;
    interfaceStarted = false;

    enableCompletionFunctor = false;
    enableStreamMatcher = false;

    enableMetrics();

    receivedData = boost::shared_array<uint8_t>(new uint8_t[500]);

    interfacePort = boost::shared_ptr<boost::asio::serial_port>();


}

bool base_classes::base_serial_interface::interfaceReady()
{
    if(!interfacePort.get())
    {
        return false;
    }
    return interfacePort->is_open();
}

bool base_classes::base_serial_interface::initPlugin(ros::NodeHandlePtr nhPtr,
                                                        const boost::shared_ptr<boost::asio::io_service> ioService)
{
    enableCompletionFunctor = false;
    enableRegexReadUntil = false;
    //initilize output data stream strand
    interfaceSynchronousStrand = boost::shared_ptr<boost::asio::strand>(new boost::asio::strand(*ioService));

    //to use the base classes option settings, a blank port must be provided before
    //initilizing plugin
    interfacePort = boost::shared_ptr<boost::asio::serial_port>(new boost::asio::serial_port(*ioService));

    //call plugin setup
    ROS_DEBUG_EXTRA_SINGLE("Calling Plugin's Init");
    subPluginInit(nhPtr);

    interfacePort->open(deviceName);

    setInterfaceOptions();

    return interfaceReady();
}

bool base_classes::base_serial_interface::startWork()
{
    if(!interfaceReady())
    {
        return false;
    }
    if(!interfaceStarted)
    {
        interfaceStarted = pluginStart();
    }

    if(interfaceStarted)
    {

        if(enableCompletionFunctor)
        {
            ROS_DEBUG_ONCE("%s %d:: Async Read custom Functor", __FILE__, __LINE__);
            boost::asio::async_read(*interfacePort, boost::asio::buffer(receivedData.get(), MAX_SERIAL_READ),
                                        streamCompletionChecker,
                                        boost::bind(&base_serial_interface::handleIORequest,this,
                                                        boost::asio::placeholders::error(),
                                                        boost::asio::placeholders::bytes_transferred()));

        }
        else if(enableRegexReadUntil)
        {
			      ROS_DEBUG_ONCE("%s %d:: REGEX Async Read Until", __FILE__, __LINE__);
            boost::asio::async_read_until(*interfacePort, interfaceRegexBuffer,
                                            regexExpr,
                                            boost::bind(&base_serial_interface::handleRegexRequest,this,
                                                            boost::asio::placeholders::error(),
                                                            boost::asio::placeholders::bytes_transferred()));
        }
        else
        {
          ROS_INFO_ONCE("BASE INTERFACE_READ");
          boost::asio::async_read(*interfacePort, boost::asio::buffer(receivedData.get(), MAX_SERIAL_READ),
                                          boost::bind(&base_serial_interface::handleIORequest,this,
                                                          boost::asio::placeholders::error(),
                                                          boost::asio::placeholders::bytes_transferred()));
        }
    }
    return true;
}

bool base_classes::base_serial_interface::stopWork()
{
    if(interfaceStarted)
    {
        pluginStop();
        interfacePort->cancel();
        interfacePort->close();
        interfaceStarted = false;
        return !interfacePort->is_open();
    }
    return false;
}

bool base_classes::base_serial_interface::handleRegexRequest(const boost::system::error_code& e, std::size_t bytesTransferred)
{
	printMetrics(true);
  ROS_DEBUG("Thread <%s>:: %s:: Received Packet!:: Size %lu", THREAD_ID_TO_C_STR, this->pluginName.c_str(), bytesTransferred);

	if (!e)
	{
		std::istream is(&interfaceRegexBuffer);
		std::getline(is, receivedRegexData);

    boost::smatch result;
    if (boost::regex_search(receivedRegexData, result, regexExpr)) {
      std::string submatch(result[0].first, result[0].second);
      receivedRegexData = submatch;
    }

		ROS_DEBUG("Data -> %s", receivedRegexData.c_str());

    if(!interfaceReadHandler(bytesTransferred, dataArrayStart, e))
    {
        ROS_ERROR("Error Occurred in data handler for plugin <%s>", this->pluginName.c_str());
    }

	} else {
		ROS_ERROR("Error Occured in plugin data handler <%s>", e.message().c_str());
	}

	//restart the work
    return startWork();
}

bool base_classes::base_serial_interface::handleIORequest(const boost::system::error_code &ec, size_t bytesReceived)
{
    printMetrics(true);
    ROS_DEBUG("Thread <%s>:: %s:: Received Packet!:: Size %lu", THREAD_ID_TO_C_STR, this->pluginName.c_str(), bytesReceived);

    //TODO: in future, copy buffer contents to prevent data races (if any data race can happen)

    if(bytesReceived >= MAX_SERIAL_READ)
    {
        ROS_WARN("Buffer Filled! This shouldn't happen, most likely program fell behind or erroneous data on the port. Skipping Buffer Read.");
        return startWork();
    }

    //call plugin's data handler
    //SHORTCUT, if the first boolean check fails, the other is not called and avoids the worry of a null pointer
    if(!interfaceReadHandler(bytesReceived, dataArrayStart, ec))
    {
        ROS_ERROR("Error Occurred in plugin data Handler <%s>", this->pluginName.c_str());
    }

    //restart the work
    return startWork();
}

void base_classes::base_serial_interface::postInterfaceWriteRequest(const hw_interface_support_types::shared_const_buffer &buffer)
{
    ROS_DEBUG("%s:: Requesting interface write", pluginName.c_str());
    interfaceSynchronousStrand->post(boost::bind(&base_serial_interface::interfaceWriteHandler, this,
                                                    buffer));
}

void base_classes::base_serial_interface::interfaceWriteHandler(const hw_interface_support_types::shared_const_buffer &buffer)
{
    ROS_DEBUG("%s:: Writing Commands to interface", pluginName.c_str());
    try {
        boost::asio::write(*interfacePort, buffer);
    }
    catch(boost::system::system_error &ec)
    {
        ROS_ERROR_THROTTLE(5,"%s:: Caught Exception on WRITE!! %s", pluginName.c_str(), ec.what());
    }
    ROS_DEBUG("%s:: Writing done", pluginName.c_str());
}
