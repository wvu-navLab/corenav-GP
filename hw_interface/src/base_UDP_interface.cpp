#include <ros/ros.h>

#include <hw_interface/base_UDP_interface.hpp>

#define THREAD_ID_TO_C_STR boost::lexical_cast<std::string>(boost::this_thread::get_id()).c_str()

base_classes::base_UDP_interface::base_UDP_interface()
{
    ROS_DEBUG_EXTRA_SINGLE("UDP Plugin initialized");
    interfaceType = base_classes::UDP;
    interfaceStarted = false;

    receivedData = boost::shared_array<uint8_t>(new uint8_t[UDP_MAX_PKT_SIZE]);
    remoteEndpoint = boost::shared_ptr<boost::asio::ip::udp::endpoint>();
    localEndpoint = boost::shared_ptr<boost::asio::ip::udp::endpoint>();
    interfaceSocket = boost::shared_ptr<boost::asio::ip::udp::socket>();
}

bool base_classes::base_UDP_interface::interfaceReady()
{
    if(!interfaceSocket.get())
    {
        return false;
    }
    return interfaceSocket->is_open();
}

bool base_classes::base_UDP_interface::initPlugin(ros::NodeHandlePtr nhPtr,
                                                    const boost::shared_ptr<boost::asio::io_service> ioService)
{
    enableCompletionFunctor = false;
    enableRegexReadUntil = false;
    //initilize output data stream strand
    interfaceSynchronousStrand = boost::shared_ptr<boost::asio::strand>(new boost::asio::strand(*ioService));
    //call plugin setup
    ROS_DEBUG_EXTRA_SINGLE("Calling Plugin's Init");
    subPluginInit(nhPtr);

    ROS_DEBUG_EXTRA_SINGLE("Creating Endpoints");
    remoteEndpoint = boost::shared_ptr<boost::asio::ip::udp::endpoint>(new boost::asio::ip::udp::endpoint(remoteAddress, remotePort));
    //create the local endpoint
    localEndpoint = boost::shared_ptr<boost::asio::ip::udp::endpoint>(new boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(), localPort));
    //open, bind, and connect to the local endpoint
    ROS_DEBUG_EXTRA_SINGLE("Opening Local Socket");
    interfaceSocket = boost::shared_ptr<boost::asio::ip::udp::socket>(new boost::asio::ip::udp::socket(*ioService, *localEndpoint));
    ROS_DEBUG_EXTRA_SINGLE("Socket Created");
    return interfaceReady();
}

bool base_classes::base_UDP_interface::startWork()
{
    if(!interfaceReady())
    {
        return false;
    }

    if(!interfaceStarted)
    {
        interfaceStarted = pluginStart();
        ROS_INFO("Setting Socket send size to %d", UDP_MAX_PKT_SIZE);
        boost::asio::socket_base::send_buffer_size option(UDP_MAX_PKT_SIZE);
        interfaceSocket->set_option(option);
    }
    ROS_DEBUG("Starting UDP Work");

    interfaceSocket.get()->async_receive_from(boost::asio::buffer(receivedData.get(), UDP_MAX_PKT_SIZE), senderEndpoint,
                                            boost::bind(&base_UDP_interface::handleIORequest, this,
                                                            boost::asio::placeholders::error(),
                                                            boost::asio::placeholders::bytes_transferred()));
    return true;
}

bool base_classes::base_UDP_interface::stopWork()
{
    if(interfaceStarted)
    {
        //free all queued work
        interfaceSocket->cancel();
        //probably shouldn't close
        interfaceSocket->close();

        interfaceStarted = false;
        return !interfaceSocket->is_open();
    }
    return false;
}

bool base_classes::base_UDP_interface::handleIORequest(const boost::system::error_code &ec, size_t bytesReceived)
{
    //printMetrics(true);
    ROS_INFO_THROTTLE(5,"Thread <%s>:: %s:: Received Packet!:: Size %lu", THREAD_ID_TO_C_STR, this->pluginName.c_str(), bytesReceived);

    //call plugin's data handler
    dataArrayStart=0;
    if(!interfaceReadHandler(bytesReceived, dataArrayStart))
    {
        ROS_ERROR("Error Occurred in plugin data Handler <%s>", this->pluginName.c_str());
    }

    //restart the work
    return startWork();
}

void base_classes::base_UDP_interface::postInterfaceWriteRequest(const hw_interface_support_types::shared_const_buffer &buffer)
{
    ROS_DEBUG("%s:: Requesting interface write", pluginName.c_str());
    interfaceSynchronousStrand->post(boost::bind(&base_UDP_interface::interfaceWriteHandler, this,
                                                    buffer));
}

void base_classes::base_UDP_interface::interfaceWriteHandler(const hw_interface_support_types::shared_const_buffer &buffer)
{
    ROS_DEBUG("%s:: Writing Commands to interface", pluginName.c_str());
    try {
        //boost::system::system_error e();
        interfaceSocket->send_to(buffer,*remoteEndpoint);
    }
    catch(boost::system::system_error &ec)
    {
        ROS_ERROR("%s:: Caught Exception on WRITE!! %s", pluginName.c_str(), ec.what());
    }
}
