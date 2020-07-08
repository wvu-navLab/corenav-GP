#include <ros/ros.h>

#include <iostream>

#include <hw_interface/hw_interface.hpp>

hw_interface::hw_interface() :
    pluginLoader("hw_interface", "base_classes::base_interface")
{
    node = ros::NodeHandlePtr(new ros::NodeHandle());
    ROS_DEBUG_EXTRA_SINGLE("Creating ASIO Services");
    interfaceService = boost::shared_ptr<boost::asio::io_service>
                                    (new boost::asio::io_service);
    ROS_DEBUG_EXTRA_SINGLE("Generating Work");
    interfaceWork = boost::shared_ptr<boost::asio::io_service::work>
                                    (new boost::asio::io_service::work(*interfaceService));

    ROS_DEBUG_EXTRA_SINGLE("Loading Plugins");
    addInterfacePlugins();

    ROS_DEBUG_EXTRA_SINGLE("Starting Thread Pool");
    initThreadPool();
    ROS_DEBUG_EXTRA_SINGLE("Starting Interfaces");
    startInterfaces();
}

hw_interface::hw_interface(ros::NodeHandlePtr nhArg) :
    node(nhArg),
    pluginLoader("hw_interface", "base_classes::base_interface")
{
    ROS_DEBUG_EXTRA_SINGLE("Creating ASIO Services");
    interfaceService = boost::shared_ptr<boost::asio::io_service>
                                    (new boost::asio::io_service);
    ROS_DEBUG_EXTRA_SINGLE("Generating Work");
    interfaceWork = boost::shared_ptr<boost::asio::io_service::work>
                                    (new boost::asio::io_service::work(*interfaceService));

    ROS_DEBUG_EXTRA_SINGLE("Loading Plugins");
    addInterfacePlugins();

    ROS_DEBUG_EXTRA_SINGLE("Starting Thread Pool");
    initThreadPool();
    ROS_DEBUG_EXTRA_SINGLE("Starting Interfaces");
    startInterfaces();
}

hw_interface::~hw_interface()
{
    std::printf("HW Interfaces are stopping.\r\n");
    //interfaceWork.reset();
    node.reset();
    interfaceService->stop();
    interfaceWorkerGroup.join_all();
    //interfaceService->reset();

    for(std::vector<boost::shared_ptr<base_classes::base_interface> >::iterator vectorIterator = interfacePluginVector.begin();
            vectorIterator < interfacePluginVector.end();
            /*vectorIterator++*/)
    {
        std::printf("Destroying Plugin: %s\r\n", vectorIterator->get()->pluginName.c_str());
        vectorIterator->get()->stopWork();
        vectorIterator = interfacePluginVector.erase(vectorIterator);
        //std::printf("plugin destroyed\r\n");
    }
}

bool hw_interface::initPlugin(boost::shared_ptr<base_classes::base_interface> pluginPtr, std::string pluginName)
{
    try
    {
        pluginPtr->pluginName = pluginName;
        ROS_INFO("Initilizing Plugin: %s", pluginPtr->pluginName.c_str());
        pluginPtr->initPlugin(node, interfaceService);
    }
    catch(const boost::system::error_code &ec)
    {
        ROS_ERROR_EXTRA("Boost Exception Caught! \r\n %s", ec.message().c_str());
        ROS_ERROR_EXTRA("Disabling Plugin: %s", pluginPtr->pluginName.c_str());
        pluginPtr->enabled = false;
    }
    catch(const std::exception &ex)
    {
        ROS_ERROR_EXTRA("STD Exception Caught! \r\n %s", ex.what());
        ROS_ERROR_EXTRA("Disabling Plugin: %s", pluginPtr->pluginName.c_str());
        pluginPtr->enabled = false;
    }
    return pluginPtr->enabled;
}

//needs to be finished
//MUST CALL initInterfacePlugins AFTER io_service creation, not before!!!!
void hw_interface::addInterfacePlugins()
{
    ROS_DEBUG_EXTRA_SINGLE("Looking for plugins on Param Server");
    std::map<std::string, std::string> pluginMap;
    if(ros::param::get("/hw_interface/plugin_names", pluginMap))
    {
        for(std::map<std::string,std::string>::iterator mapIterator = pluginMap.begin();
                mapIterator != pluginMap.end();
                mapIterator++)
        {
            try
            {
                std::string pluginClassName = mapIterator->second + "::" + mapIterator->first;
                ROS_INFO("Found Plugin Class %s", pluginClassName.c_str());
                ROS_INFO("Description: %s", pluginLoader.getClassDescription(pluginClassName).c_str());
                std::map<std::string, std::string> instanceNameMap;
                if(ros::param::get(mapIterator->first, instanceNameMap))
                {
                    for(std::map<std::string, std::string>::iterator nameIterator = instanceNameMap.begin();
                            nameIterator != instanceNameMap.end();
                            nameIterator++)
                    {
                        //ros param with name of derived class
                        interfacePluginVector.push_back(pluginLoader.createInstance(pluginClassName.c_str()));

                        //if the plugin ptr is invalid OR the plugin initilization returns false, remove plugin
                        if(!interfacePluginVector.back().get() ||
                                !initPlugin(interfacePluginVector.back(), nameIterator->second)) //if the last added plugin is valid
                        {
                            ROS_ERROR("The plugin failed to instantiate");
                            interfacePluginVector.pop_back();
                        }
                        else
                        {
                            ROS_INFO("Instatiated Interface Plugin: %s", interfacePluginVector.back()->pluginName.c_str());
                        }
                    }
                }
                else
                {
                    ROS_ERROR("Could not find plugin name list for %s !!", pluginClassName.c_str());
                }
            }
            catch(pluginlib::PluginlibException& ex)
            {
                ROS_ERROR_EXTRA("The plugin failed to load for some reason.\r\nError: %s", ex.what());
            }
        }
    }
    else
    {
        ROS_ERROR_EXTRA_SINGLE("NO PLUGINS TO LOAD!");
        ROS_ERROR_EXTRA_SINGLE("Check Parameter Server for correct YAML dictionary entries");
    }
}

void hw_interface::initThreadPool()
{
    ROS_INFO("Starting Thread Pool");
    for(int i = 0; i < interfacePluginVector.size() * NUM_THREADS_PER_PLUGIN; i++)
    {
        interfaceWorkerGroup.create_thread(boost::bind(&interface_worker::worker, interfaceService));
    }
}

void hw_interface::initInterfacePlugins()
{
    for(std::vector<boost::shared_ptr<base_classes::base_interface> >::iterator vectorIterator = interfacePluginVector.begin();
            vectorIterator < interfacePluginVector.end();
            vectorIterator++)
    {
        try
        {
            ROS_INFO("Initilizing Plugin: %s", vectorIterator->get()->pluginName.c_str());
            vectorIterator->get()->initPlugin(node, interfaceService);
        }
        catch(const boost::system::error_code &ec)
        {
            ROS_ERROR_EXTRA("Boost Exception Caught! \r\n %s", ec.message().c_str());
            ROS_ERROR_EXTRA("Disabling Plugin: %s", vectorIterator->get()->pluginName.c_str());
            vectorIterator = interfacePluginVector.erase(vectorIterator);
        }
        catch(const std::exception &ex)
        {
            ROS_ERROR_EXTRA("STD Exception Caught! \r\n %s", ex.what());
            ROS_ERROR_EXTRA("Disabling Plugin: %s", vectorIterator->get()->pluginName.c_str());
            vectorIterator = interfacePluginVector.erase(vectorIterator);
        }
    }
}

bool hw_interface::startInterfaces()
{
    for(std::vector<boost::shared_ptr<base_classes::base_interface> >::iterator vectorIterator = interfacePluginVector.begin();
            vectorIterator < interfacePluginVector.end();
            vectorIterator++)
    {
        ROS_INFO("Starting Plugin Work: %s", vectorIterator->get()->pluginName.c_str());
        bool returnValue = vectorIterator->get()->startWork();
        ROS_INFO("Plugin Start return %d", returnValue);
    }
}

bool hw_interface::stopInterfaces()
{
    for(std::vector<boost::shared_ptr<base_classes::base_interface> >::iterator vectorIterator = interfacePluginVector.begin();
            vectorIterator < interfacePluginVector.end();
            vectorIterator++)
    {
        ROS_INFO("Stopping Plugin Work: %s", vectorIterator->get()->pluginName.c_str());
        vectorIterator->get()->stopWork();
    }
}

void interface_worker::worker(boost::shared_ptr<boost::asio::io_service> ioService)
{
    ROS_DEBUG("Thread <%s>:: Started Worker", THREAD_ID_TO_C_STR);


    //should change to poll if we want to profile individual thread performance
    while(!ioService->stopped())
    {
        try{
            ioService->run();
        }
        catch(const boost::system::error_code &ec)
        {
            ROS_ERROR("Exception Caught! \r\n %s", ec.message().c_str());
        }
        catch(const std::exception &ex)
        {
            ROS_ERROR("STD Exception Caught! \r\n %s", ex.what());
        }
        ROS_WARN("Thread <%s>:: Waiting for Work\r\n", THREAD_ID_TO_C_STR);
        ros::Duration pause(1);
        pause.sleep();
    }

    std::printf("Thread <%s>:: Stopping Worker\r\n", THREAD_ID_TO_C_STR);
}
