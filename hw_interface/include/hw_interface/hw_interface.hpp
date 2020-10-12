/*********************************************************************
* Software License Agreement (BSD License)
*
* Copyright (c) 2019, WVU Interactive Robotics Laboratory
*                       https://web.statler.wvu.edu/~irl/
* All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the WVU Interactive Robotics Laboratory nor
*     the names of its contributors may be used to endorse or promote products
*     derived from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#ifndef HW_INTERFACE_HPP__
#define HW_INTERFACE_HPP__


#include <ros/ros.h>
#include <hw_interface/base_interface.hpp>
#include <hw_interface/base_serial_interface.hpp>
#include <hw_interface/base_UDP_interface.hpp>
#include <pluginlib/class_loader.h>

#include <boost/asio.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/bind.hpp>

#define NUM_THREADS_PER_PLUGIN 2

namespace interface_worker
{
    void worker(boost::shared_ptr<boost::asio::io_service> ioService);
};

class hw_interface {
    //friend class interface_worker;

private:
    ros::NodeHandlePtr node;

    boost::shared_ptr<boost::asio::io_service> interfaceService;
    boost::shared_ptr<boost::asio::io_service::work> interfaceWork;

    std::vector<boost::shared_ptr<base_classes::base_interface> > interfacePluginVector;

    boost::thread_group interfaceWorkerGroup;

    pluginlib::ClassLoader<base_classes::base_interface> pluginLoader;

    bool initPlugin(boost::shared_ptr<base_classes::base_interface> pluginPtr,
                        std::string pluginName);

    void initInterfacePlugins();
    void initThreadPool();

public:


    hw_interface();
    hw_interface(ros::NodeHandlePtr nhArg);
    //hw_interface(int maxNumOfThreads);

    virtual ~hw_interface();

    void addInterfacePlugins();

    bool startInterfaces();
    bool stopInterfaces();      //once node has stopped, reset interfaceWork ptr.




};


#endif //HW_INTERFACE_HPP__
