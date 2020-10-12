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

#ifndef BASE_UDP_INTERFACE_HPP__
#define BASE_UDP_INTERFACE_HPP__


#include <ros/ros.h>

#include <hw_interface/base_interface.hpp>

#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include <boost/system/error_code.hpp>

#define UDP_FRAME_BUFFER_SIZE 1500
#define UDP_MAX_PKT_SIZE 65500

namespace base_classes
{
    class base_UDP_interface : public base_interface
    {
        //friend class hw_interface;

    private:

        bool interfaceReady();
        bool initPlugin(ros::NodeHandlePtr nhPtr,
                            const boost::shared_ptr<boost::asio::io_service> ioService);
        bool startWork(); //probably set some flag and start listening on port
        bool stopWork();  //probably unset some flag

        //will receive request, then run plugin readHandler(), check work flag,
            //if work flag restart async read

    protected:

        base_UDP_interface();

        boost::shared_ptr<boost::asio::ip::udp::socket> interfaceSocket;
        boost::shared_ptr<boost::asio::ip::udp::endpoint> localEndpoint;
        boost::shared_ptr<boost::asio::ip::udp::endpoint> remoteEndpoint;
        boost::asio::ip::udp::endpoint senderEndpoint;

        //needs better name
        //this function calls the plugin's function to read in ROS params,
        //subscribe to topics, publish topics. This function should fill
        //in the protected member's info
        virtual bool subPluginInit(ros::NodeHandlePtr nhPtr) = 0;

        virtual bool pluginStart()
        {
            return true;
        }

        virtual bool pluginStop()
        {
            return true;
        }

        boost::asio::ip::address localAddress;
        int localPort;
        boost::asio::ip::address remoteAddress;
        int remotePort;

        //plugin provided data handler that moves data into ROS
        virtual bool interfaceReadHandler(const size_t &bufferSize, int arrayStartPos) = 0;

        void interfaceWriteHandler(const hw_interface_support_types::shared_const_buffer &buffer);
        void postInterfaceWriteRequest(const hw_interface_support_types::shared_const_buffer &buffer);

    public:
        bool handleIORequest(const boost::system::error_code &ec, size_t bytesReceived);

    };
};
#endif //BASE_UDP_INTERFACE_HPP__
