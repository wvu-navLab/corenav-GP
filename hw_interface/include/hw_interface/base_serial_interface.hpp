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

#ifndef BASE_SERIAL_INTERFACE_HPP__
#define BASE_SERIAL_INTERFACE_HPP__


#include <ros/ros.h>

#include <hw_interface/base_interface.hpp>

#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include <boost/system/error_code.hpp>

#include <boost/asio/basic_serial_port.hpp>
#include <boost/asio/serial_port.hpp>

#include <boost/asio/read_until.hpp>
#include <boost/regex.hpp>


#define MAX_SERIAL_READ 250

namespace base_classes
{
    class base_serial_interface : public base_interface
    {

    public:
        bool handleIORequest(const boost::system::error_code &ec, size_t bytesReceived);

        //handles ascii roboteq runtime queries
		    bool handleRegexRequest(const boost::system::error_code& e, std::size_t bytesTransferred);
        boost::regex regexExpr;
        std::string receivedRegexData;

        //this definition is used to check if a certain character sequence has been encountered
        //on the stream. The begin and end iterators represent positions in the stream that this
        //current functor is working. The return iterator represents where the next function call
        //(if needed) will begin in the buffer stream. The return bool indicates if the ASIO Service
        //is complete and should call the plugin IO Handler. (true = yes, call handler)

        //Plugins should overide this function if it intends on using this functionality

        virtual std::pair<matcherIterator, bool> matchFooter(matcherIterator begin, matcherIterator end, const char* sequence)
        {
            return std::make_pair(end, true);
        }

    private:

        bool interfaceReady();
        bool initPlugin(ros::NodeHandlePtr nhPtr,
                            const boost::shared_ptr<boost::asio::io_service> ioService);
        bool startWork(); //probably set some flag and start listening on port
        bool stopWork();  //probably unset some flag

        //will receive request, then run plugin readHandler(), check work flag,
            //if work flag restart async read

        //this ptr is used to pass the beginning of data to the plugin data handler.
        //  its void because we don't type discriminate around here.
        //But mainly so the plugin can interpret the data however it wants to.

    protected:
        boost::shared_ptr<boost::asio::serial_port> interfacePort;

        std::string deviceName;
        boost::asio::streambuf interfaceRegexBuffer;

        int readLength;
        std::string headerString, footerString;

        base_serial_interface();

        //needs better name
        //this function calls the plugin's function to read in ROS params,
        //subscribe to topics, publish topics. This function should fill
        //in the protected member's info
        virtual bool subPluginInit(ros::NodeHandlePtr nhPtr) = 0;
        virtual void setInterfaceOptions() = 0;

        virtual bool interfaceReadHandler(const size_t &bufferSize, int arrayStartPos, const boost::system::error_code &ec) = 0;

        void interfaceWriteHandler(const hw_interface_support_types::shared_const_buffer &buffer);
        void postInterfaceWriteRequest(const hw_interface_support_types::shared_const_buffer &buffer);

        virtual bool pluginStart()
        {
            return true;
        }

        virtual bool pluginStop()
        {
            return true;
        }

        template<typename Option>
        boost::system::error_code setOption(const Option * newOption)
        {
            boost::system::error_code ec;
            if(interfacePort.get()){
                return interfacePort->set_option<Option>(*newOption, ec);
            }
            ec.assign(static_cast<int>(boost::system::errc::no_such_device), boost::asio::error::get_system_category());
            return ec;
        }

        template<typename Option>
        Option getOption(){
            Option returnValue;
            if(interfacePort.get()){
                interfacePort->get_option<Option>(returnValue);
            }
            return returnValue;
        }
    };
};


#endif //BASE_SERIAL_INTERFACE_HPP__
