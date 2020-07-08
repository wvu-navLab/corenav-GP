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

#ifndef BASE_INTERFACE_HPP__
#define BASE_INTERFACE_HPP__


#include <ros/ros.h>

#include <boost/system/error_code.hpp>
#include <boost/smart_ptr.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include <hw_interface/shared_const_buffer.hpp>

#include <boost/cstdint.hpp>
#include <boost/crc.hpp>
#include <cstdint>

#include <boost/filesystem/path.hpp>

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/rolling_mean.hpp>

#define THREAD_ID_TO_C_STR boost::lexical_cast<std::string>(boost::this_thread::get_id()).c_str()
#define FILENAME_C_STR boost::filesystem::path(__FILE__).filename().c_str()
#define LINENUMBER_C_STR boost::lexical_cast<std::string>(__LINE__).c_str()
#define ROS_INFO_EXTRA_SINGLE(arg1  ) ROS_INFO("%s %s:: " arg1 ,FILENAME_C_STR, LINENUMBER_C_STR)
#define ROS_INFO_EXTRA(arg1, args...) ROS_INFO("%s %s:: " arg1 ,FILENAME_C_STR, LINENUMBER_C_STR, args)
#define ROS_INFO_EXTRA_NAME(arg1, args...) ROS_INFO("%s:: " arg1 , pluginName.c_str(), args)
#define ROS_DEBUG_EXTRA_NAME(arg1, args...) ROS_DEBUG("%s:: " arg1 , pluginName.c_str(), args)
#define ROS_WARN_EXTRA_NAME(arg1, args...) ROS_WARN("%s:: " arg1 , pluginName.c_str(), args)
#define ROS_ERROR_EXTRA_NAME(arg1, args...) ROS_ERROR("%s:: " arg1 , pluginName.c_str(), args)
#define ROS_ERROR_EXTRA_SINGLE(arg1  ) ROS_ERROR("%s %s:: " arg1 ,FILENAME_C_STR, LINENUMBER_C_STR)
#define ROS_ERROR_EXTRA(arg1, args...) ROS_ERROR("%s %s:: " arg1 ,FILENAME_C_STR, LINENUMBER_C_STR, args)
#define ROS_DEBUG_EXTRA_SINGLE(arg1  ) ROS_DEBUG("%s %s:: " arg1 ,FILENAME_C_STR, LINENUMBER_C_STR)
#define ROS_DEBUG_EXTRA(arg1, args...) ROS_DEBUG("%s %s:: " arg1 ,FILENAME_C_STR, LINENUMBER_C_STR, args)
#define ROS_WARN_EXTRA_SINGLE(arg1  ) ROS_WARN("%s %s:: " arg1 ,FILENAME_C_STR, LINENUMBER_C_STR)
#define ROS_WARN_EXTRA(arg1, args...) ROS_WARN("%s %s:: " arg1 ,FILENAME_C_STR, LINENUMBER_C_STR, args)
#define NUM_BUFFERS_PER_PLUGIN 4

namespace base_classes
{
    enum interfaceType_t {Serial, UDP, Custom};

    class base_interface
    {

    public:
        typedef boost::asio::buffers_iterator<boost::asio::streambuf::const_buffers_type> matcherIterator;
        //const boost::shared_ptr<boost::asio::io_service> interfaceService

        std::string pluginName;
        interfaceType_t interfaceType;
        bool interfaceStarted;
        bool enabled;

        bool enableCompletionFunctor;
        bool enableStreamMatcher;
        bool enableRegexReadUntil; //enabled in roboteq plugin

        ros::Publisher rosDataPub; //publisher, data from interface to ros
        ros::Subscriber rosDataSub;//subscriber, data from ros to interface

        //This ASIO strand is used for writing data to the interface.
        //If this object is not used for writing data to the interface, data writes could happen
        //  in any order, meaning output data packets can be sent in reverse order (bad for control packets).
        boost::shared_ptr<boost::asio::strand> interfaceSynchronousStrand;

        boost::function<std::size_t(const boost::system::error_code& error,
                                    std::size_t totalBytesRecevied)> streamCompletionChecker;
        boost::function<std::pair<matcherIterator, bool> (matcherIterator begin, matcherIterator end) >
                                    streamSequenceMatcher;

        //hw_interface will call to check if work can continue
        virtual bool interfaceReady() {}

        //called after io_service init
        virtual bool initPlugin(ros::NodeHandlePtr nhPtr,
                                const boost::shared_ptr<boost::asio::io_service> ioService) {}

        //called after init, used to start interface and restart listen
        virtual bool startWork() {}

        //called to stop interface
        virtual bool stopWork() {}

        //called by hw_interface
        virtual bool verifyChecksum() {}

        //beware that size_t is UNSIGNED. Operations should not underflow!
        virtual bool handleIORequest(const boost::system::error_code &ec, size_t bytesReceived) {}

        virtual void postInterfaceWriteRequest(const hw_interface_support_types::shared_const_buffer &buffer) {}
        virtual void interfaceWriteHandler(const hw_interface_support_types::shared_const_buffer &buffer, const boost::system::error_code &ec) {}

        static uint16_t calcCRC16Block(const void * const buf, std::size_t numOfBytes);
        static uint32_t calcCRC32Block(const void * const buf, std::size_t numOfBytes);

        void setupStreamMatcherDelimAndLength(const int packetLengthInBytes, const char *headerSequence,
                                              const char *footerSequence)
        {
            streamCompletionChecker =
                    boost::bind(&base_interface::streamMatcherDelimAndLength, this,
                                    _1, _2, packetLengthInBytes, headerSequence,
                                    footerSequence);
            enableCompletionFunctor =! streamCompletionChecker.empty();
        }


        //this definition is called repeatedly and used to check if a packet on the stream
        //has been received.
        //a return of 0 means the read has completed and the ASIO Services should call the plugin IO Handle

        //Plugins should overide this function if it intends on using this functionality
        std::size_t streamMatcherDelimAndLength(const boost::system::error_code &error, long totalBytesInBuffer,
                                                    const int &packetLengthInBytes, const char *headerSequence,
                                                    const char *footerSequence);

        bool enableMetrics();
        bool disableMetrics();
        std::string printMetrics(bool printSideEffect);

        base_interface();
        virtual ~base_interface(){}

    protected:
        boost::shared_array<uint8_t> receivedData;
        int dataArrayStart;

    private:

        bool metricsEnabled;

        //boost::mutex metricMuxtex;
        //ros::Time does not need a thread lock because it is actually locked internally
        boost::accumulators::accumulator_set<double, boost::accumulators::stats<boost::accumulators::tag::rolling_mean> > acc;
        ros::Time lastTimeMetric;
        double deltaTime;
    };
};

#endif //BASE_INTERFACE_HPP__
