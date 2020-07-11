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

#ifndef HW_INTERFACE_PLUGIN_ROBOTEQ_HPP__
#define HW_INTERFACE_PLUGIN_ROBOTEQ_HPP__

#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>

#include <hw_interface/shared_const_buffer.hpp>
#include <hw_interface/base_interface.hpp>
#include <hw_interface/base_serial_interface.hpp>

#include <boost/tokenizer.hpp>
#include <iterator>
#include <boost/regex.hpp>
#include <map>

#include <fstream> // std::ifstream

#include <hw_interface_plugin_roboteq/ActuatorOut.h>
#include <hw_interface_plugin_roboteq/RoboteqData.h>

namespace hw_interface_plugin_roboteq {

    enum controller_t { Other, Left_Drive_Roboteq, Right_Drive_Roboteq };

   class roboteq_serial : public base_classes::base_serial_interface
   {
    public:
        typedef boost::asio::buffers_iterator<boost::asio::streambuf::const_buffers_type> matcherIterator;

        roboteq_serial();
        virtual ~roboteq_serial() {} //need to implement closing of the port here

    protected:
        ros::NodeHandlePtr nh;

        typedef boost::tokenizer<boost::char_separator<char> > tokenizer;
        std::string getInitCommands(std::string initializationCmd, int initCmdCycle);

        hw_interface_plugin_roboteq::ActuatorOut latestActuatorCmd;
        controller_t roboteqType;

        bool subPluginInit(ros::NodeHandlePtr nhPtr);
        void setInterfaceOptions();
        bool interfaceReadHandler(const size_t &length, int arrayStartPos, const boost::system::error_code &ec);
        bool verifyChecksum();

        bool pluginStart()
        {
            return implStart();
        }

        bool pluginStop()
        {
            return implStop();
        }

        bool implInit();
        void rosMsgCallback(const hw_interface_plugin_roboteq::ActuatorOut::ConstPtr &msgIn);
        std::string m_command;

        hw_interface_plugin_roboteq::RoboteqData roboteqData;

        virtual bool implStart() = 0;
        virtual bool implStop() = 0;
        virtual bool implDataHandler() = 0;

        std::map <std::string, std::string> command_list = {
          {"motor_amps", "A"},
          {"analog_inputs", "AI"},
          {"analog_inputs_conversion", "AIC"},
          {"bl_motor_speed_rpm", "BS"},
          {"individual_digital_inputs", "DI"},
          {"destination_reached", "DR"},
          {"fault_flags", "FF"},
          {"encoder_counter_absolute", "C"},
          {"encoder_count_relative", "CR"},
          {"battery_amps", "BA"},
          {"feedback", "F"},
          {"var_2", "VAR"},
          {"volts", "V"}
        };

        std::pair<matcherIterator, bool> matchFooter(matcherIterator begin, matcherIterator end, const char *sequence);

        std::string roboteqInit;
      private:
        int m_numInitCmds; // # of commands from launch file
        int m_numCmdsMatched; // # of commands matched regex from roboteqs

        bool m_exStop;

        bool dataHandler(tokenizer::iterator tok_iter, tokenizer tokens);

   };
}

#endif
