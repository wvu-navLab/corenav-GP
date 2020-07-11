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

#include <hw_interface_plugin_roboteq/hw_interface_plugin_roboteq.hpp>

hw_interface_plugin_roboteq::roboteq_serial::roboteq_serial()
{
    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info))
    {
        ros::console::notifyLoggerLevelsChanged();
    }
    ROS_INFO_EXTRA_SINGLE("Roboteq Plugin Instantiated");
    m_exStop=true;
}

bool hw_interface_plugin_roboteq::roboteq_serial::subPluginInit(ros::NodeHandlePtr nhPtr)
{
    nh = ros::NodeHandlePtr(nhPtr);

    std::string tempString = "";
    if(ros::param::get(pluginName+"/controllerType", tempString))
    {

        if (!tempString.compare("Left_Drive")) { roboteqType = controller_t::Left_Drive_Roboteq; }
        else if(!tempString.compare("Right_Drive")) { roboteqType = controller_t::Right_Drive_Roboteq; }
        else { roboteqType = controller_t::Other; }
    }
    else
    {
        ROS_ERROR("%s:: No Controller Type Specifiecompleted!", pluginName.c_str());
    }

    ROS_DEBUG("%s:: Implementation Init", pluginName.c_str());
    implInit();
    ROS_INFO_EXTRA_SINGLE("Roboteq Plugin Init");
    enableMetrics();

    enableRegexReadUntil = true;
    regexExpr = "(C|A|AI|AIC|BS|DI|DR|F|FF|CR|BA|VAR|V){1}=((-?\\d+):)+(-?\\d+)+((\\r){2})";
    m_numCmdsMatched = 0;

    deviceName = "";
    ros::param::get(pluginName+"/deviceName", deviceName);

    return true;
}

void hw_interface_plugin_roboteq::roboteq_serial::rosMsgCallback(const hw_interface_plugin_roboteq::ActuatorOut::ConstPtr &msgIn)
{
    std::string motorSpeedCmds = "";

    if(roboteqType == controller_t::Left_Drive_Roboteq)
    {
        motorSpeedCmds += "!G 2 " + boost::lexical_cast<std::string>(msgIn->fl_speed_cmd) + "\r";
        motorSpeedCmds += "!G 1 " + boost::lexical_cast<std::string>(msgIn->bl_speed_cmd) + "\r";
        postInterfaceWriteRequest(hw_interface_support_types::shared_const_buffer(motorSpeedCmds));
    }
    else if(roboteqType == controller_t::Right_Drive_Roboteq)
    {
        motorSpeedCmds += "!G 2 " + boost::lexical_cast<std::string>(msgIn->fr_speed_cmd) + "\r";
        motorSpeedCmds += "!G 1 " + boost::lexical_cast<std::string>(msgIn->br_speed_cmd) + "\r";
        postInterfaceWriteRequest(hw_interface_support_types::shared_const_buffer(motorSpeedCmds));
    }
    else
    {
        ROS_WARN_THROTTLE(2,"%s:: No Data written because of Incorrect Roboteq Type", pluginName.c_str());
    }

    ROS_DEBUG("%s", motorSpeedCmds.c_str());

    //need to add monitoring facilities to monitor health
}

bool hw_interface_plugin_roboteq::roboteq_serial::implInit()
{
    std::string tempString;
    if(ros::param::get(pluginName+"/subscribeToTopic", tempString))
    {
      rosDataSub = nh->subscribe(tempString, 1, &roboteq_serial::rosMsgCallback, this);
    }
    else
    {
        ROS_ERROR("%s:: Could not find topic subscription name", pluginName.c_str());
    }

    if(ros::param::get(pluginName+"/publishToTopic", tempString))
    {
      rosDataPub = nh->advertise<hw_interface_plugin_roboteq::RoboteqData>(tempString, 1, false);
    }
    else
    {
        ROS_ERROR("%s:: Could not find topic advertisment name", pluginName.c_str());
    }

    //need to start async timers here for grabber monitoring
    return true;
}

std::string hw_interface_plugin_roboteq::roboteq_serial::getInitCommands(std::string initializationCmd, int initCmdCycle)
{
  boost::char_separator<char> seperator(" :/,");
  tokenizer tokens(initializationCmd, seperator);
  tokenizer::iterator tok_iter = tokens.begin();

  initializationCmd = "\r^ECHOF 1\r# C\r";
  m_numInitCmds = 0;

  while ( tok_iter != tokens.end() )
  {
    ROS_DEBUG("Init Cmd -> %s", tok_iter->c_str());

    if (!command_list[*tok_iter].length())
    {
      ROS_WARN("RoboteQ command << %s >> was not found", tok_iter->c_str());
    }
    else
    {
      initializationCmd += "?" + command_list[*tok_iter] + "\r";
      ++m_numInitCmds;
    }

    ++tok_iter;
  }
  if (!m_numInitCmds)
  {
    m_numInitCmds = 1;
  }
  initCmdCycle = initCmdCycle / m_numInitCmds;
  std::string cycle = boost::lexical_cast<std::string>(initCmdCycle);
  return initializationCmd += "# " + cycle + " \r";
}

void hw_interface_plugin_roboteq::roboteq_serial::setInterfaceOptions()
{
	int tempBaudRate = 0;
    ros::param::get(pluginName+"/baudrate", tempBaudRate);
    setOption<boost::asio::serial_port_base::baud_rate>(
                new boost::asio::serial_port_base::baud_rate(tempBaudRate));
    //8 bits per character
    setOption<boost::asio::serial_port_base::character_size>(
    			new boost::asio::serial_port_base::character_size( 8 ));

    //flow control
    setOption<boost::asio::serial_port_base::flow_control>(
    			new boost::asio::serial_port_base::flow_control(
    										boost::asio::serial_port_base::flow_control::type::none));

    //parity
    setOption<boost::asio::serial_port_base::parity>(
    			new boost::asio::serial_port_base::parity(
    										boost::asio::serial_port_base::parity::type::none));

    //stop-bits
    setOption<boost::asio::serial_port_base::stop_bits>(
    			new boost::asio::serial_port_base::stop_bits(
    										boost::asio::serial_port_base::stop_bits::type::one));

    //ROS_INFO("%s :: Device: %s :: Baudrate %d", pluginName.c_str(), deviceName.c_str(), tempBaudRate);
}

bool hw_interface_plugin_roboteq::roboteq_serial::interfaceReadHandler(const size_t &length, int arrayStartPos, const boost::system::error_code &ec)
{
    ROS_DEBUG_THROTTLE(2,"Roboteq Plugin Data Handler");

    ROS_DEBUG("\n\nContents: %s\n", receivedRegexData.c_str());

    boost::char_separator<char> sep("= :\r\n");
    tokenizer tokens(receivedRegexData, sep);

    try
    {
      tokenizer::iterator tok_iter = tokens.begin();

      if(tok_iter != tokens.end())
      {
        m_command = tok_iter->c_str();
        ++tok_iter;
      }

      if(!dataHandler(tok_iter, tokens))
      {
        ROS_ERROR("%s :: Implementation Data Handler returned a BAD Return", pluginName.c_str());
      }
    }
    catch (const boost::bad_lexical_cast& e ){
      ROS_ERROR("%s:: Caught bad lexical cast with error %s", pluginName.c_str(), e.what());
    }
    catch(...){
      ROS_ERROR("%s:: Caught Unknown Error while parsing packet in data handler", pluginName.c_str());
    }


    return true;
}

bool hw_interface_plugin_roboteq::roboteq_serial::dataHandler(tokenizer::iterator tok_iter, tokenizer tokens)
{
  try{

    if (!m_command.compare("A"))
    {
      roboteqData.motor_amps.clear();
      while ( tok_iter != tokens.end() )
      {
        int16_t value = boost::lexical_cast<int16_t>(tok_iter->c_str());
        roboteqData.motor_amps.push_back(value);
        ++tok_iter;
      }
    }
    else if (!m_command.compare("AI"))
    {
      roboteqData.analog_inputs.clear();
      while ( tok_iter != tokens.end() )
      {
        int16_t value = boost::lexical_cast<int16_t>(tok_iter->c_str());
        roboteqData.analog_inputs.push_back(value);
        ++tok_iter;
      }
    }
    else if (!m_command.compare("AIC"))
    {
      roboteqData.analog_inputs_conversion.clear();
      while ( tok_iter != tokens.end() )
      {
        int16_t value = boost::lexical_cast<int16_t>(tok_iter->c_str());
        roboteqData.analog_inputs_conversion.push_back(value);
        ++tok_iter;
      }
    }
    else if (!m_command.compare("BA"))
    {
      roboteqData.battery_amps.clear();
      while ( tok_iter != tokens.end() )
      {
        int16_t value = boost::lexical_cast<int16_t>(tok_iter->c_str());
        roboteqData.battery_amps.push_back(value);
        ++tok_iter;
      }
    }
    else if (!m_command.compare("CR"))
    {
      roboteqData.encoder_count_relative.clear();
      while ( tok_iter != tokens.end() )
      {
        int32_t value = boost::lexical_cast<int32_t>(tok_iter->c_str());
        roboteqData.encoder_count_relative.push_back(value);
        ++tok_iter;
      }
    }
    else if (!m_command.compare("BS"))
    {
      roboteqData.bl_motor_speed_rpm.clear();
      while ( tok_iter != tokens.end() )
      {
        int16_t value = boost::lexical_cast<int16_t>(tok_iter->c_str());
        roboteqData.bl_motor_speed_rpm.push_back(value);
        ++tok_iter;
      }
    }
    else if (!m_command.compare("C"))
    {
      roboteqData.encoder_counter_absolute.clear();
      while ( tok_iter != tokens.end() )
      {
        int32_t value = boost::lexical_cast<int32_t>(tok_iter->c_str());
        roboteqData.encoder_counter_absolute.push_back(value);
        ++tok_iter;
      }
    }
    else if (!m_command.compare("DI"))
    {
      roboteqData.individual_digital_inputs.clear();
      while ( tok_iter != tokens.end() )
      {
        bool value = boost::lexical_cast<bool>(tok_iter->c_str());
        roboteqData.individual_digital_inputs.push_back(value);
        ++tok_iter;
      }
    }
    else if (!m_command.compare("DR"))
    {
      roboteqData.destination_reached.clear();
      while ( tok_iter != tokens.end() )
      {
        uint8_t value = boost::lexical_cast<uint8_t>(tok_iter->c_str());
        roboteqData.destination_reached.push_back(value);
        ++tok_iter;
      }
    }
    else if (!m_command.compare("FF"))
    {
      uint8_t value = boost::lexical_cast<uint8_t>(tok_iter->c_str());
      roboteqData.fault_flags = value;
    }
    else if (!m_command.compare("F"))
    {
      roboteqData.feedback.clear();
      while ( tok_iter != tokens.end() )
      {
        int16_t value = boost::lexical_cast<int16_t>(tok_iter->c_str());
        roboteqData.feedback.push_back(value);
        ++tok_iter;
      }
    }
    else if (!m_command.compare("VAR"))
    {
      roboteqData.user_integer_variable.clear();
      while ( tok_iter != tokens.end() )
      {
        int32_t value = boost::lexical_cast<int32_t>(tok_iter->c_str());
        roboteqData.user_integer_variable.push_back(value);
        ++tok_iter;
      }
    }
    else if (!m_command.compare("V"))
    {
      roboteqData.volts.clear();
      while ( tok_iter != tokens.end() )
      {
        uint16_t value = boost::lexical_cast<uint16_t>(tok_iter->c_str());
        roboteqData.volts.push_back(value);
        ++tok_iter;
      }
    }
    else
    {
      ROS_ERROR_EXTRA("RoboteQ Data Handler found no match for << %s >> command", m_command.c_str());
      return false;
    }
    ++m_numCmdsMatched;
  }
  catch(const std::exception &ex)
  {
      ROS_ERROR_EXTRA("STD Exception Caught! \r\n %s", ex.what());
      ROS_ERROR("REGEX Container %s", receivedRegexData.c_str());
  }

  if (m_numCmdsMatched >= m_numInitCmds)
  {
    rosDataPub.publish(roboteqData);
    m_numCmdsMatched = 0;
  }
  return true;
}

bool hw_interface_plugin_roboteq::roboteq_serial::verifyChecksum()
{
    return true;
}

typedef boost::asio::buffers_iterator<boost::asio::streambuf::const_buffers_type> matcherIterator;

std::pair<matcherIterator, bool>
hw_interface_plugin_roboteq::roboteq_serial::matchFooter(matcherIterator begin, matcherIterator end, const char* sequence)
{
    int i = 0;
    for(matcherIterator footerIt = (end-std::strlen(sequence)); footerIt!=end; footerIt++)
    {
        if(*footerIt!=sequence[i])
        {
            return std::make_pair(begin, false);
        }
        i++;
    }
    return std::make_pair(begin, true);
}
