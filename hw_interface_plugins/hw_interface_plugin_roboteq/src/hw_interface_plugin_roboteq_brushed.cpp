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

#include <hw_interface_plugin_roboteq/hw_interface_plugin_roboteq_brushed.hpp>

hw_interface_plugin_roboteq::brushed::brushed()
{
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info))
  {
    ros::console::notifyLoggerLevelsChanged();
  }
}

bool hw_interface_plugin_roboteq::brushed::implStart()
{
  ROS_INFO("%s:: Plugin start!", pluginName.c_str());

  /*
   * Roboteq usage
   * 1. Disable Command Echo
   * 2. send '# C' to clear history buffer
   * 3. send '?AIC' read analog sensor after conversion
   * 4. send '# 20' to have runtime queries repeated at 20 ms delta (50 hz)
   */
  std::string initializationCmd = "";
  int initCmdCycle = 0;
  if(ros::param::get(pluginName+"/initializationCmd", initializationCmd))
  {
    if (!ros::param::get(pluginName+"/initCmdCycle", initCmdCycle))
    {
      initCmdCycle = 20;
    }
    initializationCmd = hw_interface_plugin_roboteq::roboteq_serial::getInitCommands(initializationCmd, initCmdCycle);
  }
  else
  {
    ROS_WARN("Roboteq Initialization Command Unspecified, defaulting");
    initializationCmd = "\r^ECHOF 1\r# C\r?AIC\r# 20\r";
  }
  ROS_INFO("Roboteq Init Cmd %s", initializationCmd.c_str());
  postInterfaceWriteRequest(hw_interface_support_types::shared_const_buffer(initializationCmd));

  return true;
}

bool hw_interface_plugin_roboteq::brushed::implStop()
{
    ROS_INFO("%s:: Plugin stop!", pluginName.c_str());
    return true;
}

bool hw_interface_plugin_roboteq::brushed::implDataHandler()
{
    ROS_DEBUG("%s :: Roboteq Brushed Implementation Data Handler", pluginName.c_str());
    //should check size of buffer is equal to size of msg, just in case.

    return true;
}
