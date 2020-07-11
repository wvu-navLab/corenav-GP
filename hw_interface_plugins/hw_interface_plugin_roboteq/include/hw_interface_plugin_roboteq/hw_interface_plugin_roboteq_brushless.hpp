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

#ifndef HW_INTERFACE_PLUGIN_ROBOTEQ_BRUSHLESS_HPP__
#define HW_INTERFACE_PLUGIN_ROBOTEQ_BRUSHLESS_HPP__

#include <hw_interface_plugin_roboteq/hw_interface_plugin_roboteq.hpp>

namespace hw_interface_plugin_roboteq
{
  class brushless : public hw_interface_plugin_roboteq::roboteq_serial
  {
  public:
    brushless();

  protected:
    bool implStart();
    bool implStop();
    bool implDataHandler();

  private:
    std::map <std::string, std::string> script_list = {
      {"Reg_Left", 
          "# C\r"
          "^ALIM 1 220\r"
          "^ALIM 2 220\r"
          "^DINA 6 0\r"
          "^THLD 2\r"
          "^BLFB 1 1\r"
          "^BLSTD 1 0\r"
          "^BLL 1 -65535\r"
          "^BHL 1 65535\r"
          "^MXRPM 1 6500\r"
          "^MAC 1 60000\r"
          "^MDEC 1 100000\r"
          "^MMOD 1 1\r"
          "^MVEL 1 6500\r"
          "^MXTRN 1 1092250\r"
          "^KP 1 1\r"
          "^KI 1 3\r"
          "^KD 1 5\r"
          "^CLERD 1 0\r"
          "^BLFB 2 1\r"
          "^BLSTD 2 0\r"
          "^BLL 2 -65535\r"
          "^BHL 2 65535\r"
          "^MXRPM 2 6500\r"
          "^MAC 2 60000\r"
          "^MDEC 2 100000\r"
          "^MMOD 2 1\r"
          "^MVEL 2 6500\r"
          "^MXTRN 2 1092250\r"
          "^KP 2 1\r"
          "^KI 2 2\r"
          "^KD 2 4\r"
          "^CLERD 2 0\r"
          "%eesav\r"
          "# C\r"
          "#\r"},
      {"Reg_Right", 
          "# C\r"
          "^ALIM 1 220\r"
          "^ALIM 2 220\r"
          "^DINA 6 0\r"
          "^THLD 2\r"
          "^BLFB 1 1\r"
          "^BLSTD 1 0\r"
          "^BLL 1 -65535\r"
          "^BHL 1 65535\r"
          "^MXRPM 1 6500\r"
          "^MAC 1 60000\r"
          "^MDEC 1 100000\r"
          "^MMOD 1 1\r"
          "^MVEL 1 6500\r"
          "^MXTRN 1 1092250\r"
          "^KP 1 1\r"
          "^KI 1 3\r"
          "^KD 1 5\r"
          "^CLERD 1 0\r"
          "^BLFB 2 1\r"
          "^BLSTD 2 0\r"
          "^BLL 2 -65535\r"
          "^BHL 2 65535\r"
          "^MXRPM 2 6500\r"
          "^MAC 2 60000\r"
          "^MDEC 2 100000\r"
          "^MMOD 2 1\r"
          "^MVEL 2 6500\r"
          "^MXTRN 2 1092250\r"
          "^KP 2 1\r"
          "^KI 2 2\r"
          "^KD 2 4\r"
          "^CLERD 2 0\r"
          "%eesav\r"
          "# C\r"
          "#\r"},
      {"Comp_Left", 
          "# C\r"
          "^ALIM 1 220\r"
          "^ALIM 2 220\r"
          "^DINA 6 0\r"
          "^THLD 2\r"
          "^BLFB 1 1\r"
          "^BLSTD 1 3\r"
          "^BLL 1 -65535\r"
          "^BHL 1 65535\r"
          "^MXRPM 1 6500\r"
          "^MAC 1 60000\r"
          "^MDEC 1 100000\r"
          "^MMOD 1 1\r"
          "^MVEL 1 6500\r"
          "^MXTRN 1 1092250\r"
          "^KP 1 1\r"
          "^KI 1 3\r"
          "^KD 1 1\r"
          "^CLERD 1 0\r"
          "^BLFB 2 1\r"
          "^BLSTD 2 3\r"
          "^BLL 2 -65535\r"
          "^BHL 2 65535\r"
          "^MXRPM 2 6500\r"
          "^MAC 2 60000\r"
          "^MDEC 2 100000\r"
          "^MMOD 2 1\r"
          "^MVEL 2 6500\r"
          "^MXTRN 2 1092250\r"
          "^KP 2 1\r"
          "^KI 2 2\r"
          "^KD 2 2\r"
          "^CLERD 2 0\r"
          "%eesav\r"
          "# C\r"
          "#\r"},
      {"Comp_Right", 
          "# C\r"
          "^ALIM 1 220\r"
          "^ALIM 2 220\r"
          "^DINA 6 0\r"
          "^THLD 2\r"
          "^BLFB 1 1\r"
          "^BLSTD 1 3\r"
          "^BLL 1 -65535\r"
          "^BHL 1 65535\r"
          "^MXRPM 1 6500\r"
          "^MAC 1 60000\r"
          "^MDEC 1 100000\r"
          "^MMOD 1 1\r"
          "^MVEL 1 6500\r"
          "^MXTRN 1 1092250\r"
          "^KP 1 1\r"
          "^KI 1 3\r"
          "^KD 1 1\r"
          "^CLERD 1 0\r"
          "^BLFB 2 1\r"
          "^BLSTD 2 3\r"
          "^BLL 2 -65535\r"
          "^BHL 2 65535\r"
          "^MXRPM 2 6500\r"
          "^MAC 2 60000\r"
          "^MDEC 2 100000\r"
          "^MMOD 2 1\r"
          "^MVEL 2 6500\r"
          "^MXTRN 2 1092250\r"
          "^KP 2 1\r"
          "^KI 2 2\r"
          "^KD 2 2\r"
          "^CLERD 2 0\r"
          "%eesav\r"
          "# C\r"
          "#\r"}
    };

  };
}

PLUGINLIB_EXPORT_CLASS(hw_interface_plugin_roboteq::brushless, base_classes::base_interface)

#endif //HW_INTERFACE_PLUGIN_ROBOTEQ_BRUSHLESS_HPP__
