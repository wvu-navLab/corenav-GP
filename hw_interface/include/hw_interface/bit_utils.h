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

#ifndef BIT_UTILS_H
#define BIT_UTILS_H

class Utils_Base
{
public:
	int zero_one(int input_val) {if (input_val == 0) return 0; else return 1;}
};

class Set_Reset:public Utils_Base
{
private: 
	int val;
public: 
	Set_Reset() {val = 0;}
	int set(int input_val) {if (zero_one(input_val)==1) val = 1; return val;}
	int reset(int input_val) {if (zero_one(input_val)==1) val = 0; return val;}
	int get_val() {return val;}
};

class Leading_Edge_Latch:public Utils_Base
{
private:
	int last_val;
	int LE_val;
	int first_pass;
public:
	Leading_Edge_Latch() {last_val = 0; LE_val = 0; first_pass = 1;}
	int LE_Latch(int input_val) {if(first_pass==1){last_val = zero_one(input_val);} LE_val = (zero_one(input_val)==1/*!=last_val*/) && (last_val==0); last_val = zero_one(input_val); first_pass = 0; return LE_val;}
	int get_val() {return LE_val;}
};

class Trailing_Edge_Latch:public Utils_Base
{
private:
	int last_val;
	int TE_val;
public:
	Trailing_Edge_Latch() {last_val = 0; TE_val = 0;}
	int TE_Latch(int input_val) {TE_val = (zero_one(input_val)!=last_val) && (last_val==1); last_val = input_val; return TE_val;}
	int get_val() {return TE_val;}
};
#endif /* BIT_UTILS_H */
