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

#ifndef TELEMETRY_TIME_HPP__
#define TELEMETRY_TIME_HPP__

#include <ros/ros.h>
#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/stats.hpp>
#include <boost/accumulators/statistics/rolling_mean.hpp>

class delta_loop_time
{
    private:

        bool metricsEnabled;

        //boost::mutex metricMuxtex;
        //ros::Time does not need a thread lock because it is actually locked internally
        boost::accumulators::accumulator_set<double, boost::accumulators::stats<boost::accumulators::tag::rolling_mean> > acc;
        ros::Time lastTimeMetric;
        double deltaTime;

    public:
        delta_loop_time():
        lastTimeMetric(ros::Time::now()),
        acc(boost::accumulators::tag::rolling_window::window_size = 150)
        {
            metricsEnabled = true;
        }
        bool enableMetrics()
        {
            metricsEnabled = true;
            this->lastTimeMetric = ros::Time::now();
            return metricsEnabled;
        };

        bool disableMetrics()
        {
            metricsEnabled = false;
            return metricsEnabled;
        };

        float updateMetrics()
        {
            if(metricsEnabled)
            {
                boost::scoped_ptr<char> output(new char[255]);
                deltaTime = ros::Time::now().toSec()-this->lastTimeMetric.toSec();
                acc(deltaTime);
                this->lastTimeMetric = ros::Time::now();
                return 1/(boost::accumulators::rolling_mean(acc));
            }
            return 0;
        };

        std::string printMetrics(bool printSideEffect)
        {
            if(metricsEnabled)
            {
                boost::scoped_ptr<char> output(new char[255]);
                deltaTime = ros::Time::now().toSec()-this->lastTimeMetric.toSec();
                acc(deltaTime);
                sprintf(output.get(), "Delta Time %2.3f:: Hz %2.2f:: Avg Hz %2.2f",
                                                deltaTime, 1/deltaTime, 1/(boost::accumulators::rolling_mean(acc)));
                this->lastTimeMetric = ros::Time::now();
                if(printSideEffect)
                {
                    ROS_DEBUG("%s", output.get());
                }

                std::string outputString(1, *output);
                return outputString;
            }
            else
            {
                return "";
            }
        };
};

#endif //TELEMETRY_TIME_HPP__

