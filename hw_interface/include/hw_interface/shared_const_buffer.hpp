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

#ifndef SHARED_CONST_BUFFER_HPP__
#define SHARED_CONST_BUFFER_HPP__

//
// reference_counted.hpp
// ~~~~~~~~~~~~~~~~~~~~~
//
// Copyright (c) 2003-2015 Christopher M. Kohlhoff (chris at kohlhoff dot com)
//
// Distributed under the Boost Software License, Version 1.0. (See accompanying
// file LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)
//

#include <boost/asio.hpp>
#include <vector>

// A reference-counted non-modifiable buffer class.

namespace hw_interface_support_types
{
    class shared_const_buffer
    {
        public:
            // Construct from a std::string.
            shared_const_buffer(const std::string& data)
                : data_(new std::vector<char>(data.begin(), data.end())),
                  buffer_(new boost::asio::const_buffer(boost::asio::buffer(*data_)))
            {
            }

            shared_const_buffer(const char* data, const int length)
                : data_(new std::vector<char>(data, data+length)),
                  buffer_(new boost::asio::const_buffer(boost::asio::buffer(*data_)))
            {
            }

            shared_const_buffer(const char* data, const int length, bool LEtoBE)
                : data_(new std::vector<char>(length)),
                  buffer_(new boost::asio::const_buffer(boost::asio::buffer(*data_)))
            {
                if(LEtoBE)
                {
                    for(int i = 0; i < length; i++)
                    {
                        data_->data()[i] = data[length-1-i];
                    }
                }
            }

            shared_const_buffer(const std::vector<char> &data)
                : data_(new std::vector<char>(data)),
                  buffer_(new boost::asio::const_buffer(boost::asio::buffer(*data_)))
            {

            }

            template<typename T>
            shared_const_buffer(const std::vector<T> &data)
                : data_(new std::vector<char>(data)),
                  buffer_(new boost::asio::const_buffer(boost::asio::buffer(*data_)))
            {

            }

            template<typename S>
            shared_const_buffer(const S &data)
                :   data_(new std::vector<char>(reinterpret_cast<char*>(&data), sizeof(S)+reinterpret_cast<char*>(&data))),
                    buffer_(boost::asio::buffer(*data_))
            {

            }


            // Implement the ConstBufferSequence requirements.
            typedef boost::asio::const_buffer value_type;
            typedef const boost::asio::const_buffer* const_iterator;
            const boost::asio::const_buffer* begin() const { return buffer_.get(); }
            const boost::asio::const_buffer* end() const { return buffer_.get() + 1; }

        protected:
            //protected empty constructor for implementation overlods if needed
            shared_const_buffer() {}
            std::shared_ptr<std::vector<char> > data_;
            std::shared_ptr<boost::asio::const_buffer> buffer_;

    };
}

#endif //SHARED_CONST_BUFFER_HPP__

