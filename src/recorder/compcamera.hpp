/*
 * Copyright 2015 Aldebaran
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#ifndef COMPCAMERA_RECORDER_HPP
#define COMPCAMERA_RECORDER_HPP

/*
* BOOST includes
*/
#include <boost/circular_buffer.hpp>

/*
* LOCAL includes
*/
#include <naoqi_driver/recorder/globalrecorder.hpp>
#include "../helpers/recorder_helpers.hpp"

/*
* ROS includes
*/
#include <std_msgs/UInt8MultiArray.h>

namespace naoqi
{
namespace recorder
{

class CompressedCameraRecorder
{

public:
  CompressedCameraRecorder(const std::string& topic, float buffer_frequency );

  void write( std_msgs::UInt8MultiArray& img);

  void reset(boost::shared_ptr<naoqi::recorder::GlobalRecorder> gr, float conv_frequency );

  void bufferize( std_msgs::UInt8MultiArray& img);

  void writeDump(const ros::Time& time);

  void setBufferDuration(float duration);

  inline std::string topic() const
  {
    return topic_img_;
  }

  inline bool isInitialized() const
  {
    return is_initialized_;
  }

  inline void subscribe( bool state)
  {
    is_subscribed_ = state;
  }

  inline bool isSubscribed() const
  {
    return is_subscribed_;
  }

protected:
  bool is_initialized_;
  bool is_subscribed_;

  boost::circular_buffer< std_msgs::UInt8MultiArray > buffer_;
  size_t buffer_size_;
  float buffer_duration_;

  boost::mutex mutex_;

  boost::shared_ptr<naoqi::recorder::GlobalRecorder> gr_;
  std::string topic_img_;

  float buffer_frequency_;
  float conv_frequency_;
  int counter_;
  int max_counter_;

}; // class

} //publisher
} // naoqi

#endif
