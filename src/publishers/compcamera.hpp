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

#ifndef PUBLISHER_COMPCAMERA_HPP
#define PUBLISHER_COMPCAMERA_HPP

/*
* ROS includes
*/
#include <ros/ros.h>
#include <std_msgs/UInt8MultiArray.h>

namespace naoqi
{
namespace publisher
{

class CompressedCameraPublisher
{
public:
  CompressedCameraPublisher( const std::string& topic, int camera_source );

  ~CompressedCameraPublisher();

  inline std::string topic() const
  {
    return topic_;
  }

  inline bool isInitialized() const
  {
    return is_initialized_;
  }

  void publish( std_msgs::UInt8MultiArray& img);

  void reset( ros::NodeHandle& nh );

  inline bool isSubscribed() const
  {
    if (is_initialized_ == false) 
        return false;
    return pub_compcamera_.getNumSubscribers() > 0;
  }

private:
  std::string topic_;

  bool is_initialized_;

  /** initialize separate publishers for js and odom */
  ros::Publisher pub_compcamera_;
  
  int camera_source_;
};

} //publisher
} //naoqi


#endif
