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

/*
* LOCAL includes
*/
#include "compcamera.hpp"

/*
* ALDEBARAN includes
*/
#include "../tools/alvisiondefinitions.h" // for kTop...

namespace naoqi
{
namespace publisher
{

CompressedCameraPublisher::CompressedCameraPublisher( const std::string& topic, int camera_source ):
  topic_( topic ),
  is_initialized_(false),
  camera_source_( camera_source )
{
}

CompressedCameraPublisher::~CompressedCameraPublisher()
{
}

void CompressedCameraPublisher::publish(std_msgs::UInt8MultiArray& img)
{
    pub_compcamera_.publish(img);
}

void CompressedCameraPublisher::reset( ros::NodeHandle& nh )
{
  pub_compcamera_ = nh.advertise<std_msgs::UInt8MultiArray>( topic_, 10 ); 
  is_initialized_ = true;
}

} // publisher
} //naoqi
