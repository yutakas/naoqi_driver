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

#ifndef CONVERTER_COMPCAMERA_HPP
#define CONVERTERCONVERTER_COMPCAMERA_HPP_CAMERA_HPP

/*
* LOCAL includes
*/
#include "converter_base.hpp"
#include <naoqi_driver/message_actions.h>

/*
* ROS includes
*/
#include <std_msgs/UInt8MultiArray.h>

namespace naoqi
{
namespace converter
{

class CompressedCameraConverter : public BaseConverter<CompressedCameraConverter>
{

  typedef boost::function<void(std_msgs::UInt8MultiArray&)> Callback_t;

public:
  CompressedCameraConverter( const std::string& name, const float& frequency, const qi::SessionPtr& session);

  ~CompressedCameraConverter();

  void reset();

  void registerCallback( const message_actions::MessageAction action, Callback_t cb );

  void callAll( const std::vector<message_actions::MessageAction>& actions );

private:
  std::map<message_actions::MessageAction, Callback_t> callbacks_;

  qi::AnyObject p_sessionManager_;
  qi::AnyObject p_video_;

  // string indicating image transport encoding
  // goes along with colorspace_
  std::string msg_colorspace_;
  int cv_mat_type_;
  // msg frame id
  std::string msg_frameid_;
};

} //publisher
} //naoqi


#endif
