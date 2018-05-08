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


#ifndef PLAYAUDIO_SUBSCRIBER_HPP
#define PLAYAUDIO_SUBSCRIBER_HPP

/*
 * LOCAL includes
 */
#include "subscriber_base.hpp"

/*
 * ROS includes
 */
#include <ros/ros.h>
#include <std_msgs/UInt8MultiArray.h>

namespace naoqi
{
namespace subscriber
{

class PlayAudioSubscriber: public BaseSubscriber<PlayAudioSubscriber>
{
public:
  PlayAudioSubscriber( const std::string& name, const std::string& speech_topic, const qi::SessionPtr& session );
  ~PlayAudioSubscriber(){}

  void reset( ros::NodeHandle& nh );
  void playaudio_callback( const std_msgs::UInt8MultiArrayConstPtr& speech_msg );

private:

  std::string playaudio_topic_;

  qi::AnyObject p_tts_;
  ros::Subscriber sub_playaudio_;



}; // class PlayAudio

} // subscriber
}// naoqi
#endif
