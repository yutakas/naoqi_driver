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
#include "playaudio.hpp"


namespace naoqi
{
namespace subscriber
{

PlayAudioSubscriber::PlayAudioSubscriber( const std::string& name, const std::string& playaudio_topic, const qi::SessionPtr& session ):
  playaudio_topic_(playaudio_topic),
  BaseSubscriber( name, playaudio_topic, session ),
  p_tts_( session->service("PlayAudioService") )
{}

void PlayAudioSubscriber::reset( ros::NodeHandle& nh )
{
  sub_playaudio_ = nh.subscribe( playaudio_topic_, 10, &PlayAudioSubscriber::playaudio_callback, this );
  is_initialized_ = true;
}

void PlayAudioSubscriber::playaudio_callback( const std_msgs::UInt8MultiArrayConstPtr& msg )
{
  p_tts_.async<void>("play", msg->data);
}

} //publisher
} // naoqi
