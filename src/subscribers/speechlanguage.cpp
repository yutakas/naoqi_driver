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
#include "speechlanguage.hpp"


namespace naoqi
{
namespace subscriber
{

SpeechLanguageSubscriber::SpeechLanguageSubscriber( const std::string& name, const std::string& speechlanguage_topic, const qi::SessionPtr& session ):
  speechlanguage_topic_(speechlanguage_topic),
  BaseSubscriber( name, speechlanguage_topic, session ),
  p_tts_( session->service("ALTextToSpeech") )
{}

void SpeechLanguageSubscriber::reset( ros::NodeHandle& nh )
{
  sub_speechlanguage_ = nh.subscribe( speechlanguage_topic_, 10, &SpeechLanguageSubscriber::speechlanguage_callback, this );

  is_initialized_ = true;
}

void SpeechLanguageSubscriber::speechlanguage_callback( const std_msgs::StringConstPtr& string_msg )
{
  p_tts_.async<void>("setLanguage", string_msg->data);
}

} //publisher
} // naoqi
