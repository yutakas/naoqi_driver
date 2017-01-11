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
#include "runbehavior.hpp"


namespace naoqi
{
namespace subscriber
{

RunBehaviorSubscriber::RunBehaviorSubscriber( const std::string& name, const std::string& runbehavior_topic, const qi::SessionPtr& session ):
  runbehavior_topic_(runbehavior_topic),
  BaseSubscriber( name, runbehavior_topic, session ),
  p_bm_( session->service("ALBehaviorManager") )
{}

void RunBehaviorSubscriber::reset( ros::NodeHandle& nh )
{
  sub_runbehavior_ = nh.subscribe( runbehavior_topic_, 10, &RunBehaviorSubscriber::runbehavior_callback, this );

  is_initialized_ = true;
}

void RunBehaviorSubscriber::runbehavior_callback( const std_msgs::StringConstPtr& string_msg )
{
  p_bm_.async<void>("runBehavior", string_msg->data);
}

} //publisher
} // naoqi
