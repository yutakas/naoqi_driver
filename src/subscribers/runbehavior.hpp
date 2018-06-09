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


#ifndef RUNBEHAVIOR_SUBSCRIBER_HPP
#define RUNBEHAVIOR_SUBSCRIBER_HPP

/*
 * LOCAL includes
 */
#include "subscriber_base.hpp"

/*
 * ROS includes
 */
#include <ros/ros.h>
#include <std_msgs/String.h>

#include <queue>
#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

namespace naoqi
{
namespace subscriber
{

class RunBehaviorSubscriber: public BaseSubscriber<RunBehaviorSubscriber>
{
public:
  RunBehaviorSubscriber( const std::string& name, const std::string& runbehavior_topic, const qi::SessionPtr& session );
  ~RunBehaviorSubscriber();
  
  void reset( ros::NodeHandle& nh );
  void runbehavior_callback( const std_msgs::StringConstPtr& speech_msg );

private:

  std::string runbehavior_topic_;

  qi::AnyObject p_bm_;
  ros::Subscriber sub_runbehavior_;

    boost::thread processCb_;
    std::queue<std::string> behaviors_;
    void processCb();
    boost::mutex mutex_;
    boost::condition cond_;
    bool running_;
}; // class RunBehaviorSubscriber

} // subscriber
}// naoqi
#endif
