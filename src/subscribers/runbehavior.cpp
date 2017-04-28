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
  running_(false),
  runbehavior_topic_(runbehavior_topic),
  BaseSubscriber( name, runbehavior_topic, session ),
  p_bm_( session->service("ALBehaviorManager") )
{}

RunBehaviorSubscriber::~RunBehaviorSubscriber()
{
  if (!running_)
  {
    running_ = false;
    while (!behaviors_.empty())
    {
        behaviors_.pop();
    }
    cond_.notify_all();
    processCb_.join();
  }
}


void RunBehaviorSubscriber::reset( ros::NodeHandle& nh )
{
  if (!running_)
  {
    processCb_ = boost::thread(&RunBehaviorSubscriber::processCb, this);
    running_ = false;
  }
  sub_runbehavior_ = nh.subscribe( runbehavior_topic_, 10, &RunBehaviorSubscriber::runbehavior_callback, this );

  is_initialized_ = true;
}

void RunBehaviorSubscriber::runbehavior_callback( const std_msgs::StringConstPtr& string_msg )
{
    if (running_)
    {
        boost::mutex::scoped_lock  lock(mutex_);
        // p_bm_.call<void>("runBehavior", string_msg->data);
        // p_bm_.async<void>("runBehavior", string_msg->data);
        behaviors_.push(string_msg->data);
        cond_.notify_all();
    }
}

void RunBehaviorSubscriber::processCb()
{
    while(running_)
    {
        { 
        boost::unique_lock<boost::mutex> lock(mutex_);
        while(behaviors_.empty()) { cond_.wait(lock); }
        }
        // std::cout << __FILE__ << " " << __func__ << " : " << std::endl;        
        while (!behaviors_.empty())
        {
            std::string behavior;
            {
            boost::mutex::scoped_lock lock(mutex_);
            behavior = behaviors_.front();
            behaviors_.pop();
            }
            // std::cout << __FILE__ << " " << __func__ << " : " << behavior << std::endl;    
            try
            {
                p_bm_.call<void>("runBehavior", behavior);
                ros::Duration(0.1).sleep();
            }
            catch( const std::exception& e )
            {
                std::cerr << "Exception caught in RunBehaviorSubscriber " << e.what() << std::endl;
            }
        }
    }
}
} //publisher
} // naoqi
