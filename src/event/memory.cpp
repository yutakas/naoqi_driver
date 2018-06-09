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

#include <iostream>
#include <vector>

#include <boost/make_shared.hpp>

#include <ros/ros.h>

#include <qi/anyobject.hpp>

#include <naoqi_driver/recorder/globalrecorder.hpp>
#include <naoqi_driver/message_actions.h>

#include "memory.hpp"

namespace naoqi
{

template<class T>
MemoryEventRegister<T>::MemoryEventRegister()
{
}

template<class T>
MemoryEventRegister<T>::MemoryEventRegister( const std::string& name, const std::string& key, const float& frequency, const qi::SessionPtr& session )
  : serviceId(0),
    p_memory_( session->service("ALMemory")),
    session_(session),
    isStarted_(false),
    isPublishing_(false),
    isRecording_(false),
    isDumping_(false)
{
  publisher_ = boost::make_shared<publisher::BasicPublisher<T> >( name );
  //recorder_ = boost::make_shared<recorder::BasicEventRecorder<T> >( name );

  key_ = key;
  name_ = name;
}

template<class T>
MemoryEventRegister<T>::~MemoryEventRegister()
{
  stopProcess();
}

template<class T>
void MemoryEventRegister<T>::resetPublisher(ros::NodeHandle& nh)
{
  publisher_->reset(nh);
}

template<class T>
void MemoryEventRegister<T>::resetRecorder( boost::shared_ptr<naoqi::recorder::GlobalRecorder> gr )
{
  //recorder_->reset(gr, converter_->frequency());
}

template<class T>
void MemoryEventRegister<T>::startProcess()
{
  std::cout << __FILE__ << " " << __func__ << " -------- 0 " << std::endl;
    
  boost::mutex::scoped_lock start_lock(mutex_);
  if (!isStarted_)
  {
    if(!serviceId)
    {
      //std::string serviceName = std::string("ROS-Driver-") + typeid(T).name();
      std::string serviceName = std::string("ROS-Driver-") + key_;
  std::cout << __FILE__ << " " << __func__ << " -------- 1 " << std::endl;
      serviceId = session_->registerService(serviceName, this->shared_from_this());
  std::cout << __FILE__ << " " << __func__ << " -------- 2 " << std::endl;
        p_memory_.call<void>("subscribeToEvent",key_, serviceName, "memoryCallback");
  std::cout << __FILE__ << " " << __func__ << " -------- 3 " << std::endl;
      std::cout << serviceName << " : Start" << std::endl;
    }
    isStarted_ = true;
  }
}

template<class T>
void MemoryEventRegister<T>::stopProcess()
{
  boost::mutex::scoped_lock stop_lock(mutex_);
  if (isStarted_)
  {
    //std::string serviceName = std::string("ROS-Driver-") + typeid(T).name();
    std::string serviceName = std::string("ROS-Driver-") + key_;
    if(serviceId){
      p_memory_.call<void>("unsubscribeToEvent",key_, serviceName);
      session_->unregisterService(serviceId);
      serviceId = 0;
    }
    std::cout << serviceName << " : Stop" << std::endl;
    isStarted_ = false;
  }
}

template<class T>
void MemoryEventRegister<T>::writeDump(const ros::Time& time)
{
  if (isStarted_)
  {
    //recorder_->writeDump(time);
  }
}

template<class T>
void MemoryEventRegister<T>::setBufferDuration(float duration)
{
  //recorder_->setBufferDuration(duration);
}

template<class T>
void MemoryEventRegister<T>::isRecording(bool state)
{
  boost::mutex::scoped_lock rec_lock(mutex_);
  isRecording_ = state;
}

template<class T>
void MemoryEventRegister<T>::isPublishing(bool state)
{
  boost::mutex::scoped_lock pub_lock(mutex_);
  isPublishing_ = state;
}

template<class T>
void MemoryEventRegister<T>::isDumping(bool state)
{
  boost::mutex::scoped_lock dump_lock(mutex_);
  isDumping_ = state;
}

template<class T>
void MemoryEventRegister<T>::registerCallback()
{
}

template<class T>
void MemoryEventRegister<T>::unregisterCallback()
{
}

template<class T>
void MemoryEventRegister<T>::memoryCallback(std::string &key, qi::AnyValue &value, qi::AnyValue &message)
{

  std::vector<message_actions::MessageAction> actions;
  boost::mutex::scoped_lock callback_lock(mutex_);
  if (isStarted_) {
    // CHECK FOR PUBLISH
    if ( isPublishing_ && publisher_->isSubscribed() )
    {
      actions.push_back(message_actions::PUBLISH);
    }
    // CHECK FOR RECORD
    if ( isRecording_ )
    {
      //actions.push_back(message_actions::RECORD);
    }
    if ( !isDumping_ )
    {
      //actions.push_back(message_actions::LOG);
    }
    if (actions.size() >0)
    {
      callConverter(actions);
    }
  }
}


// http://stackoverflow.com/questions/8752837/undefined-reference-to-template-class-constructor
template class MemoryEventRegister<naoqi_bridge_msgs::BoolStamped>;
template class MemoryEventRegister<naoqi_bridge_msgs::IntStamped>;
template class MemoryEventRegister<naoqi_bridge_msgs::FloatStamped>;
template class MemoryEventRegister<naoqi_bridge_msgs::StringStamped>;

}//namespace
