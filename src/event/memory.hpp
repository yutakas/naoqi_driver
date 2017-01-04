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

#ifndef MEMORY_EVENT_REGISTER_HPP
#define MEMORY_EVENT_REGISTER_HPP

#include <string>

#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/enable_shared_from_this.hpp>

#include <qi/session.hpp>

#include <ros/ros.h>

#include <naoqi_driver/tools.hpp>
#include <naoqi_driver/recorder/globalrecorder.hpp>

// Converter
#include "../src/converters/memory/bool.hpp"
#include "../src/converters/memory/int.hpp"
#include "../src/converters/memory/float.hpp"
#include "../src/converters/memory/string.hpp"
// Publisher
#include "../src/publishers/basic.hpp"
// Recorder
#include "../recorder/basic_event.hpp"

namespace naoqi
{

/**
* @brief GlobalRecorder concept interface
* @note this defines an private concept struct,
* which each instance has to implement
* @note a type erasure pattern in implemented here to avoid strict inheritance,
* thus each possible publisher instance has to implement the virtual functions mentioned in the concept
*/
template<class T>
class MemoryEventRegister: public boost::enable_shared_from_this<MemoryEventRegister<T> >
{

public:

  /**
  * @brief Constructor for recorder interface
  */
  MemoryEventRegister();
  MemoryEventRegister(const std::string& name, const std::string& key, const float& frequency, const qi::SessionPtr& session );
  ~MemoryEventRegister();

  void resetPublisher( ros::NodeHandle& nh );
  void resetRecorder( boost::shared_ptr<naoqi::recorder::GlobalRecorder> gr );

  void startProcess();
  void stopProcess();

  void writeDump(const ros::Time& time);
  void setBufferDuration(float duration);

  void isRecording(bool state);
  void isPublishing(bool state);
  void isDumping(bool state);

  void memoryCallback(std::string &key, qi::AnyValue &value, qi::AnyValue &message);

  virtual void callConverter(std::vector<message_actions::MessageAction> actions) {}

private:
  void registerCallback();
  void unregisterCallback();
  void onEvent();

protected:
  boost::shared_ptr<publisher::BasicPublisher<T> > publisher_;
  //boost::shared_ptr<recorder::BasicEventRecorder<T> > recorder_;

  qi::SessionPtr session_;
  qi::AnyObject p_memory_;
  unsigned int serviceId;
  std::string name_;

  boost::mutex mutex_;

  bool isStarted_;
  bool isPublishing_;
  bool isRecording_;
  bool isDumping_;

protected:
  std::string key_;
}; // class

class MemoryBoolEventRegister: public MemoryEventRegister<naoqi_bridge_msgs::BoolStamped>
{
public:
  MemoryBoolEventRegister( const std::string& name, const std::string& key, const float& frequency, const qi::SessionPtr& session ) : MemoryEventRegister<naoqi_bridge_msgs::BoolStamped>(name, key, frequency, session) 
  {
    converter_ = boost::make_shared<converter::MemoryBoolConverter>( name, frequency, session , key);
    converter_->registerCallback( message_actions::PUBLISH, boost::bind(&publisher::BasicPublisher<naoqi_bridge_msgs::BoolStamped>::publish, publisher_, _1) );
  }
  void callConverter(std::vector<message_actions::MessageAction> actions)
  {
      converter_->callAll( actions );
  }
private:
  boost::shared_ptr<converter::MemoryBoolConverter> converter_;
};



class MemoryIntEventRegister: public MemoryEventRegister<naoqi_bridge_msgs::IntStamped>
{
public:
  MemoryIntEventRegister( const std::string& name, const std::string& key, const float& frequency, const qi::SessionPtr& session ) : MemoryEventRegister<naoqi_bridge_msgs::IntStamped>(name, key, frequency, session)
  {
    converter_ = boost::make_shared<converter::MemoryIntConverter>( name, frequency, session , key);
    converter_->registerCallback( message_actions::PUBLISH, boost::bind(&publisher::BasicPublisher<naoqi_bridge_msgs::IntStamped>::publish, publisher_, _1) );
  }
  void callConverter(std::vector<message_actions::MessageAction> actions)
  {
      converter_->callAll( actions );
  }
private:
  boost::shared_ptr<converter::MemoryIntConverter> converter_;
};

class MemoryFloatEventRegister: public MemoryEventRegister<naoqi_bridge_msgs::FloatStamped>
{
public:
  MemoryFloatEventRegister( const std::string& name, const std::string& key, const float& frequency, const qi::SessionPtr& session ) : MemoryEventRegister<naoqi_bridge_msgs::FloatStamped>(name, key, frequency, session)
  {
    converter_ = boost::make_shared<converter::MemoryFloatConverter>( name, frequency, session , key);
    converter_->registerCallback( message_actions::PUBLISH, boost::bind(&publisher::BasicPublisher<naoqi_bridge_msgs::FloatStamped>::publish, publisher_, _1) );
  }
  void callConverter(std::vector<message_actions::MessageAction> actions)
  {
      converter_->callAll( actions );
  }
private:
  boost::shared_ptr<converter::MemoryFloatConverter> converter_;
};

class MemoryStringEventRegister: public MemoryEventRegister<naoqi_bridge_msgs::StringStamped>
{
public:
  MemoryStringEventRegister( const std::string& name, const std::string& key, const float& frequency, const qi::SessionPtr& session ) : MemoryEventRegister<naoqi_bridge_msgs::StringStamped>(name, key, frequency, session)
  {
    converter_ = boost::make_shared<converter::MemoryStringConverter>( name, frequency, session , key);
    converter_->registerCallback( message_actions::PUBLISH, boost::bind(&publisher::BasicPublisher<naoqi_bridge_msgs::StringStamped>::publish, publisher_, _1) );
  }
  void callConverter(std::vector<message_actions::MessageAction> actions)
  {
      converter_->callAll( actions );
  }
private:
  boost::shared_ptr<converter::MemoryStringConverter> converter_;
};

static bool _qiregisterMemoryEventRegisterBool() {
  ::qi::ObjectTypeBuilder<MemoryEventRegister<naoqi_bridge_msgs::BoolStamped> > b;
  QI_VAARGS_APPLY(__QI_REGISTER_ELEMENT, MemoryEventRegister<naoqi_bridge_msgs::BoolStamped>, memoryCallback)
    b.registerType();
  return true;
  }
static bool BOOST_PP_CAT(__qi_registration, __LINE__) = _qiregisterMemoryEventRegisterBool();


static bool _qiregisterMemoryEventRegisterInt() {
  ::qi::ObjectTypeBuilder<MemoryEventRegister<naoqi_bridge_msgs::IntStamped> > b;
  QI_VAARGS_APPLY(__QI_REGISTER_ELEMENT, MemoryEventRegister<naoqi_bridge_msgs::IntStamped>, memoryCallback)
    b.registerType();
  return true;
  }
static bool BOOST_PP_CAT(__qi_registration, __LINE__) = _qiregisterMemoryEventRegisterInt();

static bool _qiregisterMemoryEventRegisterFloat() {
  ::qi::ObjectTypeBuilder<MemoryEventRegister<naoqi_bridge_msgs::FloatStamped> > b;
  QI_VAARGS_APPLY(__QI_REGISTER_ELEMENT, MemoryEventRegister<naoqi_bridge_msgs::FloatStamped>, memoryCallback)
    b.registerType();
  return true;
  }
static bool BOOST_PP_CAT(__qi_registration, __LINE__) = _qiregisterMemoryEventRegisterFloat();

static bool _qiregisterMemoryEventRegisterString() {
  ::qi::ObjectTypeBuilder<MemoryEventRegister<naoqi_bridge_msgs::StringStamped> > b;
  QI_VAARGS_APPLY(__QI_REGISTER_ELEMENT, MemoryEventRegister<naoqi_bridge_msgs::StringStamped>, memoryCallback)
    b.registerType();
  return true;
  }
static bool BOOST_PP_CAT(__qi_registration, __LINE__) = _qiregisterMemoryEventRegisterString();

} //naoqi

#endif
