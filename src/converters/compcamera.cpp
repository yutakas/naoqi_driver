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
#include "../tools/alvisiondefinitions.h" // for kTop...
#include "../tools/from_any_value.hpp"

/*
* ROS includes
*/
#include <cv_bridge/cv_bridge.h>

/*
* CV includes
*/
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

/*
* BOOST includes
*/
#include <boost/foreach.hpp>
#define for_each BOOST_FOREACH

namespace naoqi
{
namespace converter
{

// require CompressedImageService is registered 

CompressedCameraConverter::CompressedCameraConverter(const std::string& name, const float& frequency, const qi::SessionPtr& session)
  : BaseConverter( name, frequency, session )
{
    p_sessionManager_ = session->service("ALServiceManager");
    p_sessionManager_.call<qi::AnyValue>("startService", "CompressedImageService");
    ros::Duration(5.0).sleep();
    p_video_ = session->service("CompressedImageService");
}

CompressedCameraConverter::~CompressedCameraConverter()
{
}

void CompressedCameraConverter::reset()
{
}

void CompressedCameraConverter::registerCallback( const message_actions::MessageAction action, Callback_t cb )
{
  callbacks_[action] = cb;
}

void CompressedCameraConverter::callAll( const std::vector<message_actions::MessageAction>& actions )
{
    // std::cout << __FILE__ << " " << __func__ << " : " <<  std::endl;
    qi::AnyValue image_anyvalue = p_video_.call<qi::AnyValue>("getimage");
    std::string& strBuf = image_anyvalue.asString();
    uint8_t *buf = (uint8_t *)(strBuf.c_str());
    // std::cout  << "Size " << strBuf.size() << std::endl;

    std_msgs::UInt8MultiArray msg;

    msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    msg.layout.dim[0].size = strBuf.size();
    msg.layout.dim[0].stride = 1;
    msg.layout.dim[0].label = "png";
    msg.layout.data_offset = 0;
    msg.data.resize(strBuf.size());
    memcpy(&msg.data[0], buf, strBuf.size() * sizeof(uint8_t));
    
    for_each( const message_actions::MessageAction& action, actions )
    {
        callbacks_[action]( msg );
    }
}

} // publisher
} //naoqi
