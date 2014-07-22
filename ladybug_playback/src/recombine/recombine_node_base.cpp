/********************************************************
  Stanford Driving Software
  Copyright (c) 2011 Stanford University
  All rights reserved.

  Redistribution and use in source and binary forms, with
  or without modification, are permitted provided that the
  following conditions are met:

* Redistributions of source code must retain the above
  copyright notice, this list of conditions and the
  following disclaimer.
* Redistributions in binary form must reproduce the above
  copyright notice, this list of conditions and the
  following disclaimer in the documentation and/or other
  materials provided with the distribution.
* The names of the contributors may not be used to endorse
  or promote products derived from this software
  without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
  CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
  WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
  PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
  INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
  PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
  OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
  DAMAGE.
 ********************************************************/

#include <boost/lexical_cast.hpp>

#include <driver_base/SensorLevels.h>

#include "recombine_node_base.h"

namespace ladybug_playback
{

RecombineNodeBase::RecombineNodeBase(ros::NodeHandle & nh, ros::NodeHandle & priv_nh)
  : nh_(nh), priv_nh_(priv_nh), sub_raw_connected_(false), it_(nh)
  , state_(driver_base::Driver::CLOSED), reconfiguring_(false), srv_(priv_nh)
{
  for( unsigned i=0; i<NCAMS; ++i )
  {
    const std::string istr = boost::lexical_cast<std::string>(i);
    const std::string ns = std::string("/driving/ladybug/camera") + istr;
    std::string camera_info_url;
    nh.getParam(std::string("/driving/ladybug/intrinsics")+istr, camera_info_url);
    camera_info_url = std::string("file://") + camera_info_url;

    ros::NodeHandle cimnh(ns);
    cinfos_[i].reset(new camera_info_manager::CameraInfoManager(cimnh));
    cinfos_[i]->setCameraName(std::string("ladybug") + istr);

    if( ! cinfos_[i]->loadCameraInfo(camera_info_url) ) {
      ROS_WARN_STREAM("Could not load calibration information for camera "
                      <<i <<" at URL " <<camera_info_url
                      <<". Publishing uncalibrated images.");
      cinfos_[i].reset();
    }
  }

  srv_.setCallback(boost::bind(&RecombineNodeBase::reconfig, this, _1, _2));
}

void RecombineNodeBase::createPublishers(LadybugRecombineConfig &newconfig)
{
  // Monitor whether anyone is subscribed to the output
  typedef image_transport::SubscriberStatusCallback ConnectCB;
  ConnectCB connect_cb = boost::bind(&RecombineNodeBase::connectCb, this);

  // Make sure we don't enter connectCb() between advertising and assigning to pub_XXX
  boost::lock_guard<boost::recursive_mutex> lock(mutex_);

  for( unsigned i=0; i<NCAMS; ++i )
  {
    const std::string istr = boost::lexical_cast<std::string>(i);
    const std::string ns = std::string("/driving/ladybug/camera") + istr;
    const std::string topic = ns + std::string(newconfig.debayer ? "/image_color" : "/image_raw");
    pubs_[i] = it_.advertiseCamera(topic, 1, connect_cb, connect_cb);
  }

  state_ = driver_base::Driver::RUNNING;
}

// Handles (un)subscribing when clients (un)subscribe
void RecombineNodeBase::connectCb()
{
  boost::lock_guard<boost::recursive_mutex> lock(mutex_);
  unsigned nSubscribers = 0;
  for( unsigned i=0; i<NCAMS; ++i ) {
    const unsigned n = pubs_[i].getNumSubscribers();
    selector_[i] = (n!=0);
    nSubscribers += n;
  }
  if( nSubscribers == 0 ) {
    sub_raw_.shutdown();
    sub_raw_connected_ = false;
  }
  else if( ! sub_raw_connected_ ) {
    sub_raw_ = nh_.subscribe("/driving/ladybug/images", 1, &RecombineNodeBase::imageCb, this);
    sub_raw_connected_ = true;
  }
}

void RecombineNodeBase::imageCb(const stdr_msgs::LadybugImages::ConstPtr& raw_msg)
{
  if( reconfiguring_ ) return;
  boost::lock_guard<boost::recursive_mutex> lock(mutex_);
  ROS_INFO_STREAM("Recombining frame seq=" <<raw_msg->header.seq <<" stamp=" <<raw_msg->header.stamp);
  std::vector<sensor_msgs::Image::Ptr> images;

  recombine(*raw_msg, images);

  // publish
  for( unsigned i=0; i<NCAMS; ++i )
  {
    if( selector_[i] && images[i] ) {

      // if we failed to load the configuration file, try to at least provide
      // the size info that we can get from the image.
      if( !cinfos_[i] )
      {
        const std::string istr = boost::lexical_cast<std::string>(i);
        const std::string ns = std::string("/driving/ladybug/camera") + istr;
        ros::NodeHandle cimnh(ns);
        cinfos_[i].reset(new camera_info_manager::CameraInfoManager(cimnh));
        cinfos_[i]->setCameraName(std::string("ladybug") + istr);
        sensor_msgs::CameraInfo ci;
        ci.header = images[i]->header;
        ci.width = images[i]->width;
        ci.height = images[i]->height;
        // leaving roi and binning to 0, which is considered the same as full
        // size and no subsampling
        cinfos_[i]->setCameraInfo(ci);
      }

      sensor_msgs::CameraInfo::Ptr
        ci(new sensor_msgs::CameraInfo(cinfos_[i]->getCameraInfo()));
      ci->header = images[i]->header;
      pubs_[i].publish(images[i], ci);
      ROS_INFO_STREAM("Published recombined image " <<i
                      <<", seq=" <<images[i]->header.seq
                      <<" stamp=" <<images[i]->header.stamp);
    }
  }
}

dc1394bayer_method_t getBayerMethod(const LadybugRecombineConfig &newconfig)
{
  switch( newconfig.debayer_method ) {
  case LadybugRecombine_Nearest: return DC1394_BAYER_METHOD_NEAREST;
  case LadybugRecombine_Simple: return DC1394_BAYER_METHOD_SIMPLE;
  case LadybugRecombine_Bilinear: return DC1394_BAYER_METHOD_BILINEAR;
  case LadybugRecombine_HQLinear: return DC1394_BAYER_METHOD_HQLINEAR;
  case LadybugRecombine_Downsample: return DC1394_BAYER_METHOD_DOWNSAMPLE;
  case LadybugRecombine_Edgesense: return DC1394_BAYER_METHOD_EDGESENSE;
  case LadybugRecombine_VNG: return DC1394_BAYER_METHOD_VNG;
  case LadybugRecombine_AHD: return DC1394_BAYER_METHOD_AHD;
  }
  ROS_BREAK();
  return DC1394_BAYER_METHOD_NEAREST; //keeps the compiler happy
}

void RecombineNodeBase::reconfig(LadybugRecombineConfig &newconfig, uint32_t level)
{
  reconfiguring_ = true;
  boost::recursive_mutex::scoped_lock lock(mutex_);
  ROS_DEBUG("dynamic reconfigure level 0x%x", level);

  if (state_ != driver_base::Driver::CLOSED && (level & driver_base::SensorLevels::RECONFIGURE_CLOSE)) {
    // must close the device before updating these parameters
    for( unsigned i=0; i<6; ++i )
      pubs_[i].shutdown();
    state_ = driver_base::Driver::CLOSED;
  }

  if (state_ == driver_base::Driver::CLOSED) {
    createPublishers(newconfig);
  }

  debayer_ = newconfig.debayer;
  debayer_alg_ = getBayerMethod(newconfig);

  config_ = newconfig; // save new parameters
  reconfiguring_ = false;
}

}
