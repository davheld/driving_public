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

#ifndef __LADYBUG_PLAYBACK__RECOMBINE_NODE_BASE_H__
#define __LADYBUG_PLAYBACK__RECOMBINE_NODE_BASE_H__

#include <boost/thread.hpp>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <dynamic_reconfigure/server.h>
#include <driver_base/driver.h>

#include <ladybug_playback/LadybugRecombineConfig.h>
#include <ladybug_playback/recombine.h>


/** Subscribes to the separated, compressed and rotated 24 images from the ladybug
  * (stdr_msgs/LadybugImages msg) and publishes the 6 colored images
  */
namespace ladybug_playback
{

class RecombineNodeBase : Recombiner
{
public:
  RecombineNodeBase(ros::NodeHandle & nh, ros::NodeHandle & priv_nh);

private:
  static const unsigned NCAMS = 6;
  ros::NodeHandle nh_, priv_nh_;
  ros::Subscriber sub_raw_;
  bool sub_raw_connected_;
  image_transport::ImageTransport it_;
  boost::shared_ptr<camera_info_manager::CameraInfoManager> cinfos_[NCAMS];
  image_transport::CameraPublisher pubs_[NCAMS];

  boost::recursive_mutex mutex_;
  volatile driver_base::Driver::state_t state_; // current driver state
  volatile bool reconfiguring_;         // true when reconfig() running
  LadybugRecombineConfig config_;
  dynamic_reconfigure::Server<LadybugRecombineConfig> srv_;

  void createPublishers(LadybugRecombineConfig &newconfig);
  void connectCb();
  void imageCb(const stdr_msgs::LadybugImages::ConstPtr & raw_msg);
  void reconfig(LadybugRecombineConfig &newconfig, uint32_t level);
};

} //namespace ladybug_playback

#endif // __LADYBUG_PLAYBACK__RECOMBINE_NODE_BASE_H__
