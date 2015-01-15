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


#include <boost/foreach.hpp>

#include <log_and_playback/bag_tf_listener.h>

namespace log_and_playback
{

void BagTFListener::addApplanixPose(const stdr_msgs::ApplanixPose & pose)
{
  app_trans_.update(pose);
  app_trans_.addToTransformer(*this, "bag");
  if( broadcast_ )
    app_trans_.broadcast(broadcaster_);

  if( !from_localize_pose_ ) {
    fake_localizer_.update(pose);
    fake_localizer_.addToTransformer(*this, "bag");
    if( broadcast_ )
      fake_localizer_.broadcast(broadcaster_);
  }

  handleStaticTransforms(pose.header.stamp);
}

void BagTFListener::addLocalizePose(const stdr_msgs::LocalizePose & pose)
{
  if( !from_localize_pose_ ) {
    // reset the localizer
    fake_localizer_ = localize::FakeLocalizer();
  }
  from_localize_pose_ = true;
  fake_localizer_.update_transforms(pose);
  fake_localizer_.addToTransformer(*this, "bag");
  if( broadcast_ )
    fake_localizer_.broadcast(broadcaster_);
  handleStaticTransforms(pose.header.stamp);
}

void BagTFListener::addTFMsg(const tf::tfMessage & msg)
{
  static tf::StampedTransform trans;
  for( unsigned i = 0; i < msg.transforms.size(); i++ ) {
    tf::transformStampedMsgToTF(msg.transforms[i], trans);
    setTransform(trans, "bag");
    if( broadcast_ )
      broadcaster_.sendTransform(trans);
  }

  if( ! msg.transforms.empty() )
    handleStaticTransforms(msg.transforms.rbegin()->header.stamp);
}

void BagTFListener::handleStaticTransforms(const ros::Time & stamp)
{
  BOOST_FOREACH(tf::StampedTransform t, static_transforms_) {
    t.stamp_ = stamp;
    setTransform(t, "bag");
    if( broadcast_ ) broadcaster_.sendTransform(t);
  }
  initialized_ = true;
}

void BagTFListener::addStaticTransform(const tf::StampedTransform & t)
{
  static_transforms_.push_back(t);
}

void BagTFListener::addStaticTransforms(const std::vector< tf::StampedTransform > & transforms)
{
  static_transforms_.insert(static_transforms_.end(), transforms.begin(), transforms.end());
}

}
