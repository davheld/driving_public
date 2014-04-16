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

#include <localize/localizer_base.h>

namespace localize {

// no rotation quaternion
const tf::Quaternion Q0(0, 0, 0, 1); //x, y, z, w

// O=(x_offset,y_offset) is the difference between

LocalizerBase::LocalizerBase()
  : first_(true)
{
  transform_utm_to_local_utm_.frame_id_ = "utm";
  transform_utm_to_local_utm_.child_frame_id_ = "local_utm";
  transform_local_utm_to_smooth_.frame_id_ = "local_utm";
  transform_local_utm_to_smooth_.child_frame_id_ = "smooth";
}

void LocalizerBase::update_transforms(const stdr_msgs::LocalizePose & pose_msg)
{
  if( first_ ) {
    // downcast from double to float. Transforms are expressed in single precision.
    ux0_ = pose_msg.x_offset;
    uy0_ = pose_msg.y_offset;
    transform_utm_to_local_utm_.setOrigin(tf::Vector3(ux0_, uy0_, 0));
    transform_utm_to_local_utm_.setRotation(Q0);
    first_ = false;
  }

  transform_utm_to_local_utm_.stamp_ = pose_msg.header.stamp;

  const tf::Vector3 v(pose_msg.x_offset-ux0_, pose_msg.y_offset-uy0_, 0);
  transform_local_utm_to_smooth_.setOrigin(v);
  transform_local_utm_to_smooth_.setRotation(Q0);
  transform_local_utm_to_smooth_.stamp_ = pose_msg.header.stamp;
}

void LocalizerBase::broadcast(tf::TransformBroadcaster & br) const
{
  br.sendTransform(transform_utm_to_local_utm_);
  br.sendTransform(transform_local_utm_to_smooth_);
}

void LocalizerBase::addToTransformer(tf::Transformer & transformer, const std::string & authority) const
{
  transformer.setTransform(transform_utm_to_local_utm_, authority);
  transformer.setTransform(transform_local_utm_to_smooth_, authority);
}

} //namespace localize
