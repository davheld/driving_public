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

#ifndef __APPLANIX__TRANSFORMER_H__
#define __APPLANIX__TRANSFORMER_H__

#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

#include <nav_msgs/Odometry.h>
#include <stdr_msgs/ApplanixPose.h>


namespace localize {


/** Takes ApplanixPose messages and transforms them into tf transforms,
  * odometry messages, etc.
  */
class ApplanixTransformer
{
public:
  ApplanixTransformer(const std::string &child_frame="base_link");

  /// Adds both transforms to the transformer. Useful when working offline (playback)
  void addToTransformer(tf::Transformer & transformer, const std::string & authority = "default_authority") const;

  void broadcast(tf::TransformBroadcaster & br) const;

  void update(const stdr_msgs::ApplanixPose &);

  void setChildFrame(const std::string &child_frame);

  inline const nav_msgs::Odometry& odom() const { return odom_; }

private:
  nav_msgs::Odometry odom_;
  tf::StampedTransform transform_smooth_to_baselinkxyz_, transform_baselinkxyz_to_baselink_;
};

} //namespace localize

#endif // __APPLANIX__TRANSFORMER_H__
