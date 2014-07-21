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

#include <localize/applanix_transformer.h>


namespace localize {


ApplanixTransformer::ApplanixTransformer(std::string child_frame)
{
  transform_smooth_to_baselinkxyz_.frame_id_ = "smooth";
  transform_smooth_to_baselinkxyz_.child_frame_id_ = child_frame + "_xyz";
  transform_baselinkxyz_to_baselink_.frame_id_ = child_frame + "_xyz";
  transform_baselinkxyz_to_baselink_.child_frame_id_ = child_frame;

  odom_.header.frame_id = "smooth";
  odom_.child_frame_id = child_frame;
}


void ApplanixTransformer::update(const stdr_msgs::ApplanixPose& pose)
{
  odom_.header = pose.header;

  odom_.pose.pose.position.x = pose.smooth_x;
  odom_.pose.pose.position.y = pose.smooth_y;
  odom_.pose.pose.position.z = pose.smooth_z;
  odom_.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(pose.roll, pose.pitch, pose.yaw);

  odom_.twist.twist.linear.x = pose.speed;
  odom_.twist.twist.angular.x = pose.rate_roll;
  odom_.twist.twist.angular.y = pose.rate_pitch;
  odom_.twist.twist.angular.z = pose.rate_yaw;


  transform_smooth_to_baselinkxyz_.setOrigin( tf::Vector3(pose.smooth_x, pose.smooth_y, pose.smooth_z) );
  transform_smooth_to_baselinkxyz_.setRotation( tf::createQuaternionFromRPY(0, 0, 0) );
  transform_smooth_to_baselinkxyz_.stamp_ = pose.header.stamp;

  transform_baselinkxyz_to_baselink_.setOrigin( tf::Vector3(0, 0, 0) );
  transform_baselinkxyz_to_baselink_.setRotation( tf::createQuaternionFromRPY(pose.roll, pose.pitch, pose.yaw) );
  transform_baselinkxyz_to_baselink_.stamp_ = pose.header.stamp;
}


void ApplanixTransformer::broadcast(tf::TransformBroadcaster & br) const
{
  br.sendTransform(transform_smooth_to_baselinkxyz_);
  br.sendTransform(transform_baselinkxyz_to_baselink_);
}


void ApplanixTransformer::addToTransformer(tf::Transformer & transformer, const std::string & authority) const
{
  transformer.setTransform(transform_smooth_to_baselinkxyz_, authority);
  transformer.setTransform(transform_baselinkxyz_to_baselink_, authority);
}


} //namespace localize
