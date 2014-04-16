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

#include "conversion_node_base.h"

namespace stdr_velodyne
{

ConversionNodeBase::ConversionNodeBase(ros::NodeHandle nh, ros::NodeHandle priv_nh)
  : nh_(nh), priv_nh_(priv_nh)
{
  pub_ = nh_.advertise<velodyne_msgs::VelodyneScan>("/driving/velodyne/packets", 100);
  sub_ = nh_.subscribe("/driving/velodyne/raw_scans", 100, &ConversionNodeBase::cb, this);
}

void ConversionNodeBase::cb(const stdr_msgs::RawScans::ConstPtr & raw_scans)
{
  if (pub_.getNumSubscribers() == 0)         // no one listening?
    return;                                     // avoid much work

  //ROS_DEBUG("Received a raw scan with time stamp %f. Delay is %fms",
  //          raw_scans->header.stamp.toSec(), (ros::Time::now()-raw_scans->header.stamp).toSec()*1000);

  velodyne_msgs::VelodyneScan::Ptr vscans( new velodyne_msgs::VelodyneScan );
  convert(*raw_scans, *vscans);

  //ROS_DEBUG("Publishing the corresponding velodyne packet. Delay is %fms",
  //          (ros::Time::now()-vscans->header.stamp).toSec()*1000);

  pub_.publish(vscans);
}

} //namespace stdr_velodyne
