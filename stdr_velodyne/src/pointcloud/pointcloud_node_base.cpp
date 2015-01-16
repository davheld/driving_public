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

#include <stdr_lib/rosparam_helpers.h>

#include "pointcloud_node_base.h"
#include <stdr_velodyne/point_type.h>

namespace stdr_velodyne
{


PacketToPcdNodeBase::PacketToPcdNodeBase(ros::NodeHandle node_handle, ros::NodeHandle private_nh)
  : PacketToPcd(node_handle)
{
  Configuration::Ptr config = Configuration::getStaticConfigurationInstance();

  std::string  cal_filename, int_filename;
  GET_ROS_PARAM_ABORT(node_handle, "cal_file", cal_filename);
  config->readCalibration(cal_filename);

  GET_ROS_PARAM_WARN(node_handle, "calibrate_intensities", calibrate_intensities_, true);

  if( calibrate_intensities_ ) {
    if( ! node_handle.getParam("int_file", int_filename) ) {
      ROS_FATAL("Could not get calibration file from rosparam");
      ROS_BREAK();
    }
    config->readIntensity(int_filename);
  }

  pub_ = node_handle.advertise<PointCloud>("points", 100);
  sub_ = node_handle.subscribe("packets", 100, &PacketToPcdNodeBase::cb, this);
}


void PacketToPcdNodeBase::cb(const velodyne_msgs::VelodyneScan::ConstPtr & vscan)
{
  //ROS_DEBUG("Received a VelodyneScan with time stamp %f. Delay is %fms",
  //          vscan->header.stamp.toSec(), (ros::Time::now()-vscan->header.stamp).toSec()*1000);

  PointCloud::Ptr scan_pcd(new PointCloud);
  scan_pcd->header = pcl_conversions::toPCL(vscan->header);
  scan_pcd->points.reserve(vscan->packets.size() * velodyne_rawdata::SCANS_PER_PACKET);

  BOOST_FOREACH(const velodyne_msgs::VelodynePacket & packet, vscan->packets) {
    processPacket(packet, *scan_pcd);
  }

  //ROS_DEBUG("Publishing point cloud with time stamp %f. Delay is %fms",
  //          pcl_conversions::fromPCL(scan_pcd->header).stamp.toSec(),
  //          (ros::Time::now() - pcl_conversions::fromPCL(scan_pcd->header).stamp).toSec()*1000);

  pub_.publish(scan_pcd);
}

} //namespace stdr_velodyne
