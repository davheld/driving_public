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


#include <ros/ros.h>
#include <velodyne_pointcloud/rawdata.h>
#include <stdr_velodyne/transform.h>
#include <timer/timer.h>


namespace stdr_velodyne {

void transform_scan_in_place(
    const tf::TransformerHelper &tfl,
    const std::string & target_frame,
    PointCloud & spin)
{
  static const unsigned NBEAMS = 32;

  if( tf::strip_leading_slash(target_frame) == tf::strip_leading_slash(spin.header.frame_id) ) {
    ROS_DEBUG_THROTTLE(1, "Transforming to same frame. Skipping.");
    return;
  }

  //ScopedTimer timer("transform_scan");

  sensor_msgs::PointCloud pcl_in, pcl_out;
  pcl_in.header = pcl_conversions::fromPCL(spin.header);
  pcl_in.points.reserve(velodyne_rawdata::SCANS_PER_PACKET);
  pcl_out.points.reserve(velodyne_rawdata::SCANS_PER_PACKET);

  unsigned b=0;
  while( b<spin.size() )
  {
    pcl_in.points.clear();
    unsigned n = 0;
    for( ; b+n<spin.size() && spin[b+n].timestamp == spin[b].timestamp; ++n )
      pcl_in.points.push_back(spin[b+n].asPoint<geometry_msgs::Point32>());
    pcl_in.header.stamp.fromSec(spin[b].timestamp);

    tfl.transformPointCloud(target_frame, pcl_in, pcl_out);

    for( unsigned i=0; i<n; i++ ) {
      spin[b+i].x = pcl_out.points[i].x;
      spin[b+i].y = pcl_out.points[i].y;
      spin[b+i].z = pcl_out.points[i].z;
    }

    b += n;
  }

  spin.header.frame_id = target_frame;
}

PointCloud::Ptr transform_scan(
    const tf::TransformerHelper & tfl,
    const std::string & target_frame,
    const PointCloud & spin_in)
{
  PointCloud::Ptr spin_out(new PointCloud(spin_in));
  transform_scan_in_place(tfl, target_frame, *spin_out);
  return spin_out;
}

void transform_scan(
    const tf::TransformerHelper & tfl,
    const std::string & target_frame,
    const PointCloud & spin_in,
    PointCloud &spin_out)
{
  spin_out = spin_in; //copy
  transform_scan_in_place(tfl, target_frame, spin_out);
}

}
