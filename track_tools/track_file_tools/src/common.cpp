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
#include <ros/assert.h>
#include "common.h"

void concatSMP2s(sensor_msgs::PointCloud2& smp,
                 const std::vector<const sensor_msgs::PointCloud2*>& smps)
{
  if( smps.empty() )
    return;

  smp.header = smps.front()->header;
  smp.height = smps.front()->height;
  smp.fields = smps.front()->fields;
  smp.is_bigendian = smps.front()->is_bigendian;
  smp.point_step = smps.front()->point_step;
  smp.row_step = smps.front()->row_step;
  smp.is_dense = smps.front()->is_dense;

  size_t size = 0;
  smp.width = 0;
  BOOST_FOREACH(const sensor_msgs::PointCloud2* cloud, smps) {
    ROS_ASSERT( smp.header.stamp == cloud->header.stamp );
    ROS_ASSERT( smp.header.frame_id == cloud->header.frame_id );
    ROS_ASSERT( smp.height == cloud->height );
    ROS_ASSERT( smp.fields.size() == cloud->fields.size() );
    for(unsigned i=0; i<smp.fields.size(); ++i ) {
      ROS_ASSERT( smp.fields[i].name == cloud->fields[i].name );
      ROS_ASSERT( smp.fields[i].offset == cloud->fields[i].offset );
      ROS_ASSERT( smp.fields[i].datatype == cloud->fields[i].datatype );
      ROS_ASSERT( smp.fields[i].count == cloud->fields[i].count );
    }
    ROS_ASSERT( smp.is_bigendian == cloud->is_bigendian );
    ROS_ASSERT( smp.point_step == cloud->point_step );
    ROS_ASSERT( smp.row_step == cloud->row_step );
    ROS_ASSERT( smp.is_dense == cloud->is_dense );

    smp.width += cloud->width;
    size += cloud->data.size();
  }

  smp.data.reserve(smp.data.size() + size);
  BOOST_FOREACH(const sensor_msgs::PointCloud2* cloud, smps) {
    smp.data.insert(smp.data.end(),
                    cloud->data.begin(),
                    cloud->data.end());
  }
}
