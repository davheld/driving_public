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

#include <limits>
#include <velodyne_pointcloud/rawdata.h>
#include <timer/timer.h>
#include <stdr_lib/rosparam_helpers.h>
#include <stdr_velodyne/pointcloud.h>


namespace stdr_velodyne {


PacketToPcd::PacketToPcd()
  : calibrate_intensities_(true)
  , config_( stdr_velodyne::Configuration::getStaticConfigurationInstance() )
{
  GET_ROS_PARAM_INFO(ros::NodeHandle("/driving/velodyne"), "max_dist", max_dist_, std::numeric_limits<double>::max());
}

void PacketToPcd::processPacket(const velodyne_msgs::VelodynePacket& packet, PointCloud& pcd) const
{
  ROS_ASSERT(config_);
  ROS_ASSERT(config_->valid());
  //ScopedTimer timer("PacketToPcd::processPacket");

  pcd.reserve(pcd.size() + velodyne_rawdata::BLOCKS_PER_PACKET*velodyne_rawdata::SCANS_PER_BLOCK);

  const velodyne_rawdata::raw_packet_t *raw = (const velodyne_rawdata::raw_packet_t *) &(packet.data[0]);

  PointType pt;
  pt.timestamp = packet.stamp.toSec();

  BOOST_FOREACH(const velodyne_rawdata::raw_block_t & block, raw->blocks)
  {
    const unsigned e = block.rotation;
    pt.encoder = e;

    for( unsigned j=0; j<velodyne_rawdata::SCANS_PER_BLOCK; j++ ) {
      const unsigned n = config_->getBeamIndex(block.header, j);
      const RingConfig & rcfg = config_->getRingConfig(n);
      const AngleVal & hAngle = rcfg.enc_rot_angle_[pt.encoder];
      pt.h_angle = hAngle.getRads();
      pt.v_angle = rcfg.vert_angle_.getRads();
      pt.beam_id = n;
      pt.beam_nb = config_->getBeamNumber(n);

      // TODO: this could be a bit simpler if we were using a {ushort, uchar} structure
      const unsigned k = j * velodyne_rawdata::RAW_SCAN_SIZE;
      union velodyne_rawdata::two_bytes tmp;
      tmp.bytes[0] = block.data[k];
      tmp.bytes[1] = block.data[k+1];
      const uint16_t range = tmp.uint;
      const float distance = rcfg.range2dist(range);

      if( range==0 || distance>max_dist_ )
        continue;

      const uint8_t intensity = block.data[k+2];
      if( calibrate_intensities_ )
        pt.intensity = config_->correctIntensity(n, intensity);
      else
        pt.intensity = intensity;

      rcfg.project(distance, &pt);

      pcd.push_back(pt);
    }
  }
}


} // namespace stdr_velodyne
