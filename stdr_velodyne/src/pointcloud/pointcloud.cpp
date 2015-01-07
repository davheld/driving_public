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
  ros::NodeHandle nh("/driving/velodyne");

  GET_ROS_PARAM_INFO(nh, "max_dist", max_dist_, std::numeric_limits<double>::max());

  // ideally the box would be defined in base_link coordinates, and its velodyne
  // coordinates would be computed according to the pose of the velodyne.
  // However, in this node we don't have access to the velodyne extrinsincs.
  // So for now we define the box directly in velodyne frame.
  GET_ROS_PARAM_INFO(nh, "filter_points_on_car", filter_points_on_car_, false);
  if( filter_points_on_car_ ) {
    pt_on_car_min_ = Eigen::Vector3d(
          stdr::get_rosparam<double>(nh, "car_bb_min/x"),
          stdr::get_rosparam<double>(nh, "car_bb_min/y"),
          stdr::get_rosparam<double>(nh, "car_bb_min/z"));
    pt_on_car_max_ = Eigen::Vector3d(
          stdr::get_rosparam<double>(nh, "car_bb_max/x"),
          stdr::get_rosparam<double>(nh, "car_bb_max/y"),
          stdr::get_rosparam<double>(nh, "car_bb_max/z"));
  }

  std::string model;
  GET_ROS_PARAM_DEBUG(nh, "model", model, "");
  vlp16_ = (model=="VLP16");
  ROS_INFO_STREAM("" << (vlp16_?"":"not ") << "using the VLP16 mode");
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

  // in VLP16 mode, the second set of 16 returns correspond to a different
  // encoder value that we need to guess. It is so because they wanted to keep
  // the same packet than for the other models, which provide an encoder value
  // for 32 beams.
  // Here, I am computing the average distance between the 12 encoder values.
  unsigned half_encoder_offset = 0;
  if( vlp16_ ) {
    double sum = 0;
    unsigned n = 0;
    for(unsigned i=1; i < velodyne_rawdata::BLOCKS_PER_PACKET; ++i) {
      const double d = double(raw->blocks[i].rotation) - double(raw->blocks[i-1].rotation);
      if( d>0 && d < 18000 ) { // take care of wrapping around 0
        sum += d;
        ++n;
      }
    }
    ROS_ASSERT(n>0);
    half_encoder_offset = sum / n / 2;
  }

  BOOST_FOREACH(const velodyne_rawdata::raw_block_t & block, raw->blocks)
  {
    const unsigned e = block.rotation;
    pt.encoder = e;

    for( unsigned j=0; j<velodyne_rawdata::SCANS_PER_BLOCK; ++j ) {

      // compute the beam index n from the result index and the block header
      unsigned n = j;
      if( vlp16_ ) {
        if( j >= 16 ) {
          n = j - 16;
          pt.encoder = e + half_encoder_offset;
        }
        else {
          n = j;
        }
      }
      else {
        n = config_->getBeamIndex(block.header, j);
      }

      const RingConfig & rcfg = config_->getRingConfig(n);
      const AngleVal & hAngle = rcfg.enc_rot_angle_[pt.encoder];
      pt.h_angle = hAngle.getRads();
      pt.v_angle = rcfg.vert_angle_.getRads();
      pt.beam_id = n;
      pt.beam_nb = config_->getBeamNumber(n);

      const unsigned k = j * velodyne_rawdata::RAW_SCAN_SIZE;

      // this was copied from Jack O'Quin's velodyne_pointcloud/rawdata.cc
      // TODO: this could be a bit simpler if we were using a {ushort, uchar} structure
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

      if( filter_points_on_car_
          && pt.x > pt_on_car_min_.x() && pt.x < pt_on_car_max_.x()
          && pt.y > pt_on_car_min_.y() && pt.y < pt_on_car_max_.y()
          && pt.z > pt_on_car_min_.z() && pt.z < pt_on_car_max_.z() )
        continue;

      pcd.push_back(pt);
    }
  }
}


} // namespace stdr_velodyne
