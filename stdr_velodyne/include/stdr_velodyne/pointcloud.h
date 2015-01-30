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

#ifndef __STDR_VELODYNE__PKT_TO_PCD_H__
#define __STDR_VELODYNE__PKT_TO_PCD_H__


#include <stdr_velodyne/point_type.h>
#include <stdr_velodyne/config.h>
#include <velodyne_msgs/VelodynePacket.h>

namespace stdr_velodyne {

/** \brief A class to convert a raw velodyne packet into a pointcloud.
  *
  * It reads the configuration data from the static configuration instance
  * (see velodyne::Configuration::getStaticConfigurationInstance()).
  * It does not transform the points to a different frame.
  */
class PacketToPcd
{
public:
  explicit PacketToPcd(const ros::NodeHandle &nh);

  /// Converts the packet to a pcd and append the points to the output pcd.
  /// Does not transform to a different frame. And does NOT set the header of
  /// the output point cloud.
  void processPacket(const velodyne_msgs::VelodynePacket& input, PointCloud& output) const;

protected:
  bool calibrate_intensities_;
  double max_dist_;

  /// static configuration instance
  Configuration::ConstPtr config_;

  /// Whether to filter the points that fall on junior
  bool filter_points_on_car_;
  /// The box delimiting the points to filter (axis oriented, in velodyne frame)
  Eigen::Vector3d pt_on_car_max_, pt_on_car_min_;

  /// We need to know whether the model is the VLP-16 because in that case the
  /// last 16 returns (in the 32 returns block from velodyne_rawdata::raw_block_t)
  /// correspond to a different encoder value that needs to be interpolated.
  bool vlp16_;
};


} // namespace stdr_velodyne


#endif // __STDR_VELODYNE__PKT_TO_PCD_H__
