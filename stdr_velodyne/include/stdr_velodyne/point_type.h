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

#ifndef __STDR_VELODYNE__POINT_TYPE_H__
#define __STDR_VELODYNE__POINT_TYPE_H__

#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>

// not strictly needed here but will be needed by most user code
#include <pcl_conversions/pcl_conversions.h>


namespace stdr_velodyne
{
/** \brief A point structure representing euclidian xyz coordinates, the rgb
  * color, and other data pertinent to the velodyne.
  */
struct EIGEN_ALIGN16 PointType : pcl::PointXYZRGB
{
  double timestamp; ///< timestamp
  float intensity;  ///< intensity (using float to be compatible with pcl::PointXYZI), but range is still [0-255]
  float distance;   ///< distance to the velodyne
  uint16_t encoder; ///< rotational encoder value
  float h_angle;    ///< horizontal angle
  uint8_t beam_id;  ///< beam id [0-64], unsorted
  uint8_t beam_nb;  ///< beam number [0-64], sorted from top to bottom
  float v_angle;    ///< vertical angle of that beam

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  inline PointType ()
    : timestamp(.0), intensity(.0f), distance(.0f), encoder(0), h_angle(.0f)
    , beam_id(0), beam_nb(0), v_angle(.0f)
  {
  }

  template <class T>
  T asPoint() const
  { T t; t.x = x; t.y = y; t.z = z; return t; }

};

} // namespace stdr_velodyne

POINT_CLOUD_REGISTER_POINT_STRUCT (stdr_velodyne::PointType,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (uint32_t, rgb, rgb)
    (double, timestamp, timestamp)
    (float, intensity, intensity)
    (float, distance, distance)
    (uint16_t, encoder, encoder)
    (float, h_angle, h_angle)
    (uint8_t, beam_id, beam_id)
    (uint8_t, beam_nb, beam_nb)
    (float, v_angle, v_angle));


namespace stdr_velodyne {

typedef pcl::PointCloud<PointType> PointCloud;
typedef boost::shared_ptr<PointCloud> PointCloudPtr;
typedef boost::shared_ptr<PointCloud const> PointCloudConstPtr;

} // namespace stdr_velodyne

#endif // __STDR_VELODYNE__POINT_TYPE_H__
