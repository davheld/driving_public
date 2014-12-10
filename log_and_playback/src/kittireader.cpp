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

#include <typeinfo>

#include <boost/foreach.hpp>
#include <boost/make_shared.hpp>

#include <angles/angles.h>

#include <stdr_lib/rosparam_helpers.h>
#include <log_and_playback/kittireader.h>


/*
The KittiVeloWriter matlab script sets the timestamp of the first point of
the first spin to t=100ms
In our convention, the spin's stamp is the stamp of the last point in the
spin, so 200ms for the first spin.
We want a simple relation between the frame number and spin stamp,
so we remove 200ms. However, because that would lead to a negative time for
the points at the begining of the spin, we add 1,000,000 seconds
(which is just an arbitry large number).
With this, the stamp of the first spin (frame 0) should be 1,000,000s
*/
static const uint64_t vel_t_off = uint64_t((1e6 - 0.2)*1e6); //expressed in micro-seconds

/*
The IMU data is stamped (as per imu_kitti2.cpp) assuming a period of 100ms per
frame, and starting with frame 0 at t=0.
However, the time of the first point in the first velodyne spin is
100ms + 1e6s - 200ms (see vel_t_off above).
The IMU/GPS time corresponds to when the velodyne was in front.
That means that the stamp for the first IMU/GPS data should be 1e6 - 50ms
*/
static const double imu_t_off = 1e6 - 0.05;




namespace log_and_playback
{

void KittiApplanixReader::open(const std::string & filename)
{
  file_.open(filename.c_str(), std::ios_base::in);
  ok_ = true;
  old_hw_timestamp_ = 0;
}


KittiApplanixReader::~KittiApplanixReader()
{
  close();
}

void KittiApplanixReader::close()
{
  file_.close();
}

stdr_msgs::ApplanixPose::Ptr KittiApplanixReader::parseApplanix(
    const std::string & line, double smooth_x, double smooth_y, double smooth_z)
{
  double data[25];
  std::stringstream ss(line);


  uint64_t epoch_time;
  ss >> epoch_time;
  const double ep_time = static_cast<double>(epoch_time) * 1e-6 + imu_t_off;


  for(int i=0; i<25; i++)
    ss >> data[i];

  stdr_msgs::ApplanixPose::Ptr pose( new stdr_msgs::ApplanixPose );
  pose->latitude = data[0];
  pose->longitude = data[1];
  pose->altitude = data[2];
  pose->roll = data[3];
  pose->pitch = data[4];
  pose->yaw = data[5];
  pose->vel_north = (float)(data[6]);
  pose->vel_east = (float)(data[7]);
  pose->vel_up = (float) (data[10]);
  pose->rate_roll = data[17];
  pose->rate_pitch = data[18];
  pose->rate_yaw = data[19];
  pose->accel_x = data[11];
  pose->accel_y = data[12];
  pose->accel_z = data[13];
  pose->wander =0;
  pose->id = 0;
  pose->speed = (float)(hypot(pose->vel_north, pose->vel_east));
  pose->track = (float)(atan2(pose->vel_north, pose->vel_east));

  if (old_hw_timestamp_ !=0){
    double dt = ep_time - old_hw_timestamp_;
    pose->smooth_x = smooth_x + pose->vel_east * dt;
    pose->smooth_y = smooth_y + pose->vel_north * dt;
    pose->smooth_z = smooth_z + pose->vel_up * dt;

  }
  else{
    pose->smooth_x = 0;
    pose->smooth_y = 0;
    pose->smooth_z = 0;
  }
  old_hw_timestamp_ = pose->hardware_timestamp = ep_time;
  pose->header.stamp.fromSec(ep_time);
  return pose;
}

bool KittiApplanixReader::next()
{
  double smooth_x=0, smooth_y=0, smooth_z=0;
  if( pose_ ) {
    smooth_x = pose_->smooth_x;
    smooth_y = pose_->smooth_y;
    smooth_z = pose_->smooth_z;
  }

  pose_.reset();

  while( true )
  {
    std::string line;
    if( ok_ && std::getline(file_, line) ) {
      pose_ = parseApplanix(line, smooth_x, smooth_y, smooth_z);
      if( pose_ ) {
        time_ = pose_->header.stamp;
        return true;
      }
    }
    else {
      ok_ = false;
      return ok_;
    }
  }

  return ok_;
}

stdr_msgs::ApplanixPose::ConstPtr
KittiApplanixReader::instantiateApplanixPose() const
{
  return pose_;
}




KittiVeloReader::KittiVeloReader()
{
  ros::NodeHandle nh("/driving/velodyne");

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


  config_ = stdr_velodyne::Configuration::getStaticConfigurationInstance();
  ok_ = false;
  spin_ = boost::make_shared<stdr_velodyne::PointCloud>();
}

KittiVeloReader::~KittiVeloReader()
{
  close();
}

void KittiVeloReader::open(const std::string & filename)
{
  vfile_.open(filename.c_str());
  ROS_ASSERT(vfile_);
  ok_ = true;
}

void KittiVeloReader::close()
{
  vfile_.close();
  ok_ = false;
}

bool KittiVeloReader::next()
{
  ROS_ASSERT(config_);
  ROS_ASSERT(config_->valid());

  if( !vfile_ ) {
    ok_ = false;
    return false;
  }

  struct __attribute__ ((__packed__)) Header {
    uint32_t num_points;
    uint64_t t_start, t_end; // microseconds
  } header;

  vfile_.read((char *)(&header), sizeof(Header));

  if( !vfile_ ) {
    ok_ = false;
    return false;
  }

  if( header.num_points > 200000 ) {
    ROS_ERROR_STREAM("Got a spin with " <<header.num_points <<" points. This is probably the end of the file");
    ok_ = false;
    return false;
  }

  spin_.reset(new stdr_velodyne::PointCloud);
  spin_->reserve(header.num_points);

  spin_->header.frame_id = "velodyne";
  spin_->header.stamp = header.t_end + vel_t_off;
  time_ = pcl_conversions::fromPCL(spin_->header).stamp;


  struct __attribute__ ((__packed__)) Point {
    float x, y, z;
    float intensity;
    float h_angle;
    uint8_t beam_id;
    float distance;
  } point;

  stdr_velodyne::PointType pt;
  for( int i =0; i<header.num_points; i++) {
    vfile_.read((char *)(&point), sizeof(Point));

    if( !vfile_ ) {
      ok_ = false;
      return false;
    }

    if(point.beam_id==0 || point.beam_id-1>=stdr_velodyne::NUM_LASERS)
      continue;

    if( filter_points_on_car_
        && point.x > pt_on_car_min_.x() && point.x < pt_on_car_max_.x()
        && point.y > pt_on_car_min_.y() && point.y < pt_on_car_max_.y()
        && point.z > pt_on_car_min_.z() && point.z < pt_on_car_max_.z() )
      continue;

    const stdr_velodyne::RingConfig & rcfg = config_->getRingConfig(point.beam_id - 1);

    pt.x = point.x;
    pt.y = point.y;
    pt.z = point.z;
    pt.intensity = point.intensity * 255;
    pt.h_angle = angles::from_degrees(point.h_angle);
    pt.encoder = (360-point.h_angle) * 100;
    pt.v_angle = rcfg.vert_angle_.getRads();
    pt.beam_id = point.beam_id - 1;
    pt.beam_nb = point.beam_id - 1; //config_->getBeamNumber(pt.beam_id);
    pt.distance = point.distance;

    // get amount of rotation since the back
    // pt.h_angle is given from the front CCW
    const double alpha = fmod(540.0-point.h_angle, 360.0); //i.e. (2*M_PI-pt.h_angle)-(-M_PI)
    const double r = alpha / 360.0;
    // discretize in 200 sectors (velodyne points are stamped 600 by 600, which makes about 200 sectors)
    // stdr_velodyne::transform_scan takes advantage of this to do less transform lookups
    const double a = round(r * 200) / 200;
    // interpolate the timestamp
    pt.timestamp = (a * (header.t_end - header.t_start) + header.t_start + vel_t_off) * 1e-6;

    // add to pointcloud
    spin_->push_back(pt);
  }


  ok_ = true;
  return ok_;
}

}
