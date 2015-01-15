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


#include <boost/filesystem.hpp>
#include <ros/console.h>
#include <angles/angles.h>
#include <stdr_lib/rosparam_helpers.h>
#include <log_and_playback/spinello.h>

namespace log_and_playback
{


static const double period = 0.2;



SpinelloReader::SpinelloReader()
  : read_applanix_(5)
{
  ros::NodeHandle nh("/driving/velodyne");

  // The filtering is normally done in the stdr_velodyne/pointcloud node
  // however, with spinello, as with kitti, the velodyne data already comes in
  // velodyne format and so does not go through the stdr_velodyne/pointcloud
  // node, so we filter it here.
  // Ideally the box would be defined in base_link coordinates, and its velodyne
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

void SpinelloReader::makeApplanixData()
{
  // create a fixed pose message (the data was recorded with the car not moving)
  pose_.reset( new stdr_msgs::ApplanixPose );
  pose_->latitude = 0;
  pose_->longitude = 0;
  pose_->altitude = 0;
  pose_->roll = 0;
  pose_->pitch = 0;
  pose_->yaw = 0;
  pose_->vel_north = 0;
  pose_->vel_east = 0;
  pose_->vel_up = 0;
  pose_->rate_roll = 0;
  pose_->rate_pitch = 0;
  pose_->rate_yaw = 0;
  pose_->accel_x = 0;
  pose_->accel_y = 0;
  pose_->accel_z = 0;
  pose_->wander =0;
  pose_->id = 0;
  pose_->speed = 0;
  pose_->track = 0;
  pose_->smooth_x = 0;
  pose_->smooth_y = 0;
  pose_->smooth_z = 0;
  pose_->hardware_timestamp = time_.toSec();
  pose_->header.stamp = time_;
}

void SpinelloReader::open(const std::string & dirname)
{
  ezd_files_.clear();
  ezd_file_cnt_ = 0;

  namespace fs = boost::filesystem;
  fs::path dirpath(dirname);
  ROS_ASSERT( fs::exists(dirpath) && fs::is_directory(dirpath) );
  fs::directory_iterator end_iter;
  for( fs::directory_iterator dir_iter(dirpath); dir_iter != end_iter; ++dir_iter ) {
    if( fs::is_regular_file(dir_iter->status()) && boost::algorithm::ends_with(dir_iter->path().native(), ".ezd") ) {
      ezd_files_.push_back(dir_iter->path().native());
    }
  }
  std::sort(ezd_files_.begin(), ezd_files_.end());

  ok_ = true;
}

struct pointCompare
{
  bool operator() (const stdr_velodyne::PointType & a,
                   const stdr_velodyne::PointType & b)
  {
    // This is the angle from the back, clockwise (same as the velodyne rotation)
    return angles::normalize_angle_positive(3*M_PI-a.h_angle) <
        angles::normalize_angle_positive(3*M_PI-b.h_angle);
  }
};

void SpinelloReader::readVelodyneData()
{
  std::ifstream file_(ezd_files_[ezd_file_cnt_].c_str());

  spin_.reset(new stdr_velodyne::PointCloud);
  spin_->reserve(150000);

  std_msgs::Header h;
  h.frame_id = "velodyne";
  h.stamp = time_ + ros::Duration(period);
  pcl_conversions::toPCL(h, spin_->header);

  stdr_velodyne::PointType pt;
  std::string line;
  while( std::getline(file_, line) )
  {
    std::stringstream ss(line);
    // we cannot stream directly into pt.beam_id because it's a uint8_t, so
    // we get the value of the ascii char instead of the integer
    int beam_id;
    if( !(ss >> pt.x >> pt.y >> pt.z >> beam_id) ) {
      ROS_WARN_STREAM("Failed to read from " <<ezd_files_[ezd_file_cnt_]);
      ROS_BREAK();
    }
    ROS_ASSERT(beam_id>=0 && beam_id<64);
    pt.beam_id = beam_id;

    if( filter_points_on_car_
        && pt.x > pt_on_car_min_.x() && pt.x < pt_on_car_max_.x()
        && pt.y > pt_on_car_min_.y() && pt.y < pt_on_car_max_.y()
        && pt.z > pt_on_car_min_.z() && pt.z < pt_on_car_max_.z() )
      continue;

    const stdr_velodyne::RingConfig & rcfg = config_->getRingConfig(pt.beam_id);

    pt.intensity = 200; // arbitrary number (intensity data is not present in the dataset)
    pt.h_angle = atan2(pt.y, pt.x);
    pt.encoder = angles::to_degrees(angles::normalize_angle_positive(2*M_PI-pt.h_angle)) * 100;
    pt.v_angle = rcfg.vert_angle_.getRads();
    pt.distance = sqrt(pow(pt.x, 2)+pow(pt.y, 2)+pow(pt.z, 2));
    pt.beam_nb = config_->getBeamNumber(pt.beam_id);

    // get amount of rotation since the back
    // pt.h_angle is given from the front CCW
    const double alpha = angles::normalize_angle_positive(M_PI-pt.h_angle);
    const double r = alpha / (2*M_PI);
    // discretize in 200 sectors (velodyne points are stamped 600 by 600, which makes about 200 sectors)
    // stdr_velodyne::transform_scan takes advantage of this to do less transform lookups
    const double a = round(r * 200) / 200;
    // interpolate the timestamp
    pt.timestamp = (a * period + time_.toSec());

    // add to pointcloud
    spin_->push_back(pt);
  }

  // the RingOrganizedSpin in the perception module assumes the points to be
  // sorted according to their rotation angle, starting from the back and
  // turning clockwise
  std::sort(spin_->begin(), spin_->end(), pointCompare());
}

bool SpinelloReader::next()
{
  ROS_ASSERT(config_);
  ROS_ASSERT(config_->valid());

  spin_.reset();
  pose_.reset();

  if( read_applanix_ > 0 ) {
    -- read_applanix_;
    makeApplanixData();
    return true;
  }

  if( ezd_file_cnt_>=ezd_files_.size() ) {
    ok_ = false;
    return false;
  }

  readVelodyneData();

  ++ ezd_file_cnt_;
  time_.fromSec(ezd_file_cnt_ * period);
  read_applanix_ = 1;

  ok_ = true;
  return ok_;
}

}
