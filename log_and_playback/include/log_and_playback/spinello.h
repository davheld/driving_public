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

#ifndef __LOG_AND_PLAYBACK__SPINELLO_H
#define __LOG_AND_PLAYBACK__SPINELLO_H


#include <string>
#include <iostream>
#include <fstream>

#include <stdr_msgs/ApplanixPose.h>
#include <stdr_velodyne/point_type.h>
#include <stdr_velodyne/config.h>

#include <log_and_playback/abstract_data_reader.h>


namespace log_and_playback
{

class SpinelloReader : public AbstractDataReader
{
public:
  SpinelloReader();
  void open(const std::string & dirname);
  void close() {}

  /// Advances one data into the file. Returns false on EOF.
  bool next();
  bool ok() const { return ok_; }

  ros::Time time() const { return time_; }

  stdr_velodyne::PointCloud::ConstPtr instantiateVelodyneSpin() const { return spin_; }
  stdr_msgs::ApplanixPose::ConstPtr instantiateApplanixPose() const { return pose_; }

protected:
  /// static configuration instance
  stdr_velodyne::Configuration::ConstPtr config_;

private:
  stdr_velodyne::PointCloud::Ptr spin_;
  stdr_msgs::ApplanixPose::Ptr pose_;
  ros::Time time_;
  bool ok_;

  std::vector<std::string> ezd_files_;
  unsigned ezd_file_cnt_;
  unsigned read_applanix_;

  /// Whether to filter the points that fall on junior
  bool filter_points_on_car_;
  /// The box delimiting the points to filter (axis oriented, in velodyne frame)
  Eigen::Vector3d pt_on_car_max_, pt_on_car_min_;

  void makeApplanixData();
  void readVelodyneData();
};

}

#endif // __LOG_AND_PLAYBACK__SPINELLO_H
