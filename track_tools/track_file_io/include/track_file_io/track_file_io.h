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

#ifndef TRACK_FILE_IO_H
#define TRACK_FILE_IO_H


#include <track_file_io/Tracks.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>


class TrackFileWriter
{
  rosbag::Bag bag_;
  bool velodyne_pose_set_, velodyne_pose_saved_;
  geometry_msgs::Pose velodyne_pose_;

public:
  TrackFileWriter(const std::string& filename);
  void setVelodynePose(const geometry_msgs::Pose& velodyne_pose);
  bool isVelodynePoseSet() const { return velodyne_pose_set_; }
  void write(const track_file_io::Track&);
};

class TrackFileReader
{
  rosbag::Bag bag_;
  rosbag::View tracks_view_;
  rosbag::View::iterator track_it_;
  geometry_msgs::Pose vel_pose_;
  unsigned n_tracks_;

public:
  TrackFileReader(const std::string& filename);
  const geometry_msgs::Pose& getVelodynePose() const { return vel_pose_; }
  unsigned getNTracks() const { return n_tracks_; }
  bool read(track_file_io::Track& track);
};

namespace track_file_io
{
void save(const std::string& filename, const track_file_io::Tracks& tracks);
void load(const std::string& filename, track_file_io::Tracks& tracks);
}


#endif //TRACK_FILE_IO_H
