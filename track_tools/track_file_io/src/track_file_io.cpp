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
#include <track_file_io/track_file_io.h>


static const std::string track_topic = "/tracks";
static const std::string vel_pose_topic = "/velpose";


TrackFileWriter::TrackFileWriter(const std::string& filename)
  : bag_(filename, rosbag::bagmode::Write)
  , velodyne_pose_set_(false)
  , velodyne_pose_saved_(false)
{
}

void TrackFileWriter::setVelodynePose(const geometry_msgs::Pose &velodyne_pose)
{
  velodyne_pose_ = velodyne_pose;
  velodyne_pose_set_ = true;
}

void TrackFileWriter::write(const track_file_io::Track& track)
{
  ROS_ASSERT( velodyne_pose_set_ );
  if( !velodyne_pose_saved_ ) {
    bag_.write(vel_pose_topic, track.frames.front().stamp, velodyne_pose_);
    velodyne_pose_saved_ = true;
  }
  bag_.write(track_topic, track.frames.front().stamp, track);
}


TrackFileReader::TrackFileReader(const std::string& filename)
  : bag_(filename, rosbag::bagmode::Read)
{
  rosbag::View view(bag_, rosbag::TopicQuery(vel_pose_topic));
  bool found_vel_pose = false;
  BOOST_FOREACH(rosbag::MessageInstance const m, view) {
    geometry_msgs::Pose::ConstPtr p = m.instantiate<geometry_msgs::Pose>();
    ROS_ASSERT(p!=NULL);
    vel_pose_ = *p;
    found_vel_pose = true;
    break;
  }
  ROS_ASSERT(found_vel_pose);

  tracks_view_.addQuery(bag_, rosbag::TopicQuery(track_topic));
  track_it_ = tracks_view_.begin();
  n_tracks_ = std::distance(track_it_, tracks_view_.end());
}

bool TrackFileReader::read(track_file_io::Track &track)
{
  if( track_it_==tracks_view_.end() )
    return false;
  track_file_io::Track::ConstPtr t = track_it_->instantiate<track_file_io::Track>();
  ROS_ASSERT(t);
  track = *t;
  ++track_it_;
  return true;
}



namespace track_file_io
{

void save(const std::string& filename,
          const track_file_io::Tracks& tracks)
{
  TrackFileWriter writer(filename);
  writer.setVelodynePose(tracks.velodyne_pose);
  BOOST_FOREACH(const track_file_io::Track& track, tracks.tracks) {
    writer.write(track);
  }
}

void load(const std::string& filename, track_file_io::Tracks& tracks)
{
  TrackFileReader reader(filename);
  tracks.tracks.clear();
  tracks.velodyne_pose = reader.getVelodynePose();
  tracks.tracks.reserve(reader.getNTracks());
  track_file_io::Track track;
  while( reader.read(track) )
    tracks.tracks.push_back(track);
}

} //namespace track_file_io
