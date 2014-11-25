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
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <track_file_io/track_file_io.h>
#include <track_file_io/manipulations.h>
#include <gtest/gtest.h>

using namespace track_file_io;

std::string tmptrk()
{
  return boost::filesystem::unique_path(boost::filesystem::temp_directory_path()
                                        /"test-track-file-io-%%%%%%.trk"
                                        ).native();
}

pcl::PointCloud<pcl::PointXYZ> randomPCD(unsigned npts)
{
  pcl::PointCloud<pcl::PointXYZ> pcd;
  pcd.points.resize(npts);
  for(unsigned i=0; i<npts; ++i) {
    pcd[i].x = ((double) rand()) / RAND_MAX;
    pcd[i].y = ((double) rand()) / RAND_MAX;
    pcd[i].z = ((double) rand()) / RAND_MAX;
  }
  return pcd;
}

Tracks randomTracks(unsigned nTracks)
{
  Tracks tracks;

  for(unsigned i=0; i<nTracks; ++i) {
    Track track;
    track.id = i;
    for(unsigned j=0; j<10; ++j) {
      Frame frame;
      frame.stamp.fromSec(j+1);
      pcl::toROSMsg(randomPCD(10), frame.cloud);
      track.frames.push_back(frame);
    }
    tracks.tracks.push_back(track);
  }
  return tracks;
}

namespace std_msgs {
bool operator== (const Header &h1, const Header &h2)
{
  return h1.frame_id==h2.frame_id && h1.stamp==h2.stamp && h1.seq==h2.seq;
}
} //namespace std_msgs

namespace geometry_msgs {
bool operator== (const Point &p1, const Point &p2)
{
  return p1.x==p2.x && p1.y==p2.y && p1.z==p2.z;
}
bool operator== (const Quaternion &q1, const Quaternion &q2)
{
  return q1.x==q2.x && q1.y==q2.y && q1.z==q2.z && q1.w==q2.w;
}
bool operator== (const Pose &p1, const Pose &p2)
{
  return p1.position==p2.position && p1.orientation==p2.orientation;
}
} //namespace geometry_msgs

namespace sensor_msgs {
bool operator== (const PointField &pf1, const PointField &pf2)
{
  return pf1.count==pf2.count && pf1.datatype==pf2.datatype && pf1.name==pf2.name
      && pf1.offset==pf2.offset;
}

bool operator== (const PointCloud2 &pcd1, const PointCloud2 &pcd2)
{
  return pcd1.fields==pcd2.fields && pcd1.header==pcd2.header
      && pcd1.height==pcd2.height && pcd1.is_bigendian==pcd2.is_bigendian
      && pcd1.is_dense==pcd2.is_dense && pcd1.point_step==pcd2.point_step
      && pcd1.row_step==pcd2.row_step && pcd1.width==pcd2.width
      && pcd1.data==pcd2.data;
}
} //namespace sensor_msgs

namespace track_file_io {
bool operator== (const Frame &frame1, const Frame &frame2)
{
  return frame1.stamp == frame2.stamp && frame1.robot_pose == frame2.robot_pose && frame1.cloud == frame2.cloud;
}

bool operator== (const Track &track1, const Track &track2)
{
  return track1.id == track2.id && track1.label == track2.label && track1.frames == track2.frames;
}

bool operator== (const Tracks &tracks1, const Tracks &tracks2)
{
  return tracks1.velodyne_pose == tracks2.velodyne_pose && tracks1.tracks == tracks2.tracks;
}
} //namespace track_file_io




TEST(TrackFileIO, WriteAndRead)
{
  const std::string path = tmptrk();
  const Tracks ref_tracks = randomTracks(10);

  save(path, ref_tracks);
  const Tracks tracks = load(path);
  EXPECT_TRUE(ref_tracks==tracks);
}

TEST(TrackFileIO, Copy)
{
  const Tracks ref_tracks = randomTracks(10);
  {
    Tracks tracks;
    tracks = ref_tracks;
    EXPECT_TRUE(ref_tracks==tracks);
  }
  {
    const Tracks tracks(ref_tracks);
    EXPECT_TRUE(ref_tracks==tracks);
  }
}

TEST(TrackFileIO, Delete)
{
  const Tracks ref_tracks = randomTracks(10);
  Tracks tracks = ref_tracks;

  {
    Tracks::_tracks_type::const_iterator it = find(tracks, 3);
    EXPECT_FALSE(it==tracks.tracks.end());
    deleteTrack(tracks, 3);
    it = find(tracks, 3);
    EXPECT_TRUE(it==tracks.tracks.end());
    EXPECT_EQ(9, tracks.tracks.size());
  }

  EXPECT_FALSE(ref_tracks==tracks);

  for(unsigned id=0; id<10; ++id) {
    if( id==3 ) continue;

    const Tracks::_tracks_type::const_iterator ref_it = find(ref_tracks, id);
    EXPECT_FALSE(ref_it==ref_tracks.tracks.end());
    const Tracks::_tracks_type::const_iterator it = find(tracks, id);
    EXPECT_FALSE(it==tracks.tracks.end());

    EXPECT_TRUE(*ref_it == *it);
  }
}

unsigned nPts(const Tracks& tracks, Track::_id_type id)
{
  unsigned n = 0;
  const Tracks::_tracks_type::const_iterator it = find(tracks, id);
  ROS_ASSERT(it!=tracks.tracks.end());
  BOOST_FOREACH(const Frame& f, it->frames) {
    n += f.cloud.width;
  }
  return n;
}

TEST(TrackFileIO, Merge)
{
  const Tracks ref_tracks = randomTracks(10);
  Tracks tracks = ref_tracks;
  Tracks::_tracks_type::const_iterator it;

  {
    it = find(tracks, 3);
    EXPECT_FALSE(it==tracks.tracks.end());
    it = find(tracks, 7);
    EXPECT_FALSE(it==tracks.tracks.end());

    mergeTracks(tracks, 3, 7);

    it = find(tracks, 7);
    EXPECT_TRUE(it==tracks.tracks.end());
    EXPECT_EQ(9, tracks.tracks.size());
    it = find(tracks, 3);
    EXPECT_FALSE(it==tracks.tracks.end());
  }

  EXPECT_FALSE(ref_tracks==tracks);

  const unsigned npts = nPts(tracks, 3);
  const unsigned nrefpts = nPts(ref_tracks, 3) + nPts(ref_tracks, 7);
  EXPECT_EQ(nrefpts, npts);

  for(unsigned id=0; id<10; ++id) {
    if( id==7 || id==3 )
      continue;

    const Tracks::_tracks_type::const_iterator ref_it = find(ref_tracks, id);
    EXPECT_FALSE(ref_it==ref_tracks.tracks.end());
    it = find(tracks, id);
    EXPECT_FALSE(it==tracks.tracks.end());
    EXPECT_TRUE(*ref_it == *it);
  }
}


int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
