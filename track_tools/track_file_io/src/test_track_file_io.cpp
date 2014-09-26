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
  return boost::filesystem3::unique_path(boost::filesystem3::temp_directory_path()
                                         /"test-track-file-io-%%%%%%.trk"
                                         ).native();
}

pcl::PointCloud<pcl::PointXYZ> makePCD(int &idx, int npts)
{
  pcl::PointCloud<pcl::PointXYZ> pcd;
  pcl::PointXYZ p;

  while(pcd.size()<npts) {
    for(int i=0; i<3 && pcd.size()<npts; ++i) {
      const int j = i % 3;
      p.x = j==0 ? idx : 0;
      p.y = j==1 ? idx : 0;
      p.z = j==2 ? idx : 0;
      ++idx;
      pcd.push_back(p);
    }
  }

  return pcd;
}

bool testPCD(const pcl::PointCloud<pcl::PointXYZ> &pcd, int idx)
{
  const pcl::PointCloud<pcl::PointXYZ> refpcd = makePCD(idx, pcd.size());
  bool ok = true;
  for(unsigned i=0; i<pcd.size() && ok; ++i) {
    ok &= pcd[i].x==refpcd[i].x;
    ok &= pcd[i].y==refpcd[i].y;
    ok &= pcd[i].z==refpcd[i].z;
  }
  return ok;
}

bool testPCD(const sensor_msgs::PointCloud2 &pc2, int idx)
{
  pcl::PointCloud<pcl::PointXYZ> pcd;
  pcl::fromROSMsg(pc2, pcd);
  return testPCD(pcd, idx);
}



TEST(TrackFileIO, WriteAndRead)
{
  Track track;
  Frame frame;
  pcl::PointCloud<pcl::PointXYZ> pcd;
  pcl::PointXYZ p;
  int pval = 0;

  const std::string path = tmptrk();

  // First let's create a track file
  {
    TrackFileWriter writer(path);
    writer.setVelodynePose(geometry_msgs::Pose());

    pcl::toROSMsg(makePCD(pval, 3), frame.cloud);

    track.id = 0;
    frame.stamp.fromSec(1);
    track.frames.push_back(frame);
    frame.stamp.fromSec(2);
    track.frames.push_back(frame);
    writer.write(track);

    track.id = 1;
    frame.stamp.fromSec(1);
    track.frames.push_back(frame);
    frame.stamp.fromSec(2);
    track.frames.push_back(frame);
    writer.write(track);
  }


  // then read the same file and compare
  {
    Tracks tracks;
    load(path, tracks);

    EXPECT_EQ(2, tracks.tracks.size());
    EXPECT_EQ(2, tracks.tracks[0].frames.size());

    EXPECT_EQ(1, tracks.tracks[0].frames[0].stamp.toSec());
    EXPECT_TRUE(testPCD(tracks.tracks[0].frames[0].cloud, 0));

    EXPECT_EQ(2, tracks.tracks[0].frames[1].stamp.toSec());
    EXPECT_TRUE(testPCD(tracks.tracks[0].frames[0].cloud, 0));

    EXPECT_EQ(1, tracks.tracks[1].frames[0].stamp.toSec());
    EXPECT_TRUE(testPCD(tracks.tracks[0].frames[0].cloud, 0));

    EXPECT_EQ(2, tracks.tracks[1].frames[1].stamp.toSec());
    EXPECT_TRUE(testPCD(tracks.tracks[0].frames[0].cloud, 0));
  }
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
