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


/** This program takes a trk file as input (see track_file_io) and
  * displays the number of tracks/frames/points it contains, broken down by
  * labels.
  */

#include <sys/stat.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <math.h>

#include <pcl/point_types.h>
#include <pcl/common/centroid.h>

#include <Eigen/Core>

#include <pcl_conversions/pcl_conversions.h>

#include <track_file_io/track_file_io.h>

using namespace std;

string usageString()
{
  ostringstream oss;
  oss << "Usage: " << endl;
  oss << "  track_stats TRACKFILE [TRACKFILE ...]" << endl;
  return oss.str();
}

Eigen::Vector3d computeCentroid(const sensor_msgs::PointCloud2& pc)
{
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg(pc, cloud);
  Eigen::Vector4d centroid;
  pcl::compute3DCentroid(cloud, centroid);
  return centroid.head<3>();
}

double computeDisplacement(const sensor_msgs::PointCloud2& pc,
                           const sensor_msgs::PointCloud2& prev)
{
  return (computeCentroid(pc)-computeCentroid(prev)).norm();
}


int main(int argc, char** argv)
{
  if(argc < 2) {
    cout << usageString() << endl;
    return 1;
  }

  map<string, int> num_clouds;
  map<string, int> num_tracks;
  map<string, unsigned long> num_points;
  for(int i=1; i<argc; ++i) {
    // -- Load the tracks
    cout << "Working on " << argv[i] << endl;
    track_file_io::Tracks tracks;
    track_file_io::load(argv[i], tracks);

    for(size_t j=0; j<tracks.tracks.size(); ++j) {
      track_file_io::Track& tr = tracks.tracks[j];
      num_clouds[tr.label] += tr.frames.size();
      num_tracks[tr.label]++;
      for(size_t k=0; k<tr.frames.size(); ++k)
        num_points[tr.label] += tr.frames[k].cloud.width;
    }
  }

  int total_tracks = 0;
  int total_clouds = 0;
  unsigned long total_points = 0;
  cout << endl << "Track Statistics: " << endl;
  for(map<string, int>::iterator it = num_tracks.begin(); it!=num_tracks.end(); ++it) {
    cout << it->first << " tracks: " << it->second << endl;
    total_tracks += it->second;
  }
  cout << "Total tracks: " << total_tracks << endl << endl;
  cout << "Cloud Statistics: " << endl;
  for(map<string, int>::iterator it = num_clouds.begin(); it!=num_clouds.end(); ++it) {
    cout << it->first << " clouds: " << it->second << endl;
    total_clouds += it->second;
  }
  cout << "Total clouds: " << total_clouds << endl << endl;
  cout << "Point Statistics: " << endl;
  for(map<string, unsigned long>::iterator it = num_points.begin(); it!=num_points.end(); ++it) {
    cout << it->first << " points: " << it->second << endl;
    total_points += it->second;
  }
  cout << "Total points: " << total_points << endl << endl;
}
