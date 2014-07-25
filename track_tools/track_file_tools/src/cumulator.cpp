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



/** This program takes a trk file (see track_file_io) as input, a
  * track number, and will cumulate all the frames for that track into one
  * single point cloud.
  *
  * If the chosen track is a static object, then the result should be a
  * nice dense point cloud representation of that object.
  *
  * The resulting point cloud can be viewed with pcl_viewer <pcd file>
  * (press l to see the list of color handlers, and then press the number for
  * the correct color handler to see the colors).
  *
  * Unfortunately, in most cases the pose errors are relatively too large,
  * and the resulting point cloud is not that great looking.
  */

#include <vector>
#include <algorithm>
#include <boost/shared_ptr.hpp>
#include <boost/foreach.hpp>
#include <boost/program_options.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/common/centroid.h>
#include <ros/assert.h>
#include <pcl_conversions/pcl_conversions.h>
#include <track_file_io/track_file_io.h>
#include "common.h"

namespace bpo = boost::program_options;

template <class PointT>
void demean(pcl::PointCloud<PointT>& pcd)
{
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(pcd, centroid);
  pcl::demeanPointCloud(pcd, centroid, pcd);
}


int main(int argc, char **argv)
{
  std::string input_tm_fn, output_pcd_fn;
  unsigned track_nb;
  bpo::options_description opcd;
  opcd.add_options()
    ("help,h", "produces this help message")
    ("input,i", bpo::value<std::string>(&input_tm_fn)->required(), "input tm file")
    ("track,t", bpo::value<unsigned>(&track_nb)->required(), "track number")
    ("output,o", bpo::value<std::string>(&output_pcd_fn)->required(), "output tm file")
    ;
  bpo::positional_options_description pd;
  pd.add("input", 1).add("track", 1).add("output", 1);
  bpo::variables_map vm;
  bpo::store(bpo::command_line_parser(argc, argv).options(opcd).positional(pd).run(), vm);
  if( vm.count("help") ) {
    std::cout << "track_cumulator: takes a tm track file, and a track number, cumulates all the points" <<std::endl;
    std::cout << "from that track into one point cloud and save it to the output file." << std::endl;
    std::cout << std::endl;
    std::cout << opcd << std::endl;
    return 0;
  }
  bpo::notify(vm);

  // TODO: do not load the whole set of tracks
  track_file_io::Tracks tracks;
  track_file_io::load(input_tm_fn, tracks);

  ROS_ASSERT(track_nb<tracks.tracks.size());

  std::vector<const sensor_msgs::PointCloud2 *> clouds;
  for( unsigned f=0; f<tracks.tracks[track_nb].frames.size(); ++f )
    clouds.push_back( & tracks.tracks[track_nb].frames[f].cloud );
  sensor_msgs::PointCloud2 pcd;
  concatSMP2s(pcd, clouds);

  bool has_color = false;
  bool has_intensity = false;
  for(unsigned i=0; i<pcd.fields.size(); ++i) {
    if( pcd.fields[i].name=="rgb" )
      has_color = true;
    else if( pcd.fields[i].name=="intensity" )
      has_intensity = true;
  }

  if( has_color ) {
    pcl::PointCloud<pcl::PointXYZRGB> cloud;
    pcl::fromROSMsg(pcd, cloud);
    demean(cloud);
    pcl::io::savePCDFileBinary(output_pcd_fn, cloud);
  }
  else if( has_intensity ) {
    pcl::PointCloud<pcl::PointXYZI> cloud;
    pcl::fromROSMsg(pcd, cloud);
    demean(cloud);
    pcl::io::savePCDFileBinary(output_pcd_fn, cloud);
  }
  else {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(pcd, cloud);
    demean(cloud);
    pcl::io::savePCDFileBinary(output_pcd_fn, cloud);
  }

  return 0;
}
