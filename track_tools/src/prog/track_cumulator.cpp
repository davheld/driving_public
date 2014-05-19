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



/** This program takes a tm file as input (see track_tools/TrackManager), a
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
#include <track_tools/track_manager.h>

namespace bpo = boost::program_options;


// -----------------------------------------------------------------------------
// Global variables

pcl::PointCloud<pcl::PointXYZRGB> pcd;


// -----------------------------------------------------------------------------


void appendPoints(const sensor_msgs::PointCloud& cloud)
{
  pcd.points.reserve( pcd.size() + cloud.points.size() );

  const std::vector<float>* colors = 0;
  const std::vector<float>* intensities = 0;
  for(unsigned i=0; i<cloud.channels.size(); ++i) {
    if( cloud.channels[i].name=="rgb" )
      colors = &(cloud.channels[i].values);
    else if( cloud.channels[i].name=="" || cloud.channels[i].name=="intensity" )
      intensities = &(cloud.channels[i].values);
  }

  for(unsigned i=0; i<cloud.points.size(); ++i) {
    pcl::PointXYZRGB p;
    p.x = cloud.points[i].x;
    p.y = cloud.points[i].y;
    p.z = cloud.points[i].z;
    if( colors ) {
      p.rgb = colors->at(i);
      //std::cout <<"color: " <<(int)p.r <<", " <<(int)p.g <<", " <<(int)p.b <<std::endl;
    }
    else if( intensities ) {
      const unsigned char I = intensities->at(i);
      p.r = I;
      p.g = I;
      p.b = I;
    }
    else {
      p.r = 255;
      p.g = 255;
      p.b = 255;
    }
    pcd.push_back(p);
  }
}

void demean()
{
  Eigen::Vector4f centroid;
  pcl::compute3DCentroid(pcd, centroid);
  pcl::PointCloud<pcl::PointXYZRGB> out;
  pcl::demeanPointCloud(pcd, centroid, out);
  pcd = out;
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

  track_manager::TrackManager itmanager;
  if( !itmanager.load(input_tm_fn) ) {
    std::cerr <<"Could not load " <<input_tm_fn <<std::endl;
    return 1;
  }

  ROS_ASSERT(track_nb<itmanager.tracks_.size());

  for( unsigned f=0; f<itmanager.tracks_[track_nb]->frames_.size(); ++f )
    appendPoints( *(itmanager.tracks_[track_nb]->frames_[f]->cloud_) );

  demean();

  pcl::io::savePCDFileBinary(output_pcd_fn, pcd);

  return 0;
}
