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

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <track_file_io/track_file_io.h>

/** This program loads a trk file (see track_file_io) and displays
  * the data in PCLVisualizer, track by track, frame by frame.
  *
  * The data can have color or not.
  */

/* TODO:
 * - Add a filter CLI options: only certain class, min number of points, etc.
 */

boost::shared_ptr<pcl::visualization::PCLVisualizer> visualizer;
track_file_io::Tracks tracks;

int dTrack=0, dFrame=0;
unsigned trackid=0, frameid=0;

enum ColorType { CT_COLOR, CT_INTENSITY, CT_NONE, CT_END };
ColorType color_type = CT_COLOR;

enum FrameType { FT_BASE_LINK, FT_DEMEAN, FT_END };
FrameType frame_type = FT_DEMEAN;

bool refresh = true;


void keyboardCallback(const pcl::visualization::KeyboardEvent& event, void* cookie)
{
  const char c = event.getKeyCode();
  if( event.keyDown() ) {
    switch( c ) {
    case '.': //next frame
      dFrame = 1;
      refresh = true;
      break;
    case ',': //prev frame
      dFrame = -1;
      refresh = true;
      break;
    case '>': //next track
      dTrack = 1;
      refresh = true;
      break;
    case '<': //prev track
      dTrack = -1;
      refresh = true;
      break;
    case 't':
      std::cout <<"target track number: ";
      int t;
      std::cin >>t;
      std::cout <<"Jumping to track " <<t <<std::endl;
      dTrack = t - (int)trackid - 1;
      refresh = true;
      break;

    case ' ':
      color_type = (ColorType)(((int)color_type+1) % CT_END);
      switch(color_type) {
      case CT_COLOR:
        std::cout <<"Displaying colors (if available)." <<std::endl;
        break;
      case CT_INTENSITY:
        std::cout <<"Coloring by intensity (if available)." <<std::endl;
        break;
      case CT_NONE:
        std::cout <<"No colors." <<std::endl;
        break;
      }
      refresh = true;
      break;

    case '/':
      frame_type = (FrameType)(((int)frame_type+1) % FT_END);
      switch(frame_type) {
      case FT_BASE_LINK:
        std::cout <<"Centering view on the vehicle." <<std::endl;
        break;
      case FT_DEMEAN:
        std::cout <<"Centering view on point cloud (demean)." <<std::endl;
        break;
      }
      refresh = true;
      break;

    case 'h':
      std::cout <<std::endl <<std::endl;
      std::cout <<"Custom keys for the track visualizer:" <<std::endl;
      std::cout <<"\t  .   : next frame" <<std::endl;
      std::cout <<"\t  ,   : prev frame" <<std::endl;
      std::cout <<"\t  >   : next track" <<std::endl;
      std::cout <<"\t  <   : prev track" <<std::endl;
      std::cout <<"\t  t   : jump to track number (type in terminal)" <<std::endl;
      std::cout <<"\tSPACE : change color display" <<std::endl;
      std::cout <<"\t  /   : change reference frame" <<std::endl;
      std::cout <<std::endl;
    }
  }
}

/// Computes the new trackid and frameid
void nav()
{
  if( dTrack>0 ) {
    const unsigned newTrack = std::min((unsigned)tracks.tracks.size()-1, trackid+dTrack);
    if( newTrack>trackid ) {
      trackid = newTrack;
      frameid = 0;
    }
  }
  else if( dTrack<0 ) {
    const unsigned newTrack = std::max(0, (int)trackid+dTrack+(frameid>0?1:0));
    if( newTrack<trackid || frameid>0 ) {
      trackid = newTrack;
      frameid = 0;
    }
  }
  dTrack = 0;

  if( dFrame>0 ) {
    if( frameid+dFrame < tracks.tracks[trackid].frames.size() ) {
      frameid += dFrame;
    }
    else if( trackid == tracks.tracks.size()-1 ) {
      frameid = tracks.tracks[trackid].frames.size() - 1;
    }
    else {
      ++trackid;
      frameid = 0;
    }
  }
  else if( dFrame<0 ) {
    if( frameid >= -dFrame ) {
      frameid += dFrame;
    }
    else if( trackid==0 ) {
      frameid = 0;
    }
    else {
      --trackid;
      frameid = tracks.tracks[trackid].frames.size() - 1;
    }
  }
  dFrame = 0;
}

template <class PointT>
void center_point_cloud(pcl::PointCloud<PointT>& pcd, const geometry_msgs::Pose& robot_pose)
{
  Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
  if( frame_type == FT_DEMEAN ) {
    BOOST_FOREACH(const PointT& p, pcd.points) {
      centroid.x() += p.x;
      centroid.y() += p.y;
      centroid.z() += p.z;
    }
    centroid /= pcd.points.size();
  }
  else if( frame_type == FT_BASE_LINK ) {
    centroid.x() = robot_pose.position.x;
    centroid.y() = robot_pose.position.y;
    centroid.z() = robot_pose.position.z;
  }
  else
    ROS_BREAK();

  BOOST_FOREACH(PointT& p, pcd.points) {
    p.x -= centroid.x();
    p.y -= centroid.y();
    p.z -= centroid.z();
  }
}


int main(int argc, char **argv)
{
  if( argc<2 ) {
    std::cerr <<"Track file required." <<std::endl;
    return 1;
  }
  track_file_io::load(argv[1], tracks);

  if( tracks.tracks.empty() ) {
    std::cerr <<"No tracks in " <<argv[1] <<std::endl;
    return 1;
  }

  visualizer.reset( new pcl::visualization::PCLVisualizer("tracks visualizer") );
  visualizer->registerKeyboardCallback(&keyboardCallback);

  while( ! visualizer->wasStopped() )
  {
    nav();
    if( refresh ) {
      const track_file_io::Track& tr = tracks.tracks[trackid];
      const sensor_msgs::PointCloud2& pcd = tr.frames[frameid].cloud;
      if( true /*tr.label_=="car" && pcd.points.size()>500*/ ) {
        visualizer->removeAllPointClouds();

        bool has_color = false;
        bool has_intensity = false;
        for(unsigned i=0; i<pcd.fields.size(); ++i) {
          if( pcd.fields[i].name=="rgb" )
            has_color = true;
          else if( pcd.fields[i].name=="intensity" )
            has_intensity = true;
        }

        if( has_color && color_type==CT_COLOR ) {
          pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
          pcl::fromROSMsg(pcd, *cloud);
          center_point_cloud(*cloud, tr.frames[frameid].robot_pose);
          visualizer->addPointCloud(cloud);
        }
        else if( has_intensity && color_type==CT_INTENSITY ) {
          pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
          pcl::fromROSMsg(pcd, *cloud);
          pcl::PointCloud<pcl::PointXYZRGB>::Ptr rgbcloud(new pcl::PointCloud<pcl::PointXYZRGB>);
          rgbcloud->resize(cloud->size());
          for( unsigned i=0; i<cloud->size(); ++i) {
            const pcl::PointXYZI& ipt = (*cloud)[i];
            pcl::PointXYZRGB& rgbpt = (*rgbcloud)[i];
            rgbpt.x = ipt.x;
            rgbpt.y = ipt.y;
            rgbpt.z = ipt.z;
            rgbpt.r = rgbpt.g = rgbpt.b = ((float)ipt.intensity/256*(256-50)) + 50;
          }
          center_point_cloud(*rgbcloud, tr.frames[frameid].robot_pose);
          visualizer->addPointCloud(rgbcloud);
        }
        else {
          pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
          pcl::fromROSMsg(pcd, *cloud);
          center_point_cloud(*cloud, tr.frames[frameid].robot_pose);
          visualizer->addPointCloud(cloud);
        }

        refresh = false;
        std::cout <<"Track " <<trackid <<"/" <<tracks.tracks.size() <<", frame "
                 <<frameid <<"/" <<tr.frames.size()
                <<". label: " <<tr.label
               <<", timestamp=" <<std::setprecision(16) <<tr.frames[frameid].stamp.toSec()
              <<"." <<std::endl;
      }
      else {
        dFrame = 1;
      }
    }
    visualizer->spinOnce(3);
  }

  return 0;
}
