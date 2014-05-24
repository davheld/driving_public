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
#include <track_tools/track_manager.h>

/** This program loads a tm file (see track_tools/TrackManager) and displays
  * the data in PCLVisualizer, track by track, frame by frame.
  *
  * The data can have color or not.
  */

/* TODO:
 * - Add a filter CLI options: only certain class, min number of points, etc.
 */

boost::shared_ptr<pcl::visualization::PCLVisualizer> visualizer;
track_manager::TrackManager tmanager;

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
    const unsigned newTrack = std::min((unsigned)tmanager.tracks_.size()-1, trackid+dTrack);
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
    if( frameid+dFrame < tmanager.tracks_[trackid]->frames_.size() ) {
      frameid += dFrame;
    }
    else if( trackid == tmanager.tracks_.size()-1 ) {
      frameid = tmanager.tracks_[trackid]->frames_.size() - 1;
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
      frameid = tmanager.tracks_[trackid]->frames_.size() - 1;
    }
  }
  dFrame = 0;
}

void toPCL(const sensor_msgs::PointCloud& pcd, pcl::PointCloud<pcl::PointXYZRGBA>& pts)
{
  pts.points.resize( pcd.points.size() );

  const std::vector<float>* colors = 0;
  const std::vector<float>* intensities = 0;
  for(unsigned i=0; i<pcd.channels.size(); ++i) {
    if( pcd.channels[i].name=="rgb" )
      colors = &(pcd.channels[i].values);
    else if( pcd.channels[i].name=="" || pcd.channels[i].name=="intensity" )
      intensities = &(pcd.channels[i].values);
  }

  Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
  if( frame_type == FT_DEMEAN ) {
    BOOST_FOREACH(const geometry_msgs::Point32& p, pcd.points) {
      centroid.x() += p.x;
      centroid.y() += p.y;
      centroid.z() += p.z;
    }
    centroid /= pcd.points.size();
  }
  else if( frame_type == FT_BASE_LINK ) {
    const dgc_transform::dgc_pose_t& robo_pose = tmanager.tracks_[trackid]->frames_[frameid]->robot_pose();
    centroid.x() = robo_pose.x;
    centroid.y() = robo_pose.y;
    centroid.z() = robo_pose.z;
  }
  else
    ROS_BREAK();

  for(unsigned i=0; i<pcd.points.size(); ++i) {
    pts.points[i].x = pcd.points[i].x - centroid.x();
    pts.points[i].y = pcd.points[i].y - centroid.y();
    pts.points[i].z = pcd.points[i].z - centroid.z();
    pts.points[i].a = 255;
    if( color_type==CT_COLOR && colors ) {
      pts.points[i].rgb = colors->at(i);
      //std::cout <<"color: " <<(int)pts.points[i].r <<", " <<(int)pts.points[i].g <<", " <<(int)pts.points[i].b <<std::endl;
    }
    else if( color_type!=CT_NONE && intensities ) {
      const unsigned char I = intensities->at(i);
      pts.points[i].r = I;
      pts.points[i].g = I;
      pts.points[i].b = I;
    }
    else {
      pts.points[i].r = 255;
      pts.points[i].g = 255;
      pts.points[i].b = 255;
    }
  }

}

int main(int argc, char **argv)
{
  if( argc<2 ) {
    std::cerr <<"Track file required." <<std::endl;
    return 1;
  }
  if( !tmanager.load(argv[1]) ) {
    std::cerr <<"Could not load " <<argv[1] <<std::endl;
    return 1;
  }
  if( tmanager.tracks_.empty() ) {
    std::cerr <<"No tracks in " <<argv[1] <<std::endl;
    return 1;
  }

  visualizer.reset( new pcl::visualization::PCLVisualizer("tracks visualizer") );
  visualizer->registerKeyboardCallback(&keyboardCallback);
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pts = boost::make_shared< pcl::PointCloud<pcl::PointXYZRGBA> >();

  while( ! visualizer->wasStopped() )
  {
    nav();
    if( refresh ) {
      const track_manager::Track& tr = *(tmanager.tracks_[trackid]);
      const sensor_msgs::PointCloud& pcd = tr.frames_[frameid]->cloud();
      if( true /*tr.label_=="car" && pcd.points.size()>500*/ ) {
        visualizer->removeAllPointClouds();
        toPCL(pcd, *pts);
        visualizer->addPointCloud(pts);
        refresh = false;
        std::cout <<"Track " <<trackid <<"/" <<tmanager.tracks_.size() <<", frame "
                 <<frameid <<"/" <<tr.frames_.size()
                <<". label: " <<tr.label_
               <<", timestamp=" <<std::setprecision(16) <<tr.frames_[frameid]->timestamp()
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
