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


/** This program loads a trk file (see track_file_io) and displays
  * the data in PCLVisualizer.
  * Either track by track, frame by frame; or all the tracks for a timestamp
  * at once, colored by track id, with id indicated. This allows to see the
  * context.
  *
  * When displayed track by track, the data can have color information,
  * intensity information, or none.
  */

/* TODO:
 * - Add a filter CLI options: only certain class, min number of points, etc.
 */


#include <map>
#include <vector>
#include <algorithm>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_cloud_color_handlers.h>
#include <pcl_conversions/pcl_conversions.h>

#include <stdr_lib/heat_map.h>
#include <track_file_io/track_file_io.h>
#include <track_file_io/manipulations.h>


// The visualizer. Will be constructed later.
boost::shared_ptr<pcl::visualization::PCLVisualizer> visualizer;

// The tracks we are working with
track_file_io::Tracks tracks;

// for navigation
// The keyboard event handler sets those to a non zero value when the navigation
// keys are pressed, but loading the new frame happens in the main loop
int dTrack=0, dFrame=0;
unsigned trackid=0, frameid=0;

// the display mode: either track by track, or the whole frame at once
enum DisplayModeType { DMT_TRACK, DMT_WHOLE, DMT_END };
DisplayModeType display_mode = DMT_TRACK;

// how to color the tracks when displayed track by track
enum ColorType { CT_COLOR, CT_INTENSITY, CT_NONE, CT_END };
ColorType color_type = CT_COLOR;

// where to center the frame, when displayed track by track
enum FrameType { FT_BASE_LINK, FT_DEMEAN, FT_END };
FrameType frame_type = FT_DEMEAN;

// in whole frame display mode, whether to show the track ids (it slows down the display)
bool show_track_ids = false;

// redraw needed
bool refresh = true;


// this text appears on stdout and on the visualizer
std::string action_feedback_msg;

void actionFeedbackMsg(const std::string &msg)
{
  std::cout <<msg <<std::endl;
  action_feedback_msg = msg;
}

void actionFeedbackMsgBool(std::string msg, bool state)
{
  if( state )
    msg = std::string("Enabled ") + msg;
  else
    msg = std::string("Disabled ") + msg;
  actionFeedbackMsg(msg);
}



// these are pointers to individual track frames, when displaying the whole frame
class Frame
{
  int trackid_;
  const geometry_msgs::Pose *robot_pose_;
  const sensor_msgs::PointCloud2 *pcd_;

public:
  Frame(const track_file_io::Track &tr, const track_file_io::Frame &f)
    : trackid_(tr.id)
    , robot_pose_(&f.robot_pose)
    , pcd_(&f.cloud)
  { }

  inline const sensor_msgs::PointCloud2 & pcd() const { return *pcd_; }
  inline const geometry_msgs::Pose & robotPose() const { return *robot_pose_; }
  inline int trackid() const { return trackid_; }
};

// we organise them in a map, for easy access by timestamp
typedef std::map<double, std::vector<Frame> > WholeFramesMap;
WholeFramesMap whole_frames_map;
WholeFramesMap::const_iterator whole_frames_it;



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
        actionFeedbackMsg("Displaying colors (if available)");
        break;
      case CT_INTENSITY:
        actionFeedbackMsg("Coloring by intensity (if available)");
        break;
      case CT_NONE:
        actionFeedbackMsg("No colors");
        break;
      }
      refresh = true;
      break;

    case '/':
      frame_type = (FrameType)(((int)frame_type+1) % FT_END);
      switch(frame_type) {
      case FT_BASE_LINK:
        actionFeedbackMsg("Centering view on the vehicle");
        break;
      case FT_DEMEAN:
        actionFeedbackMsg("Centering view on point cloud (demean)");
        break;
      }
      refresh = true;
      break;

    case 'v':
      display_mode = (DisplayModeType)(((int)display_mode+1) % DMT_END);
      switch(display_mode) {
      case DMT_TRACK:
        actionFeedbackMsg("Switching to displaying track by track");
        break;
      case DMT_WHOLE:
        actionFeedbackMsg("Switching to displaying the whole set of tracks");
        break;
      }
      refresh = true;
      break;

    case 'k':
      if( display_mode==DMT_WHOLE ) {
        show_track_ids = !show_track_ids;
        actionFeedbackMsgBool("showing track ids", show_track_ids);
        refresh = true;
      }
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
      std::cout <<"\t  v   : change display mode" <<std::endl;
      std::cout <<"\t  k   : toggle showing track ids (slows down the display when on)" <<std::endl;
      std::cout <<std::endl;
    }
  }
}

/// Computes the new trackid and frameid
void nav()
{
  if( display_mode==DMT_TRACK ) {
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
  else if( display_mode==DMT_WHOLE ) {
    if( dTrack!=0 ) {
      dFrame = dTrack * 10;
    }
    while( dFrame>0 ) {
      WholeFramesMap::const_iterator it = whole_frames_it;
      ++it;
      if( it!=whole_frames_map.end() )
        whole_frames_it = it;
      else
        break;
      --dFrame;
    }
    while( dFrame<0 && whole_frames_it!=whole_frames_map.begin() ) {
      --whole_frames_it;
      ++dFrame;
    }
    dFrame = dTrack = 0;
  }
  else {
    ROS_BREAK();
  }
}

template <class PointT>
void centerPointCloud(pcl::PointCloud<PointT>& pcd, const geometry_msgs::Pose& robot_pose, FrameType ft)
{
  Eigen::Vector3d centroid = Eigen::Vector3d::Zero();
  if( ft == FT_DEMEAN ) {
    BOOST_FOREACH(const PointT& p, pcd.points) {
      centroid.x() += p.x;
      centroid.y() += p.y;
      centroid.z() += p.z;
    }
    centroid /= pcd.points.size();
  }
  else if( ft == FT_BASE_LINK ) {
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


void viewIndividualTrack()
{
  const track_file_io::Track &tr = tracks.tracks[trackid];
  const sensor_msgs::PointCloud2 &pcd = tr.frames[frameid].cloud;

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
    centerPointCloud(*cloud, tr.frames[frameid].robot_pose, frame_type);
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
    centerPointCloud(*rgbcloud, tr.frames[frameid].robot_pose, frame_type);
    visualizer->addPointCloud(rgbcloud);
  }
  else {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(pcd, *cloud);
    centerPointCloud(*cloud, tr.frames[frameid].robot_pose, frame_type);
    visualizer->addPointCloud(cloud);
  }

  std::stringstream ss;
  ss <<"Track " <<trackid <<"/" <<tracks.tracks.size() <<", frame " <<frameid
    <<"/" <<tr.frames.size() <<". label: " <<tr.label <<", timestamp="
   <<std::setprecision(16) <<tr.frames[frameid].stamp.toSec();
  actionFeedbackMsg(ss.str());
}

static const stdr::ColorWheel rgbs(60);

void viewWholeFrame()
{
  BOOST_FOREACH(const Frame &frame, whole_frames_it->second)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(frame.pcd(), *cloud);
    centerPointCloud(*cloud, frame.robotPose(), FT_BASE_LINK);

    const stdr::ColorWheel::RGB &rgb = rgbs[frame.trackid()];
    typedef pcl::visualization::PointCloudColorHandler<pcl::PointXYZ> CH;
    typedef pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> CHP;
    CHP color_h(cloud, rgb.r, rgb.g, rgb.b);

    std::stringstream ss;
    ss <<frame.trackid();
    visualizer->addPointCloud(cloud, color_h, std::string("track")+ss.str());

    if( show_track_ids ) {
      visualizer->addText3D(ss.str(), cloud->points[0], 1,
                            ((float)rgb.r)/255, ((float)rgb.g/255), ((float)rgb.b/255),
                            std::string("id")+ss.str());
    }
  }

  visualizer->addCoordinateSystem(1);

  std::stringstream ss;
  ss <<"Timestamp=" <<std::setprecision(16) <<whole_frames_it->first;
  ss <<", frame number=" <<std::distance<WholeFramesMap::const_iterator>(whole_frames_map.begin(), whole_frames_it);
  visualizer->addText(ss.str(), 0, 30, 15, 1, 1, 1, "stamp_msg");
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


  BOOST_FOREACH(const track_file_io::Track& tr, tracks.tracks) {
    BOOST_FOREACH(const track_file_io::Frame& f, tr.frames) {
      whole_frames_map[f.stamp.toSec()].push_back( Frame(tr,f) );
    }
  }
  whole_frames_it = whole_frames_map.begin();


  visualizer.reset( new pcl::visualization::PCLVisualizer("tracks visualizer") );
  visualizer->registerKeyboardCallback(&keyboardCallback);

  while( ! visualizer->wasStopped() )
  {
    nav();

    if( refresh ) {
      visualizer->removeAllPointClouds();
      visualizer->removeAllShapes();
      visualizer->removeCoordinateSystem();

      if( display_mode==DMT_TRACK )
        viewIndividualTrack();
      else if( display_mode==DMT_WHOLE )
        viewWholeFrame();

      refresh = false;
      if( !action_feedback_msg.empty() )
        visualizer->addText(action_feedback_msg, 80, 0, 15, 1, 1, 1, "action_feedback_msg");
    }
    visualizer->spinOnce(3);
  }

  return 0;
}
