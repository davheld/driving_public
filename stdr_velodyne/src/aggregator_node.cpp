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

/** \brief Aggregates and/or downsamples the velodyne cloud for better visualization.
  *
  * The velodyne pointcloud_node by defaults publishes very small scans, very
  * frequently. Rviz can't cope with it at the moment (see rviz issue #689).
  * Also, there is no option in rviz to downsample.
  * With this node, one can create a special topic used to better visualize the
  * velodyne data.
  */

#include <string>
#include <vector>
#include <algorithm>

#include <boost/thread/mutex.hpp>

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>

#include <timer/timer.h>
#include <stdr_velodyne/config.h>
#include <stdr_velodyne/point_type.h>
#include <stdr_velodyne/transform.h>
#include <stdr_velodyne/AggregatorConfig.h>


//-----------------------------------------------------------------------------
// Parameters (controlled by dynamic_reconfigure)

/// Target frame (best if it's a global frame). If empty (default) then do not transform.
std::string target_frame_ = "";

/// The output spin will contain points covering this angular range (0 to 1).
/// Defaults to 1.
double spin_fraction_ = 1;

/// If spin_fraction is 1, then start the spin here. Defaults to the value
/// returned by stdr_velodyne::getSpinStart()
int encoder_start_ = -1;

/// How much points to keep from the input scan: keep keep_ratio*N points
/// where N is the number of points in the input scan. Defaults to 1 (keep all).
double keep_ratio_ = 1;


// -----------------------------------------------------------------------------
// Global variables

boost::shared_ptr<tf::TransformListener> tfl;
boost::shared_ptr<ros::Subscriber> sub;
ros::Publisher publisher;
stdr_velodyne::PointCloud ptbuf;
stdr_velodyne::PointCloud fullpcd;
boost::mutex fullpcd_mut;
boost::condition_variable fullpcd_cond;
int trig_encoder;


void decimateTransformAndPublish()
{
  while( ros::ok() )
  {
    stdr_velodyne::PointCloudPtr final_pcd( new stdr_velodyne::PointCloud );
    {
      boost::unique_lock<boost::mutex> lock(fullpcd_mut);
      while( fullpcd.empty() && ros::ok() )
        fullpcd_cond.wait(lock);
      if( !ros::ok() )
        return;

      //ROS_DEBUG("Received a full spin: delay=%fms, %d points.",
      //          (ros::Time::now()-pcl_conversions::fromPCL(fullpcd.header).stamp).toSec()*1000,
      //          fullpcd.size());

      final_pcd->header = fullpcd.header;

      // decimate
      if( keep_ratio_==1 ) {
        final_pcd->points.swap( fullpcd.points );
      }
      else {
        const unsigned keep_nb = keep_ratio_ * fullpcd.points.size();
        std::vector<bool> keeps(fullpcd.points.size(), false);
        std::fill(keeps.begin(), keeps.begin()+keep_nb, true);
        std::random_shuffle(keeps.begin(), keeps.end());
        final_pcd->points.reserve(keep_nb);
        for(unsigned i=0; i<fullpcd.points.size(); ++i)
          if( keeps[i] )
            final_pcd->points.push_back(fullpcd.points[i]);
      }

      fullpcd.clear();
    }

    // transform to target frame if target_frame_ is not the empty string
    if( target_frame_.empty() ) {
      publisher.publish(final_pcd);
    }
    else {
      try {
        tfl->waitForTransform(target_frame_, final_pcd->header.frame_id, ros::Time(final_pcd->points.back().timestamp), ros::Duration(.1));
        if( tfl->canTransform(target_frame_, final_pcd->header.frame_id, ros::Time(final_pcd->points.back().timestamp)) ) {
          stdr_velodyne::transform_scan_in_place(*tfl, target_frame_, *final_pcd);
          publisher.publish(final_pcd);
        }
      } catch( tf::TransformException& e ) {
        ROS_WARN_STREAM(e.what());
      }
    }

    ROS_DEBUG("Published aggregated spin: delay=%fms",
              (ros::Time::now()-pcl_conversions::fromPCL(final_pcd->header).stamp).toSec()*1000);

  }
}


bool isFirstPoint(const stdr_velodyne::PointType& p)
{
  if( ptbuf.empty() || p.beam_id!=0 )
    return false;

  const int prev_encoder = ptbuf.back().encoder;

  // crossed the encoder_start_ line, when it's far away from zero
  if( prev_encoder < trig_encoder && p.encoder >= trig_encoder )
    return true;

  // near the zero
  if( prev_encoder > p.encoder && prev_encoder < trig_encoder + stdr_velodyne::NUM_TICKS
      && p.encoder >= trig_encoder )
    return true;

  return false;
}


void velCallBack(const stdr_velodyne::PointCloudConstPtr& inpcd)
{
  if( encoder_start_==-1 )
    encoder_start_ = inpcd->points.front().encoder;

  //ROS_DEBUG("Received a partial spin: stamp=%f, delay=%fms, %d points.",
  //          pcl_conversions::fromPCL(inpcd->header).stamp.toSec(),
  //          (ros::Time::now()-pcl_conversions::fromPCL(inpcd->header).stamp).toSec() * 1000,
  //          inpcd->size());

  // NOTE: I do not decimate before inserting as it could mess up the isFirstPoint
  // condition...

  for( stdr_velodyne::PointCloud::const_iterator it=inpcd->begin(); it!=inpcd->end(); ++it ) {
    if( isFirstPoint(*it) ) {

      {
        boost::lock_guard<boost::mutex> lock(fullpcd_mut);
        fullpcd.points.swap( ptbuf.points );
        fullpcd.header = inpcd->header;
        fullpcd.header.stamp = fullpcd.back().timestamp * 1e6;

        //ROS_DEBUG("Created a full spin: stamp=%f, delay=%fms, %d points.",
        //          pcl_conversions::fromPCL(fullpcd.header).stamp.toSec(),
        //          (ros::Time::now()-pcl_conversions::fromPCL(fullpcd.header).stamp).toSec()*1000,
        //          fullpcd.size());
      }

      fullpcd_cond.notify_one();

      trig_encoder = encoder_start_;
      if( spin_fraction_!=1 ) {
        trig_encoder = it->encoder + spin_fraction_ * stdr_velodyne::NUM_TICKS;
        if(trig_encoder>=stdr_velodyne::NUM_TICKS)
          trig_encoder -= stdr_velodyne::NUM_TICKS;
      }
    }
    ptbuf.push_back(*it);
  }
}

void cfgCallback(stdr_velodyne::AggregatorConfig &config, uint32_t level)
{
  boost::unique_lock<boost::mutex> lock(fullpcd_mut);
  target_frame_ = config.target_frame;
  encoder_start_ = config.encoder_start;
  keep_ratio_ = config.keep_ratio;
  spin_fraction_ = config.spin_fraction;
}


void connectCb()
{
  ros::NodeHandle nh("~");
  sub.reset( new ros::Subscriber(nh.subscribe("points", 10000, &velCallBack)) );
}

void disconnectCb()
{
  sub->shutdown();
  sub.reset();
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "velodyne_aggregator");
  ros::NodeHandle nh("~");

  dynamic_reconfigure::Server<stdr_velodyne::AggregatorConfig> server;
  server.setCallback( boost::bind(&cfgCallback, _1, _2) );

  tfl.reset( new tf::TransformListener );
  boost::thread procThread(decimateTransformAndPublish);
  publisher = nh.advertise<stdr_velodyne::PointCloud>("output", 10, boost::bind(&connectCb), boost::bind(&disconnectCb));

  ros::spin();

  fullpcd_cond.notify_one();
  procThread.join();

  return 0;
}
