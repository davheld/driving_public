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
#include <ros/console.h>
#include <stdr_msgs/RawScans.h>
#include <stdr_velodyne/conversion.h>
#include <log_and_playback/bag_reader.h>


namespace log_and_playback
{

BagReader::BagReader()
{

}

BagReader::~BagReader()
{
  BOOST_FOREACH( boost::shared_ptr<rosbag::Bag> &bag, bags_ ) {
    bag->close();
  }
}

void BagReader::load_bag(const std::string& bagpath, ros::Duration skip)
{
  std::vector<std::string> bagpaths;
  bagpaths.push_back(bagpath);
  load_bags(bagpaths, skip);
}

void BagReader::load_bags(const std::vector<std::string> & bagpaths, ros::Duration skip)
{
  BOOST_FOREACH( boost::shared_ptr<rosbag::Bag> &bag, bags_ )
    bag->close();
  bags_.clear();

  view_.reset( new rosbag::View );
  std::vector<std::string> topics;
  topics.push_back(std::string("/driving/velodyne/raw_scans"));
  topics.push_back(std::string("/driving/velodyne/packets"));
  topics.push_back(std::string("/driving/ApplanixPose"));
  topics.push_back(std::string("/driving/ApplanixGPS"));
  topics.push_back(std::string("/driving/ApplanixRMS"));
  topics.push_back(std::string("/driving/ladybug/images"));
  topics.push_back(std::string("/driving/LocalizePose"));

  BOOST_FOREACH(std::string const& bagpath, bagpaths) {
    ROS_DEBUG_STREAM("Loading data from " <<bagpath);
    boost::shared_ptr<rosbag::Bag> bag(new rosbag::Bag);
    bag->open(bagpath);
    rosbag::View view(*bag);
    if( view.getBeginTime() > view.getEndTime() ) {
      //bag is empty
      continue;
    }
    const ros::Time start_time = view.getBeginTime() + skip;
    bags_.push_back(bag);
    view_->addQuery(*bag, rosbag::TopicQuery(topics), start_time);
  }
  const double duration = (view_->getEndTime()-view_->getBeginTime()).toSec();
  ROS_INFO_STREAM("Duration: " <<duration);
  bag_it_ = view_->begin();
}

bool BagReader::next()
{
  if( ok() ) ++bag_it_;
  return ok();
}

bool BagReader::ok() const
{
  return bag_it_!=view_->end();
}

velodyne_msgs::VelodyneScan::ConstPtr BagReader::instantiateVelodyneScans() const
{
  velodyne_msgs::VelodyneScan::ConstPtr velodyne_scan = bag_it_->instantiate<velodyne_msgs::VelodyneScan>();
  if( velodyne_scan )
    return velodyne_scan;

  stdr_msgs::RawScans::ConstPtr raw_scans = bag_it_->instantiate<stdr_msgs::RawScans>();
  if( !raw_scans )
    return velodyne_scan;

  velodyne_msgs::VelodyneScan::Ptr scan = boost::make_shared<velodyne_msgs::VelodyneScan>();
  stdr_velodyne::convert(*raw_scans, *scan);
  return scan;
}


} //namespace

