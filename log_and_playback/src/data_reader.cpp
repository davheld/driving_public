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

#include <iostream>

#include <boost/foreach.hpp>

#include <stdr_lib/rosparam_helpers.h>
#include <dgc_transform/dgc_transform.h>
#include <stdr_velodyne/transform.h>
#include <log_and_playback/data_reader.h>
#include <log_and_playback/bag_reader.h>
#include <log_and_playback/dgclog_reader.h>
#include <log_and_playback/kittireader.h>


namespace log_and_playback
{


bool data_reader_time_compare(const boost::shared_ptr<AbstractDataReader>& a,
                              const boost::shared_ptr<AbstractDataReader>& b)
{
  return a->time() < b->time();
}

void DataReader::load(const std::vector<std::string> & logs, ros::Duration skip)
{
  readers_.clear();
  bool dgc_logs=false, kitti_logs=false;
  std::vector<std::string> bag_logs;
  bool do_skip = false;

  BOOST_FOREACH(std::string const & path, logs)
  {
    if( boost::algorithm::ends_with(path, ".log") || boost::algorithm::ends_with(path, ".log.gz") )
    {
      boost::shared_ptr<LogDataReader> loggz_reader(new LogDataReader);
      loggz_reader->open(path);
      loggz_reader->next();
      readers_.push_back(boost::dynamic_pointer_cast<AbstractDataReader>(loggz_reader));
      dgc_logs = true;
      do_skip = true;
    }
    else if( boost::algorithm::ends_with(path, ".vlf") )
    {
      boost::shared_ptr<VlfDataReader> vlf_reader(new VlfDataReader);
      vlf_reader->open(path);
      vlf_reader->next();
      readers_.push_back(boost::dynamic_pointer_cast<AbstractDataReader>(vlf_reader));
      dgc_logs = true;
      do_skip = true;
    }
    else if( boost::algorithm::ends_with(path, ".llf") )
    {
      boost::shared_ptr<LlfDataReader> llf_reader(new LlfDataReader);
      llf_reader->open(path);
      llf_reader->next();
      readers_.push_back(boost::dynamic_pointer_cast<AbstractDataReader>(llf_reader));
      dgc_logs = true;
      do_skip = true;
    }
    else if( boost::algorithm::ends_with(path, ".bag") )
    {
      // load them all together later
      bag_logs.push_back(path);
      do_skip = false; //we open directly with the offset
    }
    else if( boost::algorithm::ends_with(path, ".kit") )
    {
      boost::shared_ptr<KittiVeloReader> reader(new KittiVeloReader);
      reader->open(path);
      reader->next();
      readers_.push_back(boost::dynamic_pointer_cast<AbstractDataReader>(reader));
      kitti_logs = true;
      do_skip = true;
    }
    else if( boost::algorithm::ends_with(path, ".imu") )
    {
      boost::shared_ptr<KittiApplanixReader> reader(new KittiApplanixReader);
      reader->open(path);
      reader->next();
      readers_.push_back(boost::dynamic_pointer_cast<AbstractDataReader>(reader));
      kitti_logs = true;
      do_skip = true;
    }
    else
    {
      ROS_INFO_STREAM("Unrecognized log file: " <<path <<". Skipping.");
    }
  }

  // check that we actually got some valid logs, and that we are dealing with
  // only one type of log files
  unsigned n_types = 0;
  if( dgc_logs ) ++n_types;
  if( !bag_logs.empty() ) ++n_types;
  if( kitti_logs ) ++n_types;

  if( n_types==0 ) {
    BOOST_THROW_EXCEPTION(stdr::ex::ExceptionBase() <<stdr::ex::MsgInfo("You must provide some log files"));
  }
  else if( n_types>1 ) {
    BOOST_THROW_EXCEPTION(stdr::ex::ExceptionBase() <<stdr::ex::MsgInfo("You cannot provide dgc logs and bags at the same time"));
  }

  if( !bag_logs.empty() ) {
    boost::shared_ptr<BagReader> bag_reader(new BagReader);
    bag_reader->load_bags(bag_logs, skip);
    readers_.push_back(boost::dynamic_pointer_cast<AbstractDataReader>(bag_reader));
    do_skip = false;
  }

  ok_ = !readers_.empty();
  BOOST_FOREACH(const boost::shared_ptr<AbstractDataReader>& reader, readers_) {
    ok_ &= reader->ok();
  }
  if( !ok_ )
    BOOST_THROW_EXCEPTION(stdr::ex::ExceptionBase() <<stdr::ex::MsgInfo("Failed to load the data"));

  std::sort(readers_.begin(), readers_.end(), data_reader_time_compare);

  time_ = readers_.front()->time();

  if( do_skip ) {
    const ros::Time start_time = time_ + skip;
    while( time_ < start_time && next() );
    if( !ok_ || time_ < start_time )
      BOOST_THROW_EXCEPTION(stdr::ex::ExceptionBase() <<stdr::ex::MsgInfo("Failed to load the data"));
  }
}

bool DataReader::next()
{
  static ros::Time last_time = ros::TIME_MIN;
  typedef std::vector< boost::shared_ptr<AbstractDataReader> > Readers;

  if( readers_.empty() ) {
    ok_ = false;
    return false;
  }

  readers_.front()->next();

  if( !readers_.front()->ok() ) {
    readers_.erase(readers_.begin());
    if( readers_.empty() ) {
      ok_ = false;
      return false;
    }
  }

  std::sort(readers_.begin(), readers_.end(), data_reader_time_compare);

  time_ = readers_.front()->time();
  if( time_ < last_time )
    ROS_WARN("negative time change");
  last_time = time_;

  ok_ = true;
  return true;
}

#define FUNC_BODY(T, fn) return readers_.empty() ? T::ConstPtr() : readers_.front()->fn()

stdr_msgs::ApplanixPose::ConstPtr DataReader::instantiateApplanixPose() const
{
  FUNC_BODY(stdr_msgs::ApplanixPose, instantiateApplanixPose);
}

stdr_msgs::ApplanixGPS::ConstPtr DataReader::instantiateApplanixGPS() const
{
  FUNC_BODY(stdr_msgs::ApplanixGPS, instantiateApplanixGPS);
}

stdr_msgs::ApplanixRMS::ConstPtr DataReader::instantiateApplanixRMS() const
{
  FUNC_BODY(stdr_msgs::ApplanixRMS, instantiateApplanixRMS);
}

velodyne_msgs::VelodyneScan::ConstPtr DataReader::instantiateVelodyneScans() const
{
  FUNC_BODY(velodyne_msgs::VelodyneScan, instantiateVelodyneScans);
}

stdr_msgs::LadybugImages::ConstPtr DataReader::instantiateLadybugImages() const
{
  FUNC_BODY(stdr_msgs::LadybugImages, instantiateLadybugImages);
}

stdr_velodyne::PointCloud::ConstPtr DataReader::instantiateVelodyneSpin() const
{
  FUNC_BODY(stdr_velodyne::PointCloud, instantiateVelodyneSpin);
}

stdr_msgs::LocalizePose::ConstPtr DataReader::instantiateLocalizePose() const
{
  FUNC_BODY(stdr_msgs::LocalizePose, instantiateLocalizePose);
}

stdr_msgs::EStopStatus::ConstPtr DataReader::instantiateEStopStatus() const
{
  FUNC_BODY(stdr_msgs::EStopStatus, instantiateEStopStatus);
}

stdr_msgs::PassatStatus::ConstPtr DataReader::instantiatePassatStatus() const
{
  FUNC_BODY(stdr_msgs::PassatStatus, instantiatePassatStatus);
}

stdr_msgs::Trajectory2D::ConstPtr DataReader::instantiateTrajectory2D() const
{
  FUNC_BODY(stdr_msgs::Trajectory2D, instantiateTrajectory2D);
}

} //namespace log_and_playback
