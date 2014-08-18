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

#include <dgc_transform/dgc_transform.h>
#include <stdr_velodyne/transform.h>
#include <log_and_playback/data_reader.h>
#include <iostream>

namespace bpo=boost::program_options;

namespace log_and_playback
{


bool data_reader_time_compare(const boost::shared_ptr<AbstractDataReader>& a,
                              const boost::shared_ptr<AbstractDataReader>& b)
{
  return a->time() < b->time();
}

void DataReader::load(const std::vector<std::string> & logs, ros::Duration skip)
{
  ok_ = false;
  readers_.clear();
  bool dgc_logs=false, kitti_logs=false;
  std::vector<std::string> bag_logs;

  BOOST_FOREACH(std::string const & path, logs)
  {
    if( (path.length()>4 && path.substr(path.length()-4).compare(".log")==0) ||
        (path.length()>7 && path.substr(path.length()-7).compare(".log.gz")==0) )
    {
      boost::shared_ptr<LogDataReader> loggz_reader(new LogDataReader);
      loggz_reader->open(path);
      loggz_reader->next();
      readers_.push_back(boost::dynamic_pointer_cast<AbstractDataReader>(loggz_reader));
      ok_ = true;
      dgc_logs = true;
    }
    else if( path.length()>4 && path.substr(path.length()-4).compare(".vlf")==0 )
    {
      boost::shared_ptr<VlfDataReader> vlf_reader(new VlfDataReader);
      vlf_reader->open(path);
      vlf_reader->next();
      readers_.push_back(boost::dynamic_pointer_cast<AbstractDataReader>(vlf_reader));
      ok_ = true;
      dgc_logs = true;
    }
    else if( path.length()>4 && path.substr(path.length()-4).compare(".llf")==0 )
    {
      boost::shared_ptr<LlfDataReader> llf_reader(new LlfDataReader);
      llf_reader->open(path);
      llf_reader->next();
      readers_.push_back(boost::dynamic_pointer_cast<AbstractDataReader>(llf_reader));
      ok_ = true;
      dgc_logs = true;
    }
    else if( path.length()>4 && path.substr(path.length()-4).compare(".bag")==0 )
    {
      // load them all together later
      bag_logs.push_back(path);
    }
    else if( path.length()>4 && path.substr(path.length()-4).compare(".kit")==0 )
    {
      boost::shared_ptr<KittiVeloReader> reader(new KittiVeloReader);
      reader->open(path);
      reader->next();
      readers_.push_back(boost::dynamic_pointer_cast<AbstractDataReader>(reader));
      ok_ = true;
      kitti_logs = true;
    }
    else if( path.length()>4 && path.substr(path.length()-4).compare(".imu")==0 )
    {
      boost::shared_ptr<KittiApplanixReader> reader(new KittiApplanixReader);
      reader->open(path);
      reader->next();
      readers_.push_back(boost::dynamic_pointer_cast<AbstractDataReader>(reader));
      ok_ = true;
      kitti_logs = true;
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
    ok_ = true;
  }

  std::sort(readers_.begin(), readers_.end(), data_reader_time_compare);

  time_ = readers_.front()->time();
  const ros::Time start_time = time_ + skip;
  while( time_ < start_time && next() );
}

bool DataReader::next()
{
  static ros::Time last_time=ros::TIME_MIN;
  typedef std::vector< boost::shared_ptr<AbstractDataReader> > Readers;
  for( Readers::iterator it = readers_.begin(); it!=readers_.end(); ) {
    if( ! (*it)->ok() )
      it = readers_.erase(it);
    else
      ++it;
  }

  if( readers_.empty() ) {
    ok_ = false;
    return false;
  }

  std::sort(readers_.begin(), readers_.end(), data_reader_time_compare);
  readers_.front()->next();
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


void BagTFListener::addApplanixPose(const stdr_msgs::ApplanixPose & pose)
{
  app_trans_.update(pose);
  app_trans_.addToTransformer(*this, "bag");
  if( broadcast_ )
    app_trans_.broadcast(broadcaster_);

  fake_localizer_.update(pose);
  fake_localizer_.addToTransformer(*this, "bag");
  if( broadcast_ )
    fake_localizer_.broadcast(broadcaster_);

  handleStaticTransforms(pose.header.stamp);
}

void BagTFListener::addTFMsg(const tf::tfMessage & msg)
{
  static tf::StampedTransform trans;
  for( unsigned i = 0; i < msg.transforms.size(); i++ ) {
    tf::transformStampedMsgToTF(msg.transforms[i], trans);
    setTransform(trans, "bag");
    if( broadcast_ )
      broadcaster_.sendTransform(trans);
  }

  if( ! msg.transforms.empty() )
    handleStaticTransforms(msg.transforms.rbegin()->header.stamp);
}

void BagTFListener::handleStaticTransforms(const ros::Time & stamp)
{
  BOOST_FOREACH(tf::StampedTransform t, static_transforms_) {
    t.stamp_ = stamp;
    setTransform(t, "bag");
    if( broadcast_ ) broadcaster_.sendTransform(t);
  }
}

void BagTFListener::addStaticTransform(const tf::StampedTransform & t)
{
  static_transforms_.push_back(t);
}


void SpinReader::addOptions(boost::program_options::options_description& opts_desc)
{
  opts_desc.add_options()
      ("start,s", bpo::value<double>()->default_value(0), "start SEC seconds into the bag files")
      ("nocal", "do not load any calibration file (i.e. use the default config).")
      ("cal", bpo::value<std::string>(), "velodyne calibration file")
      ("noical", "do not load any intensity calibration file (i.e. use the default config).")
      ("ical", bpo::value<std::string>(), "velodyne intensity calibration file")
      ("vtfm", bpo::value<std::string>(), "velodyne TFM file")
      ("logs", bpo::value< std::vector<std::string> >()->required(), "log files to load (bags or dgc logs)")
      ;
}

void SpinReader::addOptions(boost::program_options::positional_options_description& pos_opts_desc)
{
  pos_opts_desc.add("logs", -1);
}



SpinReader::SpinReader()
: do_I_own_the_data_reader_(false), data_reader_(0)
, config_( stdr_velodyne::Configuration::getStaticConfigurationInstance() )
{

}

SpinReader::~SpinReader()
{
  unsetDataReader();
}

void SpinReader::setDataReader(AbstractDataReader & reader)
{
  unsetDataReader();
  data_reader_ = &reader;
  do_I_own_the_data_reader_ = false;
}

void SpinReader::unsetDataReader()
{
  if( do_I_own_the_data_reader_ && data_reader_ )
    delete data_reader_;
}

void SpinReader::load(const std::vector< std::string > &logs, ros::Duration skip)
{
  unsetDataReader();
  data_reader_ = new DataReader();
  do_I_own_the_data_reader_ = true;
  ((DataReader *)data_reader_)->load(logs, skip);
}

stdr_velodyne::PointCloudConstPtr SpinReader::getSpin() const
{
  return current_spin_;
}

stdr_velodyne::PointCloudPtr SpinReader::processSpinQueue()
{
  static const std::string target_frame = "smooth";
  stdr_velodyne::PointCloudPtr spin;
  stdr_velodyne::PointCloud velo_smooth;
  bool can_transform = true;

  while( ! spinQ_.empty() && ! spin && can_transform ) {

    ROS_DEBUG_STREAM(spinQ_.size() <<" spins in Q.");
    // NEW KITTI CHANGE NOT Robust

    //can_transform = tf_listener_.canTransform(target_frame, spinQ_.front()->header.frame_id, ros::Time(spinQ_.front()->header.stamp*1E-6));
    can_transform = tf_listener_.canTransform(target_frame, spinQ_.front()->header.frame_id, ros::Time(spinQ_.front()->header.stamp*1E-9));
    if( !can_transform )
      break;

    try {
      stdr_velodyne::transform_scan(tf_listener_, target_frame, *spinQ_.front(), velo_smooth);
      spinQ_.pop();
      spin = spin_collector_.add(velo_smooth);
      if( spin ) {
        ROS_DEBUG("got whole spin");
        current_spin_ = spin;
      }
    }
    catch (tf::TransformException & e) {
      ROS_DEBUG_STREAM("transform_scan failed: " <<e.what());
      can_transform = false;
    }

  }

  // limit the size of the queue. Keep the last 200ms of data
  while( !spinQ_.empty() && (spinQ_.back()->header.stamp - spinQ_.front()->header.stamp)>200000 )
    spinQ_.pop();

  return spin;
}

bool SpinReader::prevSpin()
{
  ROS_ERROR("prevSpin not yet implemented.");
  return false;
}

bool SpinReader::nextSpin()
{
  stdr_msgs::ApplanixPose::ConstPtr applanix;
  stdr_velodyne::PointCloud::ConstPtr pcd;
  velodyne_msgs::VelodyneScan::ConstPtr scans;

  stdr_velodyne::PointCloudPtr spin = processSpinQueue();
  while( !spin && ros::ok() )
  {
    if( !data_reader_->next() )
      return false;

    if( applanix = data_reader_->instantiateApplanixPose() ) {
      ROS_DEBUG("Adding applanix pose t=%.3f", applanix->header.stamp.toSec());
      tf_listener_.addApplanixPose(*applanix);
    }
    else if( pcd = data_reader_->instantiateVelodyneSpin() ) {
      spinQ_.push(pcd);
    }
    else if( scans = data_reader_->instantiateVelodyneScans() ) {
      ROS_DEBUG("got raw scan t=%.3f", scans->header.stamp.toSec());
      BOOST_FOREACH(const velodyne_msgs::VelodynePacket & pkt, scans->packets) {
        stdr_velodyne::PointCloud::Ptr pcd_ = boost::make_shared<stdr_velodyne::PointCloud>();
        packet2pcd_convertor_.processPacket(pkt, *pcd_);
        std_msgs::Header h(scans->header);
        h.stamp = pkt.stamp;
        pcl_conversions::toPCL(h, pcd_->header);
        spinQ_.push(pcd_);
      }
    }

    spin = processSpinQueue();
  }

  return spin;
}

void SpinReader::loadCalibrationFromProgramOptions(const bpo::variables_map & vm)
{
  if( ! vm.count("nocal") ) {
    std::string calibration_file;
    if( vm.count("cal") )
      calibration_file = vm["cal"].as<std::string>();
    else if( ! ros::param::get("/driving/velodyne/cal_file", calibration_file) )
      BOOST_THROW_EXCEPTION(stdr::ex::ExceptionBase() <<stdr::ex::MsgInfo(
                              "You must provide a configuration file, either on the command line, or as a rosparam."));
    config_->readCalibration(calibration_file);
  }

  if( ! vm.count("noical") ) {
    std::string intensity_file;
    if( vm.count("ical") )
      intensity_file = vm["ical"].as<std::string>();
    else if( ! ros::param::get("/driving/velodyne/int_file", intensity_file) )
      BOOST_THROW_EXCEPTION(stdr::ex::ExceptionBase() <<stdr::ex::MsgInfo(
                              "You must provide an intensity configuration file, either on the command line, or as a rosparam."));
    config_->readIntensity(intensity_file);
  }
}

void SpinReader::loadTFMFromProgramOptions(const bpo::variables_map & vm)
{
  std::string tfm_file;
  if( vm.count("vtfm") )
    tfm_file = vm["vtfm"].as<std::string>();
  else if( ! ros::param::get("/driving/velodyne/ext_file", tfm_file) )
    BOOST_THROW_EXCEPTION(stdr::ex::ExceptionBase() <<stdr::ex::MsgInfo(
                            "You must provide a TFM file, either on the command line, or as a rosparam."));
  tf::Transform tr = dgc_transform::read(tfm_file.c_str());
  tf::StampedTransform stamped_tr(tr, ros::Time(0), "base_link", "velodyne");
  tf_listener_.addStaticTransform(stamped_tr);
}


} //namespace log_and_playback
