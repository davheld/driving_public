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

#ifndef __LOG_AND_PLAYBACK__DGCLOG_READER__H__
#define __LOG_AND_PLAYBACK__DGCLOG_READER__H__

#include <string>
#include <iostream>
#include <fstream>
#include <boost/iostreams/filtering_stream.hpp>
#include <boost/iostreams/filter/gzip.hpp>

#include <blf/llf.h>

#include <log_and_playback/abstract_data_reader.h>

namespace log_and_playback
{

class LogDataReader : public AbstractDataReader
{
public:
  LogDataReader() : ok_(false) { }
  ~LogDataReader();
  void open(const std::string & filename);
  void close();

  /// Advances one data into the file. Returns false on EOF.
  bool next();
  bool ok() const { return ok_; }
  ros::Time time() const { return time_; }

  stdr_msgs::ApplanixPose::ConstPtr instantiateApplanixPose() const;
  stdr_msgs::ApplanixGPS::ConstPtr instantiateApplanixGPS() const;
  stdr_msgs::ApplanixRMS::ConstPtr instantiateApplanixRMS() const;

private:
  std::ifstream file_;
  boost::iostreams::filtering_istream stream_;
  std::string line_;
  bool ok_;

  ros::Time time_;
  stdr_msgs::ApplanixPose::ConstPtr pose_;
  stdr_msgs::ApplanixGPS::ConstPtr gps_;
  stdr_msgs::ApplanixRMS::ConstPtr rms_;
};


class VlfDataReader : public AbstractDataReader
{
public:
  VlfDataReader() : ok_(false) { }
  ~VlfDataReader();
  void open(const std::string & filename);
  void close();

  /// Advances one data into the file. Returns false on EOF.
  bool next();
  bool ok() const { return ok_; }

  ros::Time time() const { return time_; }

  velodyne_msgs::VelodyneScan::ConstPtr instantiateVelodyneScans() const { return scan_; }

private:
  std::ifstream vfile_;
  velodyne_msgs::VelodyneScan::Ptr scan_;
  ros::Time time_;
  bool ok_;
};


class LlfDataReader : public AbstractDataReader
{
public:
  LlfDataReader() : ok_(false) { }
  ~LlfDataReader();
  void open(const std::string & filename);
  void close();

  /// Advances one data into the file. Returns false on EOF.
  bool next();
  bool ok() const { return ok_; }

  ros::Time time() const { return time_; }

  stdr_msgs::LadybugImages::ConstPtr instantiateLadybugImages() const { return imgs_; }

private:
  bool ok_;
  ros::Time time_;
  blf::LLFReader llf_;
  sensor_msgs::Image::Ptr img_;
  stdr_msgs::LadybugImages::Ptr imgs_;
};


class CombinedDgcLogsReader : public AbstractDataReader
{
public:
  CombinedDgcLogsReader();
  /// Load the logs. Optionally skip the first @param skip seconds.
  void load_logs(const std::vector<std::string> & logs, ros::Duration skip=ros::Duration(0));
  bool next();
  bool ok() const { return ok_; }
  ros::Time time() const { return time_; }

  stdr_msgs::ApplanixPose::ConstPtr instantiateApplanixPose() const;
  stdr_msgs::ApplanixGPS::ConstPtr instantiateApplanixGPS() const;
  stdr_msgs::ApplanixRMS::ConstPtr instantiateApplanixRMS() const;
  velodyne_msgs::VelodyneScan::ConstPtr instantiateVelodyneScans() const;
  stdr_msgs::LadybugImages::ConstPtr instantiateLadybugImages() const;

private:
  LogDataReader loggz_reader_;
  VlfDataReader vlf_reader_;
  LlfDataReader llf_reader_;

  typedef std::vector<AbstractDataReader*> Readers;
  Readers readers_;

  bool ok_;
  ros::Time time_;
};

} //namespace log_and_playback

#endif /* __LOG_AND_PLAYBACK__DGCLOG_READER__H__ */
