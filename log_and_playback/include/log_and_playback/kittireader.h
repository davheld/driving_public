#ifndef __LOG_AND_PLAYBACK__KITTIREADER_H
#define __LOG_AND_PLAYBACK__KITTIREADER_H

#include <string>
#include <iostream>
#include <fstream>
#include <boost/iostreams/filtering_stream.hpp>
#include <boost/iostreams/filter/gzip.hpp>

#include <blf/llf.h>

#include <log_and_playback/abstract_data_reader.h>

#include <stdr_velodyne/message_filter.h>
#include <stdr_velodyne/pointcloud.h>
#include <stdr_velodyne/config.h>
//#include <log_and_playback/dgclog_reader.h>

namespace log_and_playback
{

class KittiApplanixReader : public AbstractDataReader
{
public:
  KittiApplanixReader() : ok_(false) {}
  ~KittiApplanixReader();
  void open(const std::string & filename);
  void close();

  /// Advances one data into the file. Returns false oon EOF
  bool next();
  bool ok() const { return ok_;}
  ros::Time time() const {return time_;}

  stdr_msgs::ApplanixPose::ConstPtr instantiateApplanixPose() const;
  stdr_msgs::ApplanixGPS::ConstPtr instantiateApplanixGPS() const;
  stdr_msgs::ApplanixRMS::ConstPtr instantiateApplanixRMS() const;
  stdr_msgs::ApplanixPose::Ptr parseApplanix(const std::string & line);


private:
  std::ifstream file_;
  boost::iostreams::filtering_istream stream_;
  std::string line_;
  bool ok_;

  //double old_hw_timestamp_;
  double old_hw_timestamp_;
  ros::Time time_;
  stdr_msgs::ApplanixPose::ConstPtr pose_;
  stdr_msgs::ApplanixGPS::ConstPtr gps_;
  stdr_msgs::ApplanixRMS::ConstPtr rms_;
};


class KittiVeloReader : public AbstractDataReader
{
public:
  KittiVeloReader(); //: ok_(false) { }
  ~KittiVeloReader();
  void open(const std::string & filename);
  void close();

  /// Advances one data into the file. Returns false on EOF.
  bool next();
  bool ok() const { return ok_; }

  ros::Time time() const { return time_; }

  velodyne_msgs::VelodyneScan::ConstPtr instantiateVelodyneScans() const { return scan_; }
  stdr_velodyne::PointCloudPtr instantiateVelodyneSpins() const { return spin_; }

protected:

  /// static configuration instance
  stdr_velodyne::Configuration::ConstPtr config_;

private:
  std::ifstream vfile_;
  velodyne_msgs::VelodyneScan::Ptr scan_;
  stdr_velodyne::PointCloudPtr spin_;
  ros::Time time_;
  bool ok_;
};

class CombinedKittiReader : public AbstractDataReader
{
public:
  CombinedKittiReader();
  /// Load the logs. Optionally skip the first @param skip seconds.
  void load_logs(const std::vector<std::string> & logs, ros::Duration skip=ros::Duration(0));
  bool next();
  bool ok() const { return ok_; }
  ros::Time time() const { return time_; }

  //bool data_reader_time_compare(const AbstractDataReader *a, const AbstractDataReader *b);

  stdr_msgs::ApplanixPose::ConstPtr instantiateApplanixPose() const;
  stdr_msgs::ApplanixGPS::ConstPtr instantiateApplanixGPS() const;
  stdr_msgs::ApplanixRMS::ConstPtr instantiateApplanixRMS() const;
  velodyne_msgs::VelodyneScan::ConstPtr instantiateVelodyneScans() const;
  stdr_velodyne::PointCloudPtr instantiateVelodyneSpins();// const;
  //stdr_msgs::LadybugImages::ConstPtr instantiateLadybugImages() const;

private:
  KittiApplanixReader loggz_reader_;
  KittiVeloReader vlf_reader_;
  //LlfDataReader llf_reader_;

  typedef std::vector<AbstractDataReader*> Readers;
  Readers readers_;

  bool ok_;
  ros::Time time_;
};




} //namespace log_and_playback
#endif // KITTIREADER_H
