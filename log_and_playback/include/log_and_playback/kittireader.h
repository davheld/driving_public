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
  stdr_msgs::ApplanixPose::Ptr parseApplanix(const std::string & line, double smooth_x,
                                             double smooth_y, double smooth_z);

private:
  std::ifstream file_;
  boost::iostreams::filtering_istream stream_;
  std::string line_;
  bool ok_;

  //double old_hw_timestamp_;
  double old_hw_timestamp_;
  ros::Time time_;
  stdr_msgs::ApplanixPose::ConstPtr pose_;
};


class KittiVeloReader : public AbstractDataReader
{
public:
  KittiVeloReader();
  ~KittiVeloReader();
  void open(const std::string & filename);
  void close();

  /// Advances one data into the file. Returns false on EOF.
  bool next();
  bool ok() const { return ok_; }

  ros::Time time() const { return time_; }

  stdr_velodyne::PointCloud::ConstPtr instantiateVelodyneSpin() const { return spin_; }

protected:
  /// static configuration instance
  stdr_velodyne::Configuration::ConstPtr config_;

private:
  std::ifstream vfile_;
  stdr_velodyne::PointCloud::Ptr spin_;
  ros::Time time_;
  bool ok_;
};


} //namespace log_and_playback
#endif // KITTIREADER_H
