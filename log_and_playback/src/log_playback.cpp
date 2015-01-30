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

#include <termios.h>
#include <unistd.h>
#include <signal.h>

#include <boost/program_options.hpp>

#include <ros/publisher.h>
#include <image_transport/image_transport.h>
#include <rosgraph_msgs/Clock.h>

#include <stdr_lib/exception.h>
#include <stdr_velodyne/config.h>
#include <log_and_playback/data_reader.h>


using namespace log_and_playback;
namespace bpo = boost::program_options;
using std::cout;
using std::endl;


/*
TODO:
- make the loggz and vlf readers support seeking, following the partial read
  example in blf
- implement a mechanism to prevent from parsing the data when there is no
subscribers. See the ConnectCB mechanism as explained in
http://answers.ros.org/question/11327/argument-for-subscriberstatuscallback-in-advertise
*/

DataReader data_reader;
ros::Publisher clock_pub, scans_pub, spin_pub, pose_pub, gps_pub, rms_pub, ladybug_pub;
float rate, start_offset;
ros::Time start_time, current_time;
ros::WallTime last_wall_time;

bool    terminal_modified = false;
termios orig_flags;
fd_set  stdin_fdset;
int     maxfd;

bool paused = false;


void printTime()
{
  ros::Duration d = current_time - start_time;

  if (paused)
    printf("\r [PAUSED]   Log Time: %13.6f   Duration: %.6f            \r", current_time.toSec(), d.toSec());
  else
    printf("\r [RUNNING]  Log Time: %13.6f   Duration: %.6f            \r", current_time.toSec(), d.toSec());
  fflush(stdout);
}

void setupTerminal()
{
  if (terminal_modified)
    return;

  const int fd = fileno(stdin);
  termios flags;
  tcgetattr(fd, &orig_flags);
  flags = orig_flags;
  flags.c_lflag &= ~ICANON;      // set raw (unset canonical modes)
  flags.c_cc[VMIN]  = 0;         // i.e. min 1 char for blocking, 0 chars for non-blocking
  flags.c_cc[VTIME] = 0;         // block if waiting for char
  tcsetattr(fd, TCSANOW, &flags);

  FD_ZERO(&stdin_fdset);
  FD_SET(fd, &stdin_fdset);
  maxfd = fd + 1;
  terminal_modified = true;
}

void restoreTerminal()
{
  if (!terminal_modified)
    return;

  const int fd = fileno(stdin);
  tcsetattr(fd, TCSANOW, &orig_flags);
  terminal_modified = false;
}

char readCharFromStdin()
{
  fd_set testfd = stdin_fdset;
  timeval timeout;
  timeout.tv_sec  = 0;
  timeout.tv_usec = 0;
  if (select(maxfd, &testfd, NULL, NULL, &timeout) <= 0)
    return EOF;
  return getc(stdin);
}

void timing(const ros::Time & stamp)
{
  static bool first_time = true;
  static ros::Time last_stamp;

  if( first_time ) {
    last_stamp = stamp;
    last_wall_time = ros::WallTime::now();
    first_time = false;
  }
  else {
    double dt = (stamp-last_stamp).toSec() / rate;
    ros::WallTime next_wall_time = last_wall_time + ros::WallDuration(dt);
    ros::WallTime::sleepUntil(next_wall_time);
    last_stamp = stamp;
    last_wall_time = next_wall_time;
  }

  rosgraph_msgs::Clock clock_msg;
  clock_msg.clock = stamp;
  clock_pub.publish(clock_msg);
  current_time = stamp;
}

void doPublish()
{
  if( stdr_msgs::ApplanixPose::ConstPtr pose = data_reader.instantiateApplanixPose() ) {
    timing(pose->header.stamp);
    pose_pub.publish( *pose );
  }
  else if( velodyne_msgs::VelodyneScan::ConstPtr scans = data_reader.instantiateVelodyneScans() ) {
    timing(scans->header.stamp);
    scans_pub.publish( *scans );
  }
  else if( stdr_velodyne::PointCloud::ConstPtr spin = data_reader.instantiateVelodyneSpin() ) {
    std_msgs::Header header;
    pcl_conversions::fromPCL(spin->header, header);
    timing(header.stamp);
    spin_pub.publish( *spin );
  }
  else if( stdr_msgs::ApplanixGPS::ConstPtr gps = data_reader.instantiateApplanixGPS() ) {
    timing(gps->header.stamp);
    gps_pub.publish( *gps );
  }
  else if( stdr_msgs::ApplanixRMS::ConstPtr rms = data_reader.instantiateApplanixRMS() ) {
    timing(rms->header.stamp);
    rms_pub.publish( *rms );
  }
  else if( stdr_msgs::LadybugImages::ConstPtr img = data_reader.instantiateLadybugImages() ) {
    timing(img->header.stamp);
    ladybug_pub.publish( *img );
  }
}

bool signaled = false;
void sighandler(int)
{
  signaled = true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "log_playback");
  ros::NodeHandle nh;
  scans_pub = nh.advertise<velodyne_msgs::VelodyneScan>("/driving/velodyne/packets", 100);
  spin_pub = nh.advertise<stdr_velodyne::PointCloud>("/driving/velodyne/points", 1);
  pose_pub = nh.advertise<stdr_msgs::ApplanixPose>("/driving/ApplanixPose", 100);
  gps_pub = nh.advertise<stdr_msgs::ApplanixGPS>("/driving/ApplanixGPS", 100);
  rms_pub = nh.advertise<stdr_msgs::ApplanixRMS>("/driving/ApplanixRMS", 100);
  ladybug_pub = nh.advertise<stdr_msgs::LadybugImages>("/driving/ladybug/images", 100);
  clock_pub = nh.advertise<rosgraph_msgs::Clock>("/clock", 100);

  bpo::options_description opts_desc("Allowed options");
  opts_desc.add_options()
      ("help,h", "produce help message")
      ("rate,r", bpo::value<float>(&rate)->default_value(1.f), "multiply the publish rate by the given factor")
      ("start,s", bpo::value<float>(&start_offset)->default_value(0.f), "start arg seconds into the bag files");

  bpo::options_description hidden_opts;
  hidden_opts.add_options()
      ("logs", bpo::value< std::vector<std::string> >()->required(), "log files to load (kitti or dgc logs)");

  bpo::options_description all_opts;
  all_opts.add(opts_desc).add(hidden_opts);

  bpo::positional_options_description pos_opts_desc;
  pos_opts_desc.add("logs", -1);

  bpo::variables_map opts;
  try {
    bpo::store(bpo::command_line_parser(argc, argv).options(all_opts).positional(pos_opts_desc).run(), opts);
    if( opts.count("help") ) {
      cout << "Usage: log_playback [OPTS] logs" << endl;
      cout << endl;
      cout << opts_desc << endl;
      return 0;
    }
    bpo::notify(opts);
  }
  catch(std::exception & e) {
    ROS_FATAL_STREAM(e.what());
    cout << "Usage: log_playback [OPTS] logs" << endl;
    cout << endl;
    cout << opts_desc << endl;
    return 1;
  }

  ROS_ASSERT_MSG(rate>0, "The rate factor must be >0");
  ROS_ASSERT_MSG(start_offset>=0, "The start offset must be >0");


  // if we are playing back kitti files, we need to load the calibration files here
  // for dgc logs, we publish scans, and so the calibration files will be loaded
  // by the velodyne processor node.

  bool kitti = false, spinello = false;
  BOOST_FOREACH(const std::string &log, opts["logs"].as< std::vector<std::string> >()) {
    if( boost::filesystem::is_regular_file(log) &&
        boost::algorithm::ends_with(log, ".kit") || boost::algorithm::ends_with(log, ".imu") ) {
      kitti = true;
    }
    else if( boost::filesystem::is_directory(log) ) {
      namespace fs = boost::filesystem;
      fs::path dirpath(log);
      fs::directory_iterator end_iter;
      unsigned ezd_counter = 0;
      for( fs::directory_iterator dir_iter(dirpath); dir_iter != end_iter; ++dir_iter ) {
        if( fs::is_regular_file(dir_iter->status()) && boost::algorithm::ends_with(dir_iter->path().native(), ".ezd") ) {
          ++ ezd_counter;
        }
      }
      if( ezd_counter>0 )
        spinello = true;
    }
  }

  if( kitti || spinello ) {
    stdr_velodyne::Configuration::Ptr config =
        stdr_velodyne::Configuration::getStaticConfigurationInstance();

    std::string calibration_file;
    if( ! ros::param::get("/driving/velodyne/cal_file", calibration_file) )
      BOOST_THROW_EXCEPTION(stdr::ex::ExceptionBase() <<stdr::ex::MsgInfo(
                              "You must provide a configuration file, either on the command line, or as a rosparam."));
    config->readCalibration(calibration_file);

    std::string intensity_file;
    if( ! ros::param::get("/driving/velodyne/int_file", intensity_file) )
      BOOST_THROW_EXCEPTION(stdr::ex::ExceptionBase() <<stdr::ex::MsgInfo(
                              "You must provide an intensity configuration file, either on the command line, or as a rosparam."));
    config->readIntensity(intensity_file);
  }



  data_reader.load( opts["logs"].as< std::vector<std::string> >() );

  // advance into the files until we reach the desired start time
  start_time = data_reader.time();
  const ros::Time first_time = start_time + ros::Duration(start_offset);
  while( data_reader.ok() && nh.ok() && !signaled && data_reader.time() < first_time )
    data_reader.next();

  ros::WallTime paused_time;
  bool read_ok = true;
  signal(SIGINT, sighandler);
  setupTerminal();

  while ( (paused || read_ok) && nh.ok() && !signaled )
  {

    bool charsleftorpaused = true;
    while ( charsleftorpaused && nh.ok() && read_ok && !signaled )
    {
      switch (readCharFromStdin()) {
      case ' ':
        paused = !paused;
        if (paused) {
          paused_time = ros::WallTime::now();
        }
        else {
          last_wall_time += ros::WallTime::now() - paused_time;
        }
        break;
      case 's':
        if (paused) {
          doPublish();
          printTime();
          read_ok = data_reader.next();
        }
        break;
      case EOF:
        if (paused) {
          printTime();
          ros::WallTime::sleepUntil(ros::WallTime::now()+ros::WallDuration(.1));
        }
        else
          charsleftorpaused = false;
      }
    }

    if( !paused ) {
      doPublish();
      printTime();
      read_ok = data_reader.next();
    }
  }

  restoreTerminal();
  puts("");
  return 0;
}
