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

#include <ros/publisher.h>
#include <stdr_lib/exception.h>
#include <log_and_playback/data_reader.h>

using namespace log_and_playback;
using std::cout;
using std::endl;


// print time and sleep
void handle_timing(ros::Time current_time);


int main(int argc, char **argv)
{
  ros::init(argc, argv, "example_spin_reader");
  ros::NodeHandle nh;

  SpinReader spin_reader;

  namespace bpo = boost::program_options;
  bpo::options_description opts_desc("Allowed options");
  opts_desc.add_options()("help,h", "produce help message");
  opts_desc.add(spin_reader.opts_desc);
  bpo::variables_map opts;

  try {
    bpo::store(bpo::command_line_parser(argc, argv).options(opts_desc).positional(spin_reader.pos_opts_desc).run(), opts);
    if( opts.count("help") ) {
      cout << "Usage: example_spin_reader [OPTS] logs" << endl;
      cout << endl;
      cout << opts_desc << endl;
      return 0;
    }
    bpo::notify(opts);
  }
  catch( std::exception & e ) {
    ROS_FATAL_STREAM(e.what());
    cout << "Usage: example_spin_reader [OPTS] logs" << endl;
    cout << endl;
    cout << opts_desc << endl;
    return 1;
  }

  try {
    spin_reader.loadCalibrationFromProgramOptions(opts);
    spin_reader.loadTFMFromProgramOptions(opts);
    spin_reader.load( opts["logs"].as< std::vector<std::string> >() );
  } catch( std::exception & e ) {
    ROS_FATAL_STREAM(e.what());
    return 1;
  }

  spin_reader.tfListener().broadcast();

  while( spin_reader.nextSpin() && nh.ok() )
  {
    stdr_velodyne::PointCloudConstPtr spin = spin_reader.getSpin();
    const std_msgs::Header h = pcl_conversions::fromPCL(spin->header);
    handle_timing(h.stamp);
  }

  printf("\n");
  return 0;
}


void handle_timing(ros::Time current_time)
{
  static ros::Time start_time, prev_spin_time;
  static ros::WallTime wall_time;
  static bool first_time = true;

  if( first_time ) {
    start_time = prev_spin_time = current_time;
    wall_time = ros::WallTime::now();
    first_time = false;
  }
  else {
    wall_time += ros::WallDuration( (current_time - prev_spin_time).toSec() );
    ros::WallTime::sleepUntil(wall_time);
    prev_spin_time = current_time;
  }

  ros::Duration d = current_time - start_time;
  printf("\rLog Time: %13.6f   Duration: %.6f            \r", current_time.toSec(), d.toSec());
  fflush(stdout);
}
