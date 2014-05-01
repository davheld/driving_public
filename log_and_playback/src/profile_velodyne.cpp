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
#include <timer/timer.h>
#include <gperftools/profiler.h>

using namespace log_and_playback;
using std::cout;
using std::endl;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "profile_velodyne");

  SpinReader spin_reader;

  namespace bpo = boost::program_options;
  bpo::options_description opts_desc("Allowed options");
  opts_desc.add_options()("help,h", "produce help message");
  SpinReader::addOptions(opts_desc);
  boost::program_options::positional_options_description pos_opts_desc;
  SpinReader::addOptions(pos_opts_desc);
  bpo::variables_map opts;

  try {
    bpo::store(bpo::command_line_parser(argc, argv).options(opts_desc).positional(pos_opts_desc).run(), opts);
    if( opts.count("help") ) {
      cout << "Usage: profile_velodyne [OPTS] logs" << endl;
      cout << endl;
      cout << opts_desc << endl;
      return 0;
    }
    bpo::notify(opts);
  }
  catch( std::exception & e ) {
    ROS_FATAL_STREAM(e.what());
    cout << "Usage: profile_velodyne [OPTS] logs" << endl;
    cout << endl;
    cout << opts_desc << endl;
    return 1;
  }

  try {
    spin_reader.loadCalibrationFromProgramOptions(opts);
    spin_reader.loadTFMFromProgramOptions(opts);
    spin_reader.load(
          opts["logs"].as< std::vector<std::string> >(),
          ros::Duration(opts["start"].as<double>()) );
  } catch( std::exception & e ) {
    ROS_FATAL_STREAM(e.what());
    return 1;
  }

  HighResTimer timer("profiling loop"); timer.start();
  ProfilerStart("profile_velodyne.prof");
  while( spin_reader.nextSpin() && ros::ok() )
  {
    spin_reader.getSpin();
    timer.stop();
    cout <<timer.report() <<endl;
    timer.reset();
    timer.start();
  }
  ProfilerStop();
  cout <<"done" <<endl;
  return 0;
}
