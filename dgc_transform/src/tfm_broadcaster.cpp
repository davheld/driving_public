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

#include <string>
#include <iostream>
#include <fstream>
#include <sstream>

#include <boost/filesystem.hpp>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

#include <dgc_transform/dgc_transform.h>
#include <stdr_lib/rosparam_helpers.h>


// some global variables
char *filename_param=0, *parent_frame_id=0, *child_frame_id=0, *period_ms_str=0;


std::string help_message()
{
  std::stringstream s;
  s <<"Usage: tfm_broadcaster tfm_param parent_frame_id child_frame_id period_in_ms" <<std::endl;
  s <<"Broadcasts a TF transform between the two frames according to the transform file." <<std::endl;
  s <<std::endl <<"Where tfm_param can be:" <<std::endl;
  s << "    - a parameter name pointing to the transform file, or" <<std::endl;
  s << "    - a transform file" <<std::endl;
  s << "Supported file formats are tfm and eig (4x4 matrix)." <<std::endl;
  return s.str();
}


void parse_check_arguments(int argc, char **argv)
{
  if( argc < 4 ) {
    std::cerr << help_message();
    exit(EXIT_FAILURE);
  }
  filename_param = argv[1];
  parent_frame_id = argv[2];
  child_frame_id = argv[3];
  period_ms_str = argv[4];
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "tfm_broadcaster", ros::init_options::AnonymousName);

  parse_check_arguments(argc, argv);

  tf::TransformBroadcaster broadcaster;
  ros::NodeHandle nh;

  // the argument can be either a filename, or a parameter pointing to a filename
  std::string filename;
  if( boost::filesystem::exists(filename_param) ) {
    filename = filename_param;
  }
  else {
    // we were given a parameter
    try {
      GET_ROS_PARAM_ABORT(nh, filename_param, filename);
    }
    catch(ros::InvalidNameException &e) {
      ROS_FATAL_STREAM("Argument " <<filename_param <<" is neither a valid file nor a valid parameter name.");
      ROS_BREAK();
    }

    if( ! boost::filesystem::exists(filename) ) {
      ROS_FATAL_STREAM("Argument " <<filename_param <<" could not be resolved to a valid file.");
      ROS_BREAK();
    }
  }

  tf::Transform tr = dgc_transform::read(filename);
  ros::Rate rate(1000.0/atoi(period_ms_str));

  while( ros::ok() ) {
    broadcaster.sendTransform(tf::StampedTransform(tr, ros::Time::now(), parent_frame_id, child_frame_id));
    rate.sleep();
  }

  return 0;
}
