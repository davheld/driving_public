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
#include <cmath>
#include <string>
#include <set>

#include <boost/foreach.hpp>
#include <boost/program_options.hpp>

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <stdr_msgs/ApplanixPose.h>
#include <global_coords/global_coords.h>



/** Adds noise to the GPS data in the ApplanixPose message */


using std::cout;
using std::cerr;
using std::endl;
namespace bpo = boost::program_options;

int main(int argc, char **argv)
{
  std::string output_file, input_file;
  double noise_level, noise_period;
  bpo::options_description opts_desc_req("Required arguments");
  opts_desc_req.add_options()
      ("output,o", bpo::value<std::string>(&output_file)->required(), "output file");
  bpo::options_description opts_desc_opt("Options");
  opts_desc_opt.add_options()
      ("help,h", "produce help message")
      ("noise,n", bpo::value<double>(&noise_level)->default_value(1), "noise level (meters)")
      ("period,p", bpo::value<double>(&noise_period)->default_value(10), "noise period (seconds)");
  bpo::options_description opts_desc_hidden;
  opts_desc_hidden.add_options()
      ("bag", bpo::value<std::string>(&input_file)->required());
  bpo::options_description opts_desc_help, opts_desc_all;
  opts_desc_help.add(opts_desc_req).add(opts_desc_opt);
  opts_desc_all.add(opts_desc_help).add(opts_desc_hidden);
  bpo::positional_options_description pos_opts_desc;
  pos_opts_desc.add("bag", 1);
  bpo::variables_map opts;

  try {
    bpo::store(bpo::command_line_parser(argc, argv).options(opts_desc_all).positional(pos_opts_desc).run(), opts);
    if( opts.count("help") ) {
      cout << "Usage: add_gps_offset [OPTS] bag" << endl;
      cout << endl;
      cout << opts_desc_help << endl;
      return 0;
    }
    bpo::notify(opts);
  }
  catch( std::exception & e ) {
    cerr <<e.what() <<endl;
    cerr << "Usage: add_gps_offset [OPTS] bag" << endl;
    cerr << endl;
    cerr << opts_desc_help << endl;
    return 1;
  }


  rosbag::Bag input_bag(input_file), output_bag(output_file, rosbag::bagmode::Write);
  rosbag::View view(input_bag, rosbag::TypeQuery("stdr_msgs/ApplanixPose"));

  bool first = true;
  ros::Time first_time;
  double noise_level_lat, noise_level_lon;
  std::set<std::string> topics_seen;

  BOOST_FOREACH(const rosbag::MessageInstance &msg, view)
  {
    stdr_msgs::ApplanixPose ap = *(msg.instantiate<stdr_msgs::ApplanixPose>());

    if( first ) {
      first_time = ap.header.stamp;
      const global_coords::LatLonCoords ll0(ap.latitude, ap.longitude);
      global_coords::UtmCoords utm(ll0);
      utm += Eigen::Vector2d(noise_level, noise_level);
      const global_coords::LatLonCoords ll1(utm);
      noise_level_lat = fabs(ll1.lat - ll0.lat);
      noise_level_lon = fabs(ll1.lon - ll0.lon);
      first = false;
    }

    const double dt = (ap.header.stamp - first_time).toSec();
    const double w = dt * 2 * M_PI / noise_period;
    ap.latitude += noise_level_lat * sin(w);
    ap.longitude += noise_level_lon * cos(w);
    output_bag.write(msg.getTopic(), msg.getTime(), ap, msg.getConnectionHeader());
    topics_seen.insert(msg.getTopic());
  }

  if( topics_seen.size()!=1 ) {
    cerr <<"WARNING: saw more than one ApplanixPose topic: " <<endl;
    BOOST_FOREACH(const std::string &topic, topics_seen) {
      cerr <<"    " <<topic <<endl;
    }
    return 1;
  }

  return 0;
}
