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
#include <vector>
#include <iostream>
#include <fstream>

#define BOOST_FILESYSTEM_VERSION 3
#include <boost/filesystem.hpp>
#include <boost/iostreams/device/file.hpp>
#include <boost/iostreams/filtering_stream.hpp>
#include <boost/iostreams/filter/gzip.hpp>
#include <boost/program_options.hpp>
#include <boost/foreach.hpp>
#include <boost/algorithm/string/predicate.hpp>

#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <log_and_playback/dgclogio.h>
#include <stdr_msgs/ApplanixPose.h>
#include <velodyne_msgs/VelodyneScan.h>
#include <stdr_velodyne/conversion.h>
#include <blf/llf.h>
#include <blf/vlf.h>


using namespace log_and_playback;
namespace bpo = boost::program_options;
using std::cout;
using std::endl;


void updateProgressBar(const char *name, const ros::Time& t, const ros::Time& tmin, const ros::Time& tmax)
{
  printf("%s: ", name);
  const int percent = std::max(std::min((int)((t-tmin).toSec()/(tmax-tmin).toSec()*100), 100), 0);
  putchar('|');
  for(int i=0; i<100; i+=5)
    putchar(i<percent ? '=' : ' ');
  putchar('|');
  printf(" %d%%   \r", percent);
}

std::string make_new_filename(const std::string & fn,
                              const std::string & suffix_to_remove,
                              const std::string & new_extension)
{
  std::string out = boost::filesystem::path(fn).stem().string();
  if( boost::algorithm::ends_with(out, suffix_to_remove) )
    out = out.substr(0, out.length()-suffix_to_remove.length());
  return out + new_extension;
}


#define PROC(T, f)                                                           \
  if( stdr_msgs::T::ConstPtr msg = it->instantiate<stdr_msgs::T>() ) { \
    if( first_timestamp==ros::TIME_MIN ) first_timestamp = msg->header.stamp;\
    f(*msg, first_timestamp, os);                                            \
    os <<std::endl;                                                          \
    updateProgressBar("applanix", msg->header.stamp, first_timestamp, last_timestamp); \
    continue;                                                               \
  }

void convertApplanix(const rosbag::Bag & bag, const std::string & bag_filename)
{
  // Open a view on the bag that includes the applanix topics
  std::vector<std::string> topics;
  topics.push_back("/driving/ApplanixPose");
  topics.push_back("/driving/ApplanixRMS");
  topics.push_back("/driving/ApplanixGPS");
  topics.push_back("/driving/ApplanixDMI");
  rosbag::View view(bag, rosbag::TopicQuery(topics));

  if( view.size()==0 )
    return;

  const std::string out_filename = make_new_filename(bag_filename, ".applanix", ".log.gz");
  boost::iostreams::filtering_ostream os;
  os.push(boost::iostreams::gzip_compressor());
  os.push(boost::iostreams::file_sink(out_filename.c_str(), std::ios_base::out | std::ios_base::binary));

  // Read and convert all applanix messages
  ros::Time first_timestamp = ros::TIME_MIN;
  const ros::Time last_timestamp = view.getEndTime();
  for( rosbag::View::iterator it=view.begin(); it!=view.end(); ++it )
  {
    PROC(ApplanixPose, streamApplanixPoseAsDgcV2);
    PROC(ApplanixGPS, streamApplanixGPSAsDgc);
    PROC(ApplanixRMS, streamApplanixRMSAsDgc);
    PROC(ApplanixDMI, streamApplanixDMIAsDgc);
  }
  updateProgressBar("applanix", last_timestamp, first_timestamp, last_timestamp);
  putchar('\n');
}


void convertVelodyne(const rosbag::Bag & bag, const std::string & bag_filename)
{
  rosbag::View view(bag, rosbag::TopicQuery("/driving/velodyne/packets"));

  if( view.size()==0 )
    return;

  const std::string out_filename = make_new_filename(bag_filename, ".velodyne", ".vlf");
  std::ofstream os(out_filename.c_str(), std::ios_base::out | std::ios_base::binary);

  // Read and convert all scans
  const ros::Time first_timestamp = view.getBeginTime();
  const ros::Time last_timestamp = view.getEndTime();
  for( rosbag::View::iterator it=view.begin(); it!=view.end(); ++it ) {
    velodyne_msgs::VelodyneScan::ConstPtr velodyne_scan = it->instantiate<velodyne_msgs::VelodyneScan>();
    if( velodyne_scan ) {
      blf::vlf::write(*velodyne_scan, os);
      updateProgressBar("velodyne", velodyne_scan->header.stamp, first_timestamp, last_timestamp);
    }
    else {
      stdr_msgs::RawScans::ConstPtr raw_scans = it->instantiate<stdr_msgs::RawScans>();
      if( raw_scans ) {
        blf::vlf::write(*raw_scans, os);
        updateProgressBar("velodyne", raw_scans->header.stamp, first_timestamp, last_timestamp);
      }
    }
  }
  updateProgressBar("velodyne", last_timestamp, first_timestamp, last_timestamp);
  putchar('\n');
}

void convertLadybug(const rosbag::Bag & bag, const std::string & bag_filename)
{
  rosbag::View view(bag, rosbag::TopicQuery("/driving/ladybug/images"));

  if( view.size()==0 )
    return;

  const std::string out_filename = make_new_filename(bag_filename, ".ladybug", ".llf");
  blf::LLFWriter writer(out_filename);

  const ros::Time first_timestamp = view.getBeginTime();
  const ros::Time last_timestamp = view.getEndTime();
  for( rosbag::View::iterator it=view.begin(); it!=view.end(); ++it ) {
    if( stdr_msgs::LadybugImages::ConstPtr imgs = it->instantiate<stdr_msgs::LadybugImages>() ) {
      writer.write(*imgs);
      updateProgressBar("ladybug", imgs->header.stamp, first_timestamp, last_timestamp);
    }
  }
  updateProgressBar("ladybug", last_timestamp, first_timestamp, last_timestamp);
  putchar('\n');
}



int main(int argc, char **argv)
{
  bpo::options_description opts_desc("Allowed options");
  opts_desc.add_options()
      ("help,h", "produce help message")
      ("bags", bpo::value< std::vector<std::string> >()->required(), "bags to convert")
      ;
  bpo::positional_options_description pos_opts_desc;
  pos_opts_desc.add("bags", -1);

  bpo::variables_map opts;
  try {
    bpo::store(bpo::command_line_parser(argc, argv).options(opts_desc).positional(pos_opts_desc).run(), opts);
    if( opts.count("help") ) {
      cout << "Usage: bag2dgc [OPTS] bags" << endl;
      cout << endl;
      cout << opts_desc << endl;
      return 0;
    }
    bpo::notify(opts);
  }
  catch(std::exception & e) {
    ROS_FATAL_STREAM(e.what());
    cout << "Usage: bag2dgc [OPTS] bags" << endl;
    cout << endl;
    cout << opts_desc << endl;
    return 1;
  }

  std::vector<std::string> bags = opts["bags"].as< std::vector<std::string> >();

  BOOST_FOREACH(const std::string & bag_filename, bags)
  {
    rosbag::Bag bag(bag_filename);
    convertApplanix(bag, bag_filename);
    convertVelodyne(bag, bag_filename);
    convertLadybug(bag, bag_filename);
  }

  return 0;
}
