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
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/NavSatFix.h>
#include <stdr_msgs/ApplanixPose.h>
#include <global_coords/global_coords.h>


void printHelp(std::ostream& os)
{
  os <<"Usage: rosrun localize gps_fix_pub [options]" <<std::endl;
  os <<std::endl;
  os <<"\t--help, -h\tShow help" <<std::endl;
  os <<"\t--applanix\tSubscribe to the applanix message and publish the GPS pose from the received data" <<std::endl;
  os <<"\t--localizer\tGet the GPS pose from the tf system" <<std::endl;
}

bool use_localizer = true;
std::string utm_zone;
double altitude;
bool received_applanix = false;
sensor_msgs::NavSatFix app_fix;

void onApplanix(const stdr_msgs::ApplanixPose& pose)
{
  received_applanix = true;
  if( !use_localizer ) {
    app_fix.header.stamp = pose.header.stamp;
    app_fix.altitude = pose.altitude;
    app_fix.latitude = pose.latitude;
    app_fix.longitude = pose.longitude;
    app_fix.status.status = sensor_msgs::NavSatStatus::STATUS_FIX;
    app_fix.status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;
  }
  else {
    const global_coords::LatLonCoords ll(pose.latitude, pose.longitude);
    const global_coords::UtmCoords utm(ll);
    utm_zone = utm.zone;
    altitude = pose.altitude;
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "gps_fix_pub", ros::init_options::AnonymousName);

  for(unsigned i=1; i<argc; ++i)
  {
    if( strcmp(argv[i], "--help")==0 || strcmp(argv[i], "-h")==0 ) {
      printHelp(std::cout);
      exit(0);
    }
    else if( strcmp(argv[i], "--applanix")==0 ) {
      use_localizer = false;
    }
    else if( strcmp(argv[i], "--localizer")==0 ) {
      use_localizer = true;
    }
    else {
      std::cerr <<"Unkown option " <<argv[i] <<std::endl;
      printHelp(std::cerr);
      return 1;
    }
  }



  ros::NodeHandle nh;

  std::string applanix_topic = ros::names::remap("applanix");
  if( applanix_topic=="applanix" ) {
    ROS_WARN("You haven't remapped the applanix topic. Using /driving/ApplanixPose");
    applanix_topic = "/driving/ApplanixPose";
  }
  ros::Subscriber app_sub = nh.subscribe(applanix_topic, 1, &onApplanix);

  std::string gps_topic = ros::names::remap("gps");
  if( gps_topic=="gps" ) {
    ROS_WARN("You haven't remapped the gps topic. Using /driving/gps");
    gps_topic = "/driving/gps";
  }
  ros::Publisher pub = nh.advertise<sensor_msgs::NavSatFix>("gps_topic", 1);



  tf::TransformListener tf_listener;
  ros::Rate rate(10);

  while( ros::ok() )
  {
    if( received_applanix )
    {
      if( use_localizer )
      {
        try {
          tf::StampedTransform t;
          tf_listener.lookupTransform("utm", "base_link", ros::Time(0), t);
          sensor_msgs::NavSatFix fix;
          fix.header.stamp = t.stamp_;
          global_coords::UtmCoords utm(t.getOrigin().x(), t.getOrigin().y(), utm_zone);
          global_coords::LatLonCoords ll(utm);
          fix.altitude = altitude;
          fix.latitude = ll.lat;
          fix.longitude = ll.lon;
          fix.status.status = sensor_msgs::NavSatStatus::STATUS_FIX;
          fix.status.service = sensor_msgs::NavSatStatus::SERVICE_GPS;
          pub.publish(fix);
        }
        catch(tf::TransformException& e) {
          ROS_WARN_STREAM(e.what());
        }
      }
      else {
        pub.publish(app_fix);
      }
    }
    else {
      ROS_INFO_STREAM_THROTTLE(1, "Waiting for applanix message on " <<applanix_topic);
    }
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
