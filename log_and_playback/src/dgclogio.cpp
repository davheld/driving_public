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

#include <sstream>
#include <boost/algorithm/string/predicate.hpp>
#include <log_and_playback/dgclogio.h>


namespace log_and_playback
{

bool isApplanix(const std::string & line)
{
  return boost::starts_with(line, "APPLANIX_POSE_V2");
}

stdr_msgs::ApplanixPose::Ptr parseApplanix(const std::string & line)
{
  std::string str;
  std::stringstream ss(line);
  ss >> str;

  if( str.compare("APPLANIX_POSE_V2")==0 )
  {
    double timestamp;
    stdr_msgs::ApplanixPose::Ptr pose( new stdr_msgs::ApplanixPose );
    ss >> pose->smooth_x >> pose->smooth_y >> pose->smooth_z;
    ss >> pose->latitude >> pose->longitude >> pose->altitude;
    ss >> pose->vel_north >> pose->vel_east >> pose->vel_up;
    ss >> pose->speed >> pose->track;
    ss >> pose->roll >> pose->pitch >> pose->yaw;
    ss >> pose->rate_roll >> pose->rate_pitch >> pose->rate_yaw;
    ss >> pose->accel_x >> pose->accel_y >> pose->accel_z;
    ss >> pose->wander >> pose->id >> pose->postprocess_code;
    ss >> pose->hardware_timestamp >> pose->hardware_time_mode >> timestamp;

    if( ss ) {
      pose->header.stamp = ros::Time(timestamp);
      return pose;
    }
  }

  return stdr_msgs::ApplanixPose::Ptr();
}

void streamApplanixPoseAsDgcV2(const stdr_msgs::ApplanixPose & pose, double first_timestamp, std::ostream & os)
{
  os <<"APPLANIX_POSE_V2 ";

  os <<std::setprecision(std::numeric_limits<double>::digits10);
  os <<pose.smooth_x <<' ' <<pose.smooth_y <<' ' <<pose.smooth_z <<' ';
  os <<pose.latitude <<' ' <<pose.longitude <<' ' <<pose.altitude <<' ';

  os <<std::setprecision(std::numeric_limits<float>::digits10);
  os <<pose.vel_north <<' ' <<pose.vel_east <<' ' <<pose.vel_up <<' ';
  os <<pose.speed <<' ' <<pose.track <<' ';

  os <<std::setprecision(std::numeric_limits<double>::digits10);
  os <<pose.roll <<' ' <<pose.pitch <<' ' <<pose.yaw <<' ';
  os <<pose.rate_roll <<' ' <<pose.rate_pitch <<' ' <<pose.rate_yaw <<' ';
  os <<pose.accel_x <<' ' <<pose.accel_y <<' ' <<pose.accel_z <<' ';
  os <<pose.wander <<' ';

  os <<std::setprecision(std::numeric_limits<unsigned>::digits10);
  os <<pose.id <<' ' <<pose.postprocess_code <<' ';

  os <<std::setprecision(std::numeric_limits<double>::digits10);
  os <<pose.hardware_timestamp <<' ';

  os <<std::setprecision(std::numeric_limits<int>::digits10);
  os <<pose.hardware_time_mode <<' ';

  os <<std::setprecision(std::numeric_limits<double>::digits10);
  os <<pose.header.stamp.toSec() <<" nuthouse " <<(pose.header.stamp.toSec() - first_timestamp);
}


void streamApplanixRMSAsDgc(const stdr_msgs::ApplanixRMS & rms, double first_timestamp, std::ostream & os)
{
  os <<"APPLANIX_RMS_V1 ";

  os <<std::setprecision(std::numeric_limits<float>::digits10);
  os <<rms.rms_north <<' ' <<rms.rms_east <<' ' <<rms.rms_up <<' ';
  os <<rms.rms_v_north <<' ' <<rms.rms_v_east <<' ' <<rms.rms_v_up <<' ';
  os <<rms.rms_roll <<' ' <<rms.rms_pitch <<' ' <<rms.rms_yaw <<' ';
  os <<rms.semi_major <<' ' <<rms.semi_minor <<' ' <<rms.orientation <<' ';

  os <<std::setprecision(std::numeric_limits<unsigned>::digits10);
  os <<rms.id <<' ';

  os <<std::setprecision(std::numeric_limits<int>::digits10);
  os <<rms.postprocess_code <<' ';

  os <<std::setprecision(std::numeric_limits<double>::digits10);
  os <<rms.hardware_timestamp <<' ';

  os <<std::setprecision(std::numeric_limits<int>::digits10);
  os <<rms.hardware_time_mode;

  os <<std::setprecision(std::numeric_limits<double>::digits10);
  os <<rms.header.stamp.toSec() <<" nuthouse " <<(rms.header.stamp.toSec() - first_timestamp);
}

void streamGPS(int i, unsigned u, double d, std::ostream & os)
{
  os <<std::setprecision(std::numeric_limits<int>::digits10);
  os <<i <<' ';

  os <<std::setprecision(std::numeric_limits<unsigned>::digits10);
  os <<u <<' ';

  os <<std::setprecision(std::numeric_limits<double>::digits10);
  os <<d <<' ';
}

void streamApplanixGPSAsDgc(const stdr_msgs::ApplanixGPS & gps, double first_timestamp, std::ostream & os)
{
  os <<"APPLANIX_GPS_V1 ";

  streamGPS(gps.primary_sats, gps.primary_id, gps.primary_timestamp, os);
  streamGPS(gps.secondary_sats, gps.secondary_id, gps.secondary_timestamp, os);
  streamGPS(gps.gams_solution_code, gps.gams_id, gps.gams_timestamp, os);

  os <<gps.header.stamp.toSec() <<" nuthouse " <<(gps.header.stamp.toSec() - first_timestamp);
}

void streamApplanixDMIAsDgc(const stdr_msgs::ApplanixDMI & dmi, double first_timestamp, std::ostream & os)
{
  os <<"APPLANIX_DMI_V1 ";

  os <<std::setprecision(std::numeric_limits<double>::digits10);
  os <<dmi.signed_odometer <<' ' <<dmi.unsigned_odometer<<' ';

  os <<std::setprecision(std::numeric_limits<unsigned>::digits10);
  os <<dmi.id <<' ';

  os <<std::setprecision(std::numeric_limits<double>::digits10);
  os <<dmi.hardware_timestamp <<' ';

  os <<std::setprecision(std::numeric_limits<int>::digits10);
  os <<dmi.hardware_time_mode;

  os <<std::setprecision(std::numeric_limits<double>::digits10);
  os <<dmi.header.stamp.toSec() <<" nuthouse " <<(dmi.header.stamp.toSec() - first_timestamp);
}

} //namespace log_and_playback
