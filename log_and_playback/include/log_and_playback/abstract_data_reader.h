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


#ifndef __LOG_AND_PLAYBACK__ABS_DATA_READER__H__
#define __LOG_AND_PLAYBACK__ABS_DATA_READER__H__


#include <velodyne_msgs/VelodyneScan.h>
#include <stdr_msgs/ApplanixPose.h>
#include <stdr_msgs/ApplanixGPS.h>
#include <stdr_msgs/ApplanixDMI.h>
#include <stdr_msgs/ApplanixRMS.h>
#include <stdr_msgs/LocalizePose.h>
#include <stdr_msgs/LadybugImages.h>
#include <stdr_velodyne/point_type.h>


namespace log_and_playback
{

class AbstractDataReader
{
public:
  /// reads the next message and returns whether the source is still valid
  virtual bool next() = 0;

  /// returns whether the source is still valid
  virtual bool ok() const = 0;

  /// timestamp of current message
  virtual ros::Time time() const = 0;

  virtual stdr_msgs::ApplanixPose::ConstPtr instantiateApplanixPose() const
  { return stdr_msgs::ApplanixPose::ConstPtr(); }

  virtual stdr_msgs::ApplanixGPS::ConstPtr instantiateApplanixGPS() const
  { return stdr_msgs::ApplanixGPS::ConstPtr(); }

  virtual stdr_msgs::ApplanixRMS::ConstPtr instantiateApplanixRMS() const
  { return stdr_msgs::ApplanixRMS::ConstPtr(); }

  virtual velodyne_msgs::VelodyneScan::ConstPtr instantiateVelodyneScans() const
  { return velodyne_msgs::VelodyneScan::ConstPtr(); }

  virtual stdr_velodyne::PointCloud::ConstPtr instantiateVelodyneSpin() const
  { return stdr_velodyne::PointCloud::ConstPtr(); }

  virtual stdr_msgs::LadybugImages::ConstPtr instantiateLadybugImages() const
  { return stdr_msgs::LadybugImages::ConstPtr(); }

  virtual stdr_msgs::LocalizePose::ConstPtr instantiateLocalizePose() const
  { return stdr_msgs::LocalizePose::ConstPtr(); }
};


} //namespace

#endif //__LOG_AND_PLAYBACK__ABS_DATA_READER__H__
