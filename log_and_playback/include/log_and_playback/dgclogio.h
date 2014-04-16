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

#ifndef __LOG_AND_PLAYBACK__DGCLOGIO__H__
#define __LOG_AND_PLAYBACK__DGCLOGIO__H__

/* This file contains applanix related IOs. For velodyne related IOs, see
 * velo_support in package velodyne.
 */

#include <iostream>
#include <stdr_msgs/ApplanixPose.h>
#include <stdr_msgs/ApplanixRMS.h>
#include <stdr_msgs/ApplanixDMI.h>
#include <stdr_msgs/ApplanixGPS.h>

namespace log_and_playback
{


/** Given a line of characters, returns true if it starts with APPLANIX_POSE_V2,
  * i.e. if it's a line that describes applanix data.
  */
bool isApplanix(const std::string & line);

/** Given a line of characters, attempts to parse it as applanix data.
  * It returns an empty Ptr if it failed.
  */
stdr_msgs::ApplanixPose::Ptr parseApplanix(const std::string & line);

/// Streams the ApplanixPose to the stream
void streamApplanixPoseAsDgcV2(const stdr_msgs::ApplanixPose &, double first_timestamp, std::ostream &);

void streamApplanixRMSAsDgc(const stdr_msgs::ApplanixRMS &, double first_timestamp, std::ostream &);

void streamApplanixGPSAsDgc(const stdr_msgs::ApplanixGPS &, double first_timestamp, std::ostream &);

void streamApplanixDMIAsDgc(const stdr_msgs::ApplanixDMI &, double first_timestamp, std::ostream &);

} //namespace log_and_playback

#endif // __LOG_AND_PLAYBACK__DGCLOGIO__H__
