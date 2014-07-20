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
#include <boost/make_shared.hpp>

#include <stdr_lib/exception.h>
#include <blf/vlf.h>
#include <stdr_velodyne/conversion.h>
#include <ladybug_playback/frame_separator.h>
#include <log_and_playback/dgclog_reader.h>
#include <log_and_playback/dgclogio.h>

namespace log_and_playback
{

LogDataReader::~LogDataReader()
{
  close();
}

void LogDataReader::open(const std::string & filename)
{
  if( filename.substr(filename.length()-2).compare("gz")==0 ) {
    file_.open(filename.c_str(), std::ios_base::in | std::ios_base::binary);
    stream_.push(boost::iostreams::gzip_decompressor());
  }
  else {
    file_.open(filename.c_str(), std::ios_base::in);
  }
  stream_.push(file_);
  ok_ = true;
}

void LogDataReader::close()
{
  file_.close();
}

bool LogDataReader::next()
{
  pose_.reset();
  gps_.reset();
  rms_.reset();

  // read until we find one of them 3
  // (actually we only parse pose messages for now)
  while( true )
  {
    if( ok_ && std::getline(stream_, line_) ) {
      pose_ = parseApplanix(line_);
      if( pose_ ) {
        time_ = pose_->header.stamp;
        return true;
      }
    }
    else {
      ok_ = false;
      return ok_;
    }
  }

  return ok_;
}

stdr_msgs::ApplanixPose::ConstPtr
LogDataReader::instantiateApplanixPose() const
{
  return pose_;
}

stdr_msgs::ApplanixGPS::ConstPtr
LogDataReader::instantiateApplanixGPS() const
{
  return gps_;
}

stdr_msgs::ApplanixRMS::ConstPtr
LogDataReader::instantiateApplanixRMS() const
{
  return rms_;
}



VlfDataReader::~VlfDataReader()
{
  close();
}

void VlfDataReader::open(const std::string & filename)
{
  vfile_.open(filename.c_str());
  ok_ = true;
}

void VlfDataReader::close()
{
  vfile_.close();
  ok_ = false;
}

bool VlfDataReader::next()
{
  // not open
  if( !vfile_ )
    return false;

  velodyne_msgs::VelodynePacket packet;
  try {
    blf::vlf::read(vfile_, packet);
  }
  catch (stdr::ex::IOError& e) {
    ok_ = false;
    return ok_;
  }

  scan_.reset(new velodyne_msgs::VelodyneScan);

  //TODO: get the frame_id from rosparam /driving/velodyne/frame_id
  scan_->header.frame_id = "velodyne";
  scan_->header.stamp = packet.stamp;
  scan_->packets.push_back(packet);
  time_ = scan_->header.stamp;

  return ok_;
}



LlfDataReader::~LlfDataReader()
{
  close();
}

void LlfDataReader::open(const std::string &filename)
{
  llf_.open(filename.c_str());
  ok_ = true;
}

void LlfDataReader::close()
{
  llf_.close();
}

bool LlfDataReader::next()
{
  if( !img_ )
    img_ = blf::LLFReader::makeLadybugRawImage();

  imgs_.reset(new stdr_msgs::LadybugImages);

  try {
    double t;
    llf_.readNextPacket(&t, &(img_->data));
    time_.fromSec(t);
    img_->header.stamp = time_;
    ladybug_playback::separateImages(*imgs_, *img_);
  }
  catch( stdr::ex::EOFError & e ) {
    ok_ = false;
    imgs_.reset();
  }
  return ok_;
}

} //namespace log_and_playback
