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

#include <ros/ros.h>

#include <timer/timer.h>
#include <sensor_msgs/image_encodings.h>

#include "pgr_compressor_header_info.h"
#include <ladybug_playback/frame_separator.h>


namespace ladybug_playback
{

stdr_msgs::LadybugImages::Ptr
separateImages(const unsigned char *packet, const ros::Time * stamp)
{
  stdr_msgs::LadybugImages::Ptr images( new stdr_msgs::LadybugImages );
  separateImages(*images, packet, stamp);
  return images;
}

void
separateImages(stdr_msgs::LadybugImages & images,
               const unsigned char *packet,
               const ros::Time * stamp)
{
  LadybugCompressorHeaderInfo compressorInfo;
  compressorInfo.parse(packet);

  images.images.resize(24);

  if( stamp ) images.header.stamp = *stamp;
  images.header.frame_id = "ladybug";

  for( int i = 0; i<6; ++i ) {
    std::stringstream frame_id;
    frame_id << "ladybug" <<i;
    for( int j = 0; j < 4; j ++) {
      const unsigned k = i * 4 + j;
      const LadybugCompressorHeaderInfo::ImageInfo & imageinfo = compressorInfo.getInfo(k);
      images.images[k].format = "jpeg";
      images.images[k].header.frame_id = frame_id.str();
      if( stamp ) images.images[k].header.stamp = *stamp;
      images.images[k].data.assign(imageinfo.pData, imageinfo.pData + imageinfo.size);
    }
  }
}

stdr_msgs::LadybugImages::Ptr
separateImages(const sensor_msgs::Image & packet)
{
  stdr_msgs::LadybugImages::Ptr images( new stdr_msgs::LadybugImages );
  separateImages(*images, &(packet.data[0]), &packet.header.stamp);
  return images;
}

void
separateImages(stdr_msgs::LadybugImages & images, const sensor_msgs::Image & packet)
{
  separateImages(images, &(packet.data[0]), &packet.header.stamp);
}


} //namespace ladybug_playback
