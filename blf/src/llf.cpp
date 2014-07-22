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

#include <stdint.h>

#include <iostream>
#include <fstream>

#include <ros/ros.h>
#include <ros/package.h>

#include <stdr_lib/roslib_helpers.h>
#include <blf/llf.h>

inline unsigned
swab( unsigned num )
{
  return (( num>>24) |
          ( ( num & 0x00FF0000 ) >> 8 ) |
          ( ( num & 0x0000FF00 ) << 8 ) |
          (  num << 24 ) );
}

namespace blf
{


const unsigned LLFReader::WIDTH = 808;
const unsigned LLFReader::HEIGHT = 14784; //this is the maximum size. In practice,
                                    //due to compression, it's going to be much
                                    //smaller.


sensor_msgs::Image::Ptr LLFReader::makeLadybugRawImage()
{
  sensor_msgs::Image::Ptr img = boost::make_shared<sensor_msgs::Image>();
  img->encoding = "mono8";
  img->height = LLFReader::HEIGHT;
  img->width = LLFReader::WIDTH;
  img->step = LLFReader::WIDTH;
  img->data.resize(LLFReader::WIDTH * LLFReader::HEIGHT);
  return img;
}

LLFReader::~LLFReader()
{
  close();
}

void LLFReader::open( const char* blf_file )
{

  blf_.open(blf_file, "r");

  blf::Index blf_index_;
  if( blf_index_.load(blf_file) ) {
    const int seconds = blf_index_.block.back().timestamp - blf_index_.block.front().timestamp;
    const int hours = seconds / 3600;
    const int min = (seconds - hours * 3600) / 60;
    const int sec = seconds - hours * 3600 - min * 60;
    ROS_INFO("Opened LLF file %s", blf_file);
    ROS_INFO("Logfile length: %02dh:%02dmin:%02dsec", hours, min, sec);
  }
}

void LLFReader::close()
{
  blf_.close();
}

void LLFReader::readTimestampPacket(double time, double *timestamp, std::vector<unsigned char> *data)
{
  blf_.seek_timestamp(time);
  readNextPacket(timestamp, data);
}

void LLFReader::readNextPacket(double *timestamp, std::vector<unsigned char> *data)
{
  uint16_t pkt_id;
  blf_.read_data( &pkt_id, timestamp, data);
}




LLFWriter::LLFWriter(const std::string &filename)
{
  blf_.open(filename.c_str(), "w", false);

  const std::string black_img_fn = stdr::roslib::find_file("blf", "black.jpg");
  ROS_ASSERT(!black_img_fn.empty());
  std::ifstream black_img_f(black_img_fn.c_str());

  if( !black_img_f ) {
    ROS_FATAL_STREAM("Could not find ressource " <<black_img_fn);
    ROS_BREAK();
  }

  while( black_img_f.good() ) {
    char c = black_img_f.get();
    if( black_img_f.good() )
      black_img_.push_back(c);
  }
}

void LLFWriter::write(const stdr_msgs::LadybugImages & images)
{
  packet_.data.clear();

  // see LadybugCompressorHeaderInfo::parse for the structure of the header

  unsigned i;
  unsigned datalen = 0;
  for( i=0; i<images.images.size(); ++i )
    datalen += images.images[i].data.size();
  for( ; i<24; ++i )
    datalen += black_img_.size();
  packet_.data.resize(1024 + datalen); // the header is 1024 bytes

  unsigned char * const pData = &(packet_.data[0]);
  bzero(pData, 1024);
  unsigned * u = (unsigned *)pData;

  u[4] = swab( 0xCAFEBABE ); //signature
  u[5] = swab(2);            //version

  // the last bytes of the header give the offest and size of each of the 24 images
  u = ((unsigned *)(pData+1024)) - 24*2;
  unsigned offset = 1024;
  for( i=0; i<images.images.size(); ++i ) {
    *(u++) = swab(offset);
    const unsigned l = images.images[i].data.size();
    *(u++) = swab(l);
    memcpy(pData+offset, &(images.images[i].data[0]), l);
    offset += l;
  }
  for( ; i<24; ++i ) {
    *(u++) = swab(offset);
    const unsigned l = black_img_.size();
    *(u++) = swab(l);
    memcpy(pData+offset, &(black_img_[0]), l);
    offset += l;
  }

  packet_.timestamp = images.header.stamp.toSec();
  blf_.write_pkt(packet_);
}


} //namespace blf
