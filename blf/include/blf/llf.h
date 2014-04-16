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

#ifndef __BLF__LLF_H__
#define __BLF__LLF_H__

#include <sensor_msgs/Image.h>

#include <blf/blf.h>
#include <stdr_msgs/LadybugImages.h>

#define LADYBUG_MAX_PACKET_SIZE 4000000


namespace blf
{

/** \brief reads data from an LLF file.
  *
  * It's a thin wrapper around BLF.
  *
  * //TODO: combine and use the index file for fast seeking
  */
class LLFReader
{
public:
  static const unsigned WIDTH;
  static const unsigned HEIGHT;

  /// Creates and allocate a sensor_msgs/Image pointer to hold ladybug raw
  /// data (which is bayer coded, compressed, rotated, and the 6 cameras stacked
  /// together).
  static sensor_msgs::Image::Ptr makeLadybugRawImage();

public:
  ~LLFReader();

  void open(const char* blf_file_name);
  void close();

  /** Scans the file until a packet with a timestamp greater than the given
   * timestamp is found, then reads it.
   */
  void readTimestampPacket(double time, double *timestamp, std::vector<unsigned char> *data);

  /// Reads the next packet
  void readNextPacket(double *timestamp, std::vector<unsigned char> *data);

private:
  blf::BLF blf_;
};



class LLFWriter
{
public:
  LLFWriter(const std::string & filename);
  void write(const stdr_msgs::LadybugImages & images);

private:
  blf::BLF blf_;
  blf::Packet packet_;
  std::vector<unsigned char> black_img_;
};


} //namespace blf

#endif //__BLF__LLF_H__
