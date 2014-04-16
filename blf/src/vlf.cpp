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

#include <velodyne_pointcloud/rawdata.h>
#include <stdr_lib/exception.h>
#include <blf/vlf.h>
#include <stdr_velodyne/conversion.h>

namespace blf {
namespace vlf {

// The VLF data file is a sequence of VLF packets
struct Packet
{
  /// magic number marking the beginning of a VLF packet
  static const uint8_t START_BYTE;

  uint8_t startbyte; // must be equal to START_BYTE
  double timestamp;
  uint16_t length; // number of data bytes, must be 1206
  velodyne_rawdata::raw_packet_t pkt_data;
  uint8_t checksum; // the checksum of the pkt_data
};

const uint8_t vlf::Packet::START_BYTE = 0x1b;

uint8_t computeChecksum(const uint8_t * bytes)
{
  uint8_t c = 0;
  for (size_t i = 0; i < velodyne_rawdata::PACKET_SIZE; ++i)
    c += *(bytes++);
  return c;
}

void write(const stdr_msgs::RawScans & scans, std::ostream & os)
{
  velodyne_msgs::VelodyneScan vscans;
  stdr_velodyne::convert(scans, vscans);
  write(vscans, os);
}

void write(const velodyne_msgs::VelodyneScan & scans, std::ostream & os)
{
  BOOST_FOREACH(const velodyne_msgs::VelodynePacket & pkt, scans.packets)
    write(pkt, os);
}

void write(const velodyne_msgs::VelodynePacket & pkt, std::ostream & os)
{
  static const uint16_t LENGTH = velodyne_rawdata::PACKET_SIZE;
  os.write((const char *)&Packet::START_BYTE, 1);
  const double timestamp = pkt.stamp.toSec();
  os.write((const char *)&timestamp, 8);
  os.write((const char *)&LENGTH, 2);
  os.write((const char*)&(pkt.data[0]), velodyne_rawdata::PACKET_SIZE);
  const uint8_t cs = computeChecksum(&(pkt.data[0]));
  os.write((const char*)&cs, 1);
}

void read(std::istream & is, velodyne_msgs::VelodynePacket & pkt)
{
  // TODO: throw an exception for end of file?

  uint8_t startbyte;
  is.read((char*)&startbyte, 1);
  if(startbyte!=Packet::START_BYTE)
    BOOST_THROW_EXCEPTION(stdr::ex::IOError() <<stdr::ex::MsgInfo("Not startbyte"));

  double timestamp;
  is.read((char*)&timestamp, 8);

  uint16_t length;
  is.read((char*)&length, 2);
  if(length!=velodyne_rawdata::PACKET_SIZE)
    BOOST_THROW_EXCEPTION(stdr::ex::IOError() <<stdr::ex::MsgInfo("Wrong packet size"));

  is.read((char*)&(pkt.data[0]), velodyne_rawdata::PACKET_SIZE);
  pkt.stamp.fromSec(timestamp);

  uint8_t checksum;
  is.read((char*)&checksum, 1);
  if(checksum != computeChecksum(&(pkt.data[0])))
    BOOST_THROW_EXCEPTION(stdr::ex::IOError() <<stdr::ex::MsgInfo("Bad checksum"));
}

} //namespace vlf
} //namespace blf
