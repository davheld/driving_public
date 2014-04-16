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
#include <stdr_velodyne/conversion.h>

namespace stdr_velodyne
{

#define WRITE_BYTES(dst, val) memcpy(dst, &(val), sizeof(val)); dst+=sizeof(val)

void serialize_packet(uint8_t *dst, const stdr_msgs::RawScan *src)
{
  for( int i=0; i<velodyne_rawdata::BLOCKS_PER_PACKET; ++i ) {
    const stdr_msgs::RawScan & scan = *(src+i);
    ROS_ASSERT(scan.stamp == src->stamp);

    const uint16_t b = (scan.block_id==stdr_msgs::RawScan::BLOCK_LOWER) ?
          velodyne_rawdata::LOWER_BANK : velodyne_rawdata::UPPER_BANK;
    WRITE_BYTES(dst, b);

    WRITE_BYTES(dst, scan.encoder);
    for( int j=0; j<velodyne_rawdata::SCANS_PER_BLOCK; ++j ) {
      WRITE_BYTES(dst, scan.range[j]);
      WRITE_BYTES(dst, scan.intensity[j]);
    }
  }
}


void convert(const stdr_msgs::RawScans & in, velodyne_msgs::VelodyneScan & out)
{
  // if this fails, then I will have to implement something more complex
  ROS_ASSERT( in.scans.size() % velodyne_rawdata::BLOCKS_PER_PACKET == 0 );

  out.header = in.header;
  out.packets.resize(in.scans.size()/velodyne_rawdata::BLOCKS_PER_PACKET);
  for( unsigned i=0; i<out.packets.size(); ++i ) {
    const unsigned I = i*velodyne_rawdata::BLOCKS_PER_PACKET;
    serialize_packet( &(out.packets[i].data[0]), &(in.scans[I]) );
    out.packets[i].stamp = in.scans[I].stamp;
  }
}


} //namespace stdr_velodyne
