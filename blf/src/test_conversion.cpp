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

#include <gtest/gtest.h>
#include <velodyne_pointcloud/rawdata.h>
#include <blf/vlf.h>
#include <stdr_velodyne/conversion.h>


void dump(std::stringstream & ss, unsigned sz)
{
  uint8_t c;
  for(unsigned i=0; i<sz; ++i) {
    ss.read((char *)&c, 1);
    if( i%10==0 ) std::cout <<boost::format("%04d:")%i;
    std::cout <<boost::format(" 0x%02x")%((int)c);
    if( (i+1)%10==0 ) std::cout <<std::endl;
  }
  ss.seekg(0);
  std::cout <<std::endl;
}


TEST(Conversion, StringStreamIOBin)
{
  const uint8_t vals[] = {0x1b, 0x2c, 0x3d, 0x4e};
  std::stringstream ss;
  ss.write((const char *)vals, 4);
  //dump(ss, 4);
  uint8_t * ovals = new uint8_t[4];
  ss.read((char *)ovals, 4);
  for(unsigned i=0; i<4; ++i)
    EXPECT_EQ(vals[i], ovals[i]);
  delete[] ovals;
}

TEST(Conversion, ConvertVelodyneScanToLLF)
{
  // each llf string is 1206 + 12
  static const unsigned N = velodyne_rawdata::PACKET_SIZE+12;

  // create a RawScans with 12 scans
  stdr_msgs::RawScans raw_scans;
  raw_scans.header.frame_id = "velodyne";
  raw_scans.header.stamp = ros::Time(255, 0);
  raw_scans.scans.resize(velodyne_rawdata::BLOCKS_PER_PACKET);
  for( unsigned i=0; i<velodyne_rawdata::BLOCKS_PER_PACKET; ++i ) {
    stdr_msgs::RawScan & scan = raw_scans.scans[i];
    scan.block_id = (i%2==0) ? scan.BLOCK_UPPER : scan.BLOCK_LOWER;
    scan.stamp = raw_scans.header.stamp;
    scan.encoder = (i/2) * 100;
  }

  // convert to llf
  std::stringstream ssrawscan;
  blf::vlf::write(raw_scans, ssrawscan);
  //dump(ssrawscan, N);

  // create a VelodynePacket (initialized with random data)
  velodyne_msgs::VelodyneScan vscan;
  stdr_velodyne::convert(raw_scans, vscan);

  // convert to llf
  std::stringstream ssvscan;
  blf::vlf::write(vscan, ssvscan);
  //dump(ssvscan, N);

  // compare the 2 llf
  for( unsigned i=0; i<N; ++i ) {
    char cv, craw;
    ssvscan.read(&cv, 1);
    ssrawscan.read(&craw, 1);
    EXPECT_EQ(cv, craw);
  }

  ssvscan.seekg(0);
  velodyne_msgs::VelodynePacket pkt;
  blf::vlf::read(ssvscan, pkt);
  for( unsigned j=0; j<pkt.data.size(); ++j )
    EXPECT_EQ(pkt.data[j], vscan.packets[0].data[j]);
}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
