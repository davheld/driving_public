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

#include <cstdlib>
#include <cmath>
#include <iostream>

#include <blf/llf.h>
#include <ladybug_playback/frame_separator.h>

int main(int argc, char **argv)
{
  if( argc<2 ) {
    std::cerr <<"No LLF file provided" <<std::endl;
    exit(1);
  }
  const char * llf_file = argv[1];

  blf::LLFReader llf;
  sensor_msgs::Image::Ptr packet = blf::LLFReader::makeLadybugRawImage();
  stdr_msgs::LadybugImages separated_images;

  std::cout <<std::setprecision(std::numeric_limits<double>::digits10);

  try {
    llf.open(llf_file);

    double last_time_stamp = 0;
    unsigned nPackets = 0;
    while( ++nPackets<100 ) {

      double timestamp;
      llf.readNextPacket(&timestamp, &(packet->data));
      packet->header.stamp.fromSec(timestamp);
      std::cout <<"Read packet with timestamp " <<timestamp <<".";

      ladybug_playback::separateImages(separated_images, *packet);

      if( last_time_stamp>0 ) {
        double period = timestamp - last_time_stamp;
        double fps = 1 / period;
        std::cout <<" period=" <<(int)(period*1000) <<"ms, fps=" <<(int)round(fps) <<".";
      }
      last_time_stamp = timestamp;
      std::cout <<std::endl;
    }
  }
  catch (stdr::ex::EOFError & e) {
    std::cout <<"Done" <<std::endl;
  }
  catch (stdr::ex::ExceptionBase & e) {
    std::cerr << boost::diagnostic_information(e);
  }

  return 0;
}
