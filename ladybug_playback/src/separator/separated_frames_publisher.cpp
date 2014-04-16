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
#include <stdr_msgs/LadybugImages.h>

class Publisher
{
public:
  Publisher();

private:
  ros::NodeHandle nh_;
  ros::Publisher pubs_[24];
  ros::Subscriber sub_;

  void imgCb(const stdr_msgs::LadybugImagesConstPtr &);
};


Publisher::Publisher()
{
  for( unsigned i=0; i<6; ++i ) {
    for( unsigned j=0; j<4; ++j ) {
      const std::string topic = (boost::format("/driving/ladybug/camera%d/bayer_img%d/compressed") % i % j).str();
      pubs_[i*4+j] = nh_.advertise<sensor_msgs::CompressedImage>(topic, 20);
    }
  }

  sub_ = nh_.subscribe("/driving/ladybug/images", 10, &Publisher::imgCb, this);
}

void Publisher::imgCb(const stdr_msgs::LadybugImagesConstPtr & imgs)
{
  for( unsigned i=0; i<24; ++i )
    pubs_[i].publish(imgs->images[i]);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "separated_frames_publisher");
  Publisher p;
  ros::spin();
  return 0;
}
