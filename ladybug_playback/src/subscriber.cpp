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

#include <ladybug_playback/subscriber.h>


namespace ladybug_playback
{

Subscriber::Subscriber(const ros::NodeHandle & nh)
  : nh_(nh)
  , it_(nh)
{

}

void Subscriber::subscribe(const std::string& base_topic, uint32_t queue_size,
                           const boost::function<void(const Images::ConstPtr&)>& callback,
                           const ros::VoidPtr& tracked_object,
                           const image_transport::TransportHints &transport_hints)
{
  tracked_object_ = tracked_object;
  callback_ = callback;

  // parse the base_topic and search for "camera" followed by a digit between 0 and 4
  std::string::size_type pos = std::string::npos;
  while( true ) {
    pos = base_topic.rfind("camera", pos);
    ROS_ASSERT( pos != std::string::npos );
    const std::string::size_type dpos = pos + std::string("camera").length();
    ROS_ASSERT( dpos < base_topic.length() );
    const char c = base_topic[dpos];
    if( c>='0' && c<='5' )
      break;

    // the base_topic may contain another instance of cameraX, keep searching.
  }

  base_topic_pre_ = base_topic.substr(0, pos);
  base_topic_post_ = base_topic.substr(pos + std::string("camera").length() + 1);

  for( unsigned i=0; i<5; ++i ) {
    std::stringstream ss;
    ss <<"camera" <<i;
    subs_[i].subscribe(it_, makeTopicName(ss.str()), queue_size, transport_hints);
  }

  sync_.reset( new TSync(subs_[0], subs_[1], subs_[2], subs_[3], subs_[4], queue_size*5) );
  sync_->registerCallback( boost::bind(&Subscriber::imageCallback, this, _1, _2, _3, _4, _5) );
}

void Subscriber::imageCallback(const IMP & im0, const IMP & im1, const IMP & im2, const IMP & im3, const IMP & im4)
{
  ImagesPtr images( new Images );
  images->imgs[0] = im0;
  images->imgs[1] = im1;
  images->imgs[2] = im2;
  images->imgs[3] = im3;
  images->imgs[4] = im4;
  callback_(images);
}

void Subscriber::unsubscribe()
{
  for( unsigned i=0; i<5; ++i ) {
    subs_[i].unsubscribe();
  }
}

} //namespace ladybug_playback
