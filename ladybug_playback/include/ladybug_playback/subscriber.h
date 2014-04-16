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

#ifndef __LADYBUG_PLAYBACK__SUBSCRIBER__H__
#define __LADYBUG_PLAYBACK__SUBSCRIBER__H__

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/time_synchronizer.h>


namespace ladybug_playback
{

class Subscriber
{
public:
  /// An array of 5 images
  struct Images_ {
    sensor_msgs::Image::ConstPtr imgs[5];
    typedef boost::shared_ptr<Images_> Ptr;
    typedef boost::shared_ptr<Images_ const> ConstPtr;
  };
  typedef Images_ Images;
  typedef boost::shared_ptr<Images> ImagesPtr;
  typedef boost::shared_ptr<Images const> ImagesConstPtr;



  /// Constructor
  explicit Subscriber(const ros::NodeHandle& nh);



  /// Subscribe to an image topic, version for arbitrary boost::function object.
  void subscribe(const std::string& base_topic, uint32_t queue_size,
                 const boost::function<void(const ImagesConstPtr&)>& callback,
                 const ros::VoidPtr& tracked_object = ros::VoidPtr(),
                 const image_transport::TransportHints &transport_hints=image_transport::TransportHints());

  /// Subscribe to an image topic, version for bare function.
  void subscribe(const std::string& base_topic, uint32_t queue_size,
                 void(*fp)(const ImagesConstPtr&),
                 const image_transport::TransportHints &transport_hints=image_transport::TransportHints())
  {
    subscribe(base_topic, queue_size,
              boost::function<void(const ImagesConstPtr&)>(fp),
              ros::VoidPtr(), transport_hints);
  }

  /// Subscribe to an image topic, version for class member function with bare pointer.
  template<class T>
  void subscribe(const std::string& base_topic, uint32_t queue_size,
                 void(T::*fp)(const ImagesConstPtr&), T* obj,
                 const image_transport::TransportHints &transport_hints=image_transport::TransportHints())
  {
    subscribe(base_topic, queue_size, boost::bind(fp, obj, _1), ros::VoidPtr(), transport_hints);
  }

  /// Subscribe to an image topic, version for class member function with shared_ptr.
  template<class T>
  void subscribe(const std::string& base_topic, uint32_t queue_size,
                 void(T::*fp)(const ImagesConstPtr&),
                 const boost::shared_ptr<T>& obj,
                 const image_transport::TransportHints &transport_hints=image_transport::TransportHints())
  {
    subscribe(base_topic, queue_size, boost::bind(fp, obj.get(), _1), obj, transport_hints);
  }

  void unsubscribe();

  /// Returns base_topic_pre_ + s + base_topic_post_
  inline std::string makeTopicName(const std::string & s)
  { return base_topic_pre_ + s + base_topic_post_; }

private:
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::SubscriberFilter subs_[5];

  typedef sensor_msgs::Image IM;
  typedef sensor_msgs::Image::ConstPtr IMP;
  typedef message_filters::TimeSynchronizer<IM, IM, IM, IM, IM> TSync;
  boost::shared_ptr<TSync> sync_;
  void imageCallback(const IMP & im0, const IMP & im1, const IMP & im2, const IMP & im3, const IMP & im4);


  ros::VoidPtr tracked_object_;
  boost::function<void(const Subscriber::ImagesConstPtr&)> callback_;

  // extracted from base_topic
  std::string base_topic_pre_, base_topic_post_;
};

} //namespace ladybug_playback

#endif // __LADYBUG_PLAYBACK__SUBSCRIBER__H__
