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

#ifndef __STDR_VELODYNE__MESSAGE_FILTER_H__
#define __STDR_VELODYNE__MESSAGE_FILTER_H__


#include <string>
#include <message_filters/simple_filter.h>
#include <agent/lockable.h>
#include <stdr_velodyne/point_type.h>

namespace stdr_velodyne
{

class SpinCollector : public SharedLockable
{
public:
  SpinCollector();

  /// add the new scans to the collection. Returns a complete spin if enough scan were
  /// collected, returns a null pointer otherwise.
  PointCloudPtr add(const PointCloud &);

  bool add(const PointCloud & in, PointCloud & out);

  void clear() { scopeLockWrite; points_.clear(); }

private:
  PointCloud points_;
  int prev_encoder_;
  unsigned spin_start_;

  bool isFirstPoint(const PointType &);
};

/**
 * VelodyneScanMessageFilter makes sure clients get a complete spin from a
 * velodyne sensor. Collects any partial scans until complete scan is assembled.
 *
 * Example:

 * message_filters::Subscriber<stdr_velodyne::PointCloud> velodyne_sub(nh_, "/driving/velodyne/points", 10);
 * VelodyneScanMessageFilter velodyne_filter(velodyne_sub);
 * velodyne_filter.registerCallback(boost::bind(&Foo::spin_callback(), this, _1));
 */

class VelodyneScanMessageFilter : public message_filters::SimpleFilter<PointCloud>
{
public:
  VelodyneScanMessageFilter()
  {
  }

  VelodyneScanMessageFilter(message_filters::SimpleFilter<PointCloud>& f)
  {
    connectInput(f);
  }

  /**
   * \brief Connect this filter's input to another filter's output.  If this filter is already connected, disconnects first.
   */
  void connectInput(message_filters::SimpleFilter<PointCloud>& f)
  {
    message_connection_.disconnect();
    message_connection_ = f.registerCallback(&VelodyneScanMessageFilter::update, this);
  }

private:
  void update(const PointCloudConstPtr & packet)
  {
    PointCloudPtr spin = collector_.add(*packet);
    if( spin )
      signalMessage(spin);
  }

  message_filters::Connection message_connection_;
  SpinCollector collector_;
};

} // namespace stdr_velodyne

#endif // __STDR_VELODYNE__MESSAGE_FILTER_H__
