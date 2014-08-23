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

#ifndef LOCALIZER_BASE_H
#define LOCALIZER_BASE_H

#include <tf/transform_broadcaster.h>
#include <stdr_msgs/LocalizePose.h>

namespace localize {

/** Holds the 2 global transforms ("utm" -> "local_utm" -> "smooth") and
  * provides functions to update them from the LocalizePose, broadcast them, etc.
  */
class LocalizerBase
{
public:
  LocalizerBase(std::string frame_prefix="");

  /// Adds both transforms to the transformer. Useful when working offline (playback)
  void addToTransformer(tf::Transformer & transformer, const std::string & authority = "default_authority") const;

  /// broadcast both transforms
  void broadcast(tf::TransformBroadcaster & br) const;

  /// update the transform from the LocalizePose message
  void update_transforms(const stdr_msgs::LocalizePose &);

private:
  tf::StampedTransform transform_utm_to_local_utm_, transform_local_utm_to_smooth_;
  bool first_;
  float ux0_, uy0_; //yes, float! on purpose
};

} //namespace localize

#endif // LOCALIZER_BASE_H
