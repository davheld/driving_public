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


#include <localize/fake_localizer.h>
#include <global_coords/global_coords.h>

namespace localize {

FakeLocalizer::FakeLocalizer(double x_shift, double y_shift)
  : x_shift_(x_shift), y_shift_(y_shift)
{
}

void FakeLocalizer::update(const stdr_msgs::ApplanixPose& applanix_pose)
{
  const global_coords::UtmCoords u = global_coords::latLonToUtm(applanix_pose.latitude, applanix_pose.longitude);

  localize_pose_.header.frame_id = "utm";
  localize_pose_.header.stamp = applanix_pose.header.stamp;
  localize_pose_.x_offset = u.x - applanix_pose.smooth_x + x_shift_;
  localize_pose_.y_offset = u.y - applanix_pose.smooth_y + y_shift_;
  localize_pose_.utmzone = u.zone;
  localize_pose_.std_x = localize_pose_.std_y = localize_pose_.std_s = 0;

  update_transforms(localize_pose_);
}

} //namespace localize
