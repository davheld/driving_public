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

#ifndef __LOG_AND_PLAYBACK__BAG_TF_LISTENER_H
#define __LOG_AND_PLAYBACK__BAG_TF_LISTENER_H


#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <stdr_msgs/ApplanixPose.h>
#include <stdr_msgs/LocalizePose.h>

#include <localize/applanix_transformer.h>
#include <localize/fake_localizer.h>


namespace log_and_playback
{

/** A TransformListener that is convenient to use with bags.
 *
 * The TF tree can be updated directly from ApplanixPose/LocalizePose messages,
 * and static transforms can be added to it.
 *
 * Considering the global frame transforms ("utm" -> "local_utm" -> "smooth"):
 * - initially those are updated from the ApplanixPose using the fake localizer,
 *   unless useLocalizePose() was called.
 * - when a LocalizePose message is added (addLocalizePose), the localizer is
 *   reset and the global transforms are subsequently updated from the
 *   LocalizePose messages only.
 * This allows to conditionally work with LocalizePose messages, when not sure
 * whether they will be available.
 */
class BagTFListener : public tf::TransformerHelper
{
public:
  BagTFListener() : broadcast_(false), from_localize_pose_(false), initialized_(false) {}
  bool initialized() const { return initialized_; }
  void broadcast() { broadcast_ = true; }
  void useLocalizePose() { from_localize_pose_ = true; }

  void addApplanixPose(const stdr_msgs::ApplanixPose &);
  void addLocalizePose(const stdr_msgs::LocalizePose &);
  void addTFMsg(const tf::tfMessage & msg);

  /// Adds a static transform to the tree. Time stamp is irrelevant.
  /// Note: Static transforms will be added into the tree each time it is updated,
  /// which is probably too often...
  void addStaticTransform(const tf::StampedTransform &);

  /// Adds a static transforms to the tree. Time stamp is irrelevant.
  void addStaticTransforms(const std::vector< tf::StampedTransform > &);

private:
  std::vector< tf::StampedTransform > static_transforms_;
  bool broadcast_, from_localize_pose_;
  tf::TransformBroadcaster broadcaster_;
  localize::ApplanixTransformer app_trans_;
  localize::FakeLocalizer fake_localizer_;
  bool initialized_;

  void handleStaticTransforms(const ros::Time & stamp);
};

}

#endif // __LOG_AND_PLAYBACK__BAG_TF_LISTENER_H
