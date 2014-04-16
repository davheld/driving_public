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
#include <diagnostic_updater/publisher.h>

#include <localize/fake_localizer.h>


class FakeLocalizeNode : public localize::FakeLocalizer
{
public:
  FakeLocalizeNode(double x_shift = 0, double y_shift = 0);

private:
  ros::Subscriber applanix_sub_;
  ros::Publisher localize_pose_pub_;
  tf::TransformBroadcaster tf_br_;
  int32_t msg_count_;

  diagnostic_updater::Updater diagnostics_;
  double diag_min_freq_;
  double diag_max_freq_;
  boost::shared_ptr<diagnostic_updater::TopicDiagnostic> diag_topic_;

  void applanixPoseHandler(const stdr_msgs::ApplanixPose& applanix_pose);
};


FakeLocalizeNode::FakeLocalizeNode(double x_shift, double y_shift)
: FakeLocalizer(x_shift, y_shift)
, msg_count_(0)
{
  ros::NodeHandle nh("/driving");
  localize_pose_pub_ = nh.advertise<stdr_msgs::LocalizePose> ("LocalizePose", 1000);

  // initialize diagnostics
  diagnostics_.setHardwareID("fake_localizer");
  static const double frequency = 200;
  diag_max_freq_ = frequency;
  diag_min_freq_ = frequency;
  ROS_INFO("expected frequency: %.3f (Hz)", frequency);

  using namespace diagnostic_updater;
  static const double tolerance = 0.1;
  static const int window_size = 10;
  diag_topic_.reset(new TopicDiagnostic("localizer", diagnostics_,
                                        FrequencyStatusParam(&diag_min_freq_,
                                                             &diag_max_freq_,
                                                             tolerance,
                                                             window_size),
                                        TimeStampStatusParam()));

  applanix_sub_ = nh.subscribe("ApplanixPose", 1000, &FakeLocalizeNode::applanixPoseHandler, this);
}


void FakeLocalizeNode::applanixPoseHandler(const stdr_msgs::ApplanixPose & applanix_pose)
{
  //if(++msg_count_ < 20) return; //applanix is 200Hz !
  msg_count_ = 0;
  update(applanix_pose);
  localize_pose_pub_.publish( localize_pose_ );
  broadcast(tf_br_);

  // notify diagnostics that a message has been published, updating
  // its status
  diag_topic_->tick(applanix_pose.header.stamp);
  diagnostics_.update();
}



int32_t main(int argc, char** argv)
{
  ros::init(argc, argv, "fake_localize");

  double x_shift=0, y_shift=0;
  if(argc >=3) {
    x_shift = atof(argv[1]);
    y_shift = atof(argv[2]);
  }
  FakeLocalizeNode fake_localizer(x_shift, y_shift);

  ros::spin();
  return 0;
}
