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


#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>
#include <tf_conversions/tf_kdl.h>

#include <stdr_lib/exception.h>
#include <stdr_lib/rosparam_helpers.h>

#include <log_and_playback/robot_model.h>

namespace log_and_playback
{

void RobotModel::addParam(const std::string &param)
{
  urdf::Model model;
  if( !model.initParam(param) )
    BOOST_THROW_EXCEPTION(stdr::ex::ExceptionBase() <<stdr::ex::MsgInfo(
                            "Could not load the robot model"));
  addModel(model);
}

void RobotModel::addFile(const std::string &filename)
{
  urdf::Model model;
  if( !model.initFile(filename) )
    BOOST_THROW_EXCEPTION(stdr::ex::ExceptionBase() <<stdr::ex::MsgInfo(
                            "Could not load the robot model"));
  addModel(model);
}


void RobotModel::addChildren(const KDL::SegmentMap::const_iterator segment)
{
  const std::string& root = GetTreeElementSegment(segment->second).getName();
  const std::vector<KDL::SegmentMap::const_iterator>& children = GetTreeElementChildren(segment->second);
  for (unsigned int i=0; i<children.size(); i++) {
    const KDL::Segment& child = GetTreeElementSegment(children[i]->second);
    if (child.getJoint().getType() == KDL::Joint::None) {
      tf::StampedTransform tf_transform;
      tf::transformKDLToTF(child.pose(0), tf_transform);
      tf_transform.frame_id_ = root;
      tf_transform.child_frame_id_ = child.getName();
      static_transforms_.push_back(tf_transform);
    }
    addChildren(children[i]);
  }
}

void RobotModel::addModel(const urdf::ModelInterface& model)
{
  KDL::Tree tree;
  if (!kdl_parser::treeFromUrdfModel(model, tree))
    BOOST_THROW_EXCEPTION(stdr::ex::ExceptionBase() <<stdr::ex::MsgInfo(
                            "Failed to extract kdl tree from xml robot description"));

  addChildren(tree.getRootSegment());
}

}
