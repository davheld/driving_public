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

#ifndef __LOG_AND_PLAYBACK__ROBOT_MODEL_H
#define __LOG_AND_PLAYBACK__ROBOT_MODEL_H

#include <vector>

#include <tf/transform_datatypes.h>
#include <urdf_model/model.h>
#include <kdl/tree.hpp>


namespace log_and_playback
{

class RobotModel
{
private:
  std::vector< tf::StampedTransform > static_transforms_;

  /// Adds the static transforms from the robot model.
  void addModel(const urdf::ModelInterface& model);

  void addChildren(const KDL::SegmentMap::const_iterator segment);

public:
  /// Adds the static transforms from the robot model.
  /// The model is given as an XML string, retrieved from the parameter server,
  /// using the given parameter name.
  void addParam(const std::string & param);

  /// Adds the static transforms from the robot model.
  /// The model is given as an XML file.
  void addFile(const std::string & filename);

  inline const std::vector< tf::StampedTransform >& getStaticTransforms() const
  { return static_transforms_; }
};

}

#endif // __LOG_AND_PLAYBACK__ROBOT_MODEL_H
