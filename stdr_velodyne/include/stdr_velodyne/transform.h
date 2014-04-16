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


#ifndef __STDR_VELODYNE__SCAN_TF_H__
#define __STDR_VELODYNE__SCAN_TF_H__

#include <tf/transform_listener.h>
#include <stdr_velodyne/point_type.h>

namespace stdr_velodyne {

/** Transforms the points to the target frame (in place)
 *
 * This is quite efficient as it uses an intermediate point cloud with all the
 * points that have the same timestamp and transforms them all at once.
 * Throws all that tf::TransformListener::transformPointCloud can throw.
 *
 * @param[in] tfl: the transform listener to use to perform the transformation
 * @param[in] target_frame: the target frame
 * @param[in,out] spin: the scans to transform (in place)
 */
void transform_scan_in_place( const tf::TransformerHelper & tfl,
    const std::string & target_frame,
    PointCloud &spin);


/** Returns a new scan with the points transformed to the target frame.
 *
 * This is quite efficient as it uses an intermediate point cloud with all the
 * points that have the same timestamp and transforms them all at once.
 * Throws all that tf::TransformListener::transformPointCloud can throw.
 *
 * @param[in] tfl: the transform listener to use to perform the transformation
 * @param[in] target_frame: the target frame
 * @param[in] spin_in: the scans to transform
 * @return a newly allocated ProjectedSpin ptr
 */
PointCloud::Ptr transform_scan(
    const tf::TransformerHelper & tfl,
    const std::string & target_frame,
    const PointCloud & spin_in);


/** Returns a new scan with the points transformed to the target frame.
 *
 * This is quite efficient as it uses an intermediate point cloud with all the
 * points that have the same timestamp and transforms them all at once.
 * Throws all that tf::TransformListener::transformPointCloud can throw.
 *
 * @param[in] tfl: the transform listener to use to perform the transformation
 * @param[in] target_frame: the target frame
 * @param[in] spin_in: the scans to transform
 * @param[out] spin_out: the destination scan
 */
void transform_scan(
    const tf::TransformerHelper &tfl,
    const std::string & target_frame,
    const PointCloud & spin_in,
    PointCloud & spin_out);

} //namespace stdr_velodyne

#endif // __STDR_VELODYNE__SCAN_TF_H__
