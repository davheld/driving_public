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

#ifndef __TRAC_FILE_IO__MANIPULATIONS_H__
#define __TRAC_FILE_IO__MANIPULATIONS_H__

#include <vector>
#include <sensor_msgs/PointCloud2.h>
#include <track_file_io/Tracks.h>

namespace track_file_io
{

/// Concatenates all the clouds from @param smps into @param smp (any data in
/// @param smp will be discarded).
void concat(sensor_msgs::PointCloud2& smp,
            const std::vector<const sensor_msgs::PointCloud2*>& smps);

// a predicate to find tracks by id
class TrackIdPred
{
  Track::_id_type id_;
public:
  explicit TrackIdPred(Track::_id_type id) : id_(id) {}
  bool operator() (const Track& tr) const { return tr.id==id_; }
};

Tracks::_tracks_type::iterator find(Tracks& tracks, Track::_id_type id);
Tracks::_tracks_type::const_iterator find(const Tracks& tracks, Track::_id_type id);

void deleteTrack(Tracks& tracks,
                 Track::_id_type id);

// merge the tracks to the one with the smallest id and remove the others
Tracks::_tracks_type::iterator mergeTracks(Tracks& tracks, Track::_id_type id1, Track::_id_type id2);

Tracks::_tracks_type::iterator mergeTracks(Tracks& tracks, const std::vector<Track::_id_type>& ids);

} //namespace track_file_io

#endif //__TRAC_FILE_IO__MANIPULATIONS_H__
