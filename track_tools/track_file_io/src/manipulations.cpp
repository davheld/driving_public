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

#include <boost/foreach.hpp>
#include <ros/assert.h>
#include <track_file_io/manipulations.h>


namespace track_file_io
{

void concat(sensor_msgs::PointCloud2& smp,
            const std::vector<const sensor_msgs::PointCloud2*>& smps)
{
  if( smps.empty() )
    return;

  smp.header = smps.front()->header;
  smp.height = smps.front()->height;
  smp.fields = smps.front()->fields;
  smp.is_bigendian = smps.front()->is_bigendian;
  smp.point_step = smps.front()->point_step;
  smp.row_step = smps.front()->row_step;
  smp.is_dense = smps.front()->is_dense;

  size_t size = 0;
  smp.width = 0;
  BOOST_FOREACH(const sensor_msgs::PointCloud2* cloud, smps) {
    ROS_ASSERT( smp.header.stamp == cloud->header.stamp );
    ROS_ASSERT( smp.header.frame_id == cloud->header.frame_id );
    ROS_ASSERT( smp.height == cloud->height );
    ROS_ASSERT( smp.fields.size() == cloud->fields.size() );
    for(unsigned i=0; i<smp.fields.size(); ++i ) {
      ROS_ASSERT( smp.fields[i].name == cloud->fields[i].name );
      ROS_ASSERT( smp.fields[i].offset == cloud->fields[i].offset );
      ROS_ASSERT( smp.fields[i].datatype == cloud->fields[i].datatype );
      ROS_ASSERT( smp.fields[i].count == cloud->fields[i].count );
    }
    ROS_ASSERT( smp.is_bigendian == cloud->is_bigendian );
    ROS_ASSERT( smp.point_step == cloud->point_step );
    ROS_ASSERT( smp.row_step == cloud->row_step );
    ROS_ASSERT( smp.is_dense == cloud->is_dense );

    smp.width += cloud->width;
    size += cloud->data.size();
  }

  smp.data.reserve(smp.data.size() + size);
  BOOST_FOREACH(const sensor_msgs::PointCloud2* cloud, smps) {
    smp.data.insert(smp.data.end(),
                    cloud->data.begin(),
                    cloud->data.end());
  }
}


void deleteTrack(Tracks& tracks, Track::_id_type id)
{
  if( tracks.tracks.empty() )
    return;

  // search the index for the given track id
  Tracks::_tracks_type::iterator it =
      std::find_if(tracks.tracks.begin(), tracks.tracks.end(), TrackIdPred(id));
  if( it==tracks.tracks.end() )
    return;

  // if there is only one item, or if the item to delete is the last one, just
  // delete it
  if( tracks.tracks.size()==1 || it+1==tracks.tracks.end() ) {
    tracks.tracks.erase(tracks.tracks.end()-1);
    return;
  }

  // if there are more, we don't want to delete an item in the middle, as this
  // is going to cause to move all the subsequent items. Instead, we'll swap
  // it with the last item and delete it there. This causes only one copy.
  // this will scramble the order of tracks, but we don't care, the tracks
  // are not expected to be sorted anyway...
  *it = tracks.tracks.back();
  tracks.tracks.erase(tracks.tracks.end()-1);
}

void mergeTracks(Tracks& tracks,
                 Track::_id_type id1,
                 Track::_id_type id2)
{
  std::vector<Track::_id_type> ids;
  ids.push_back(id1);
  ids.push_back(id2);
  mergeTracks(tracks, ids);
}

void mergeTracks(Tracks& tracks,
                 const std::vector<Track::_id_type>& ids)
{
  // getting all the frames, by time
  // and the smallest_id
  typedef std::map< double, std::vector<Frame*> > FrameMap;
  FrameMap frame_map;
  Tracks::_tracks_type::iterator track_with_smallest_id = tracks.tracks.end();
  BOOST_FOREACH(const Track::_id_type& id, ids) {
    const Tracks::_tracks_type::iterator it =
        std::find_if(tracks.tracks.begin(), tracks.tracks.end(), TrackIdPred(id));
    if( it==tracks.tracks.end() )
      continue;
    if( track_with_smallest_id==tracks.tracks.end() || it->id < track_with_smallest_id->id )
      track_with_smallest_id = it;
    BOOST_FOREACH(Frame &f, it->frames) {
      frame_map[f.stamp.toSec()].push_back(&f);
    }
  }

  // none of the track ids given exists
  if( track_with_smallest_id==tracks.tracks.end() )
    return;

  // create the new track. Starting with a new empty track.
  Track new_track;
  new_track.id = track_with_smallest_id->id;
  new_track.label = track_with_smallest_id->label; //TODO: how to handle different labels...

  // then add all the frames from the tracks to be merged
  BOOST_FOREACH(const FrameMap::value_type& map_el, frame_map) {
    const std::vector<Frame*> &frames = map_el.second;

    // create a new frame
    new_track.frames.resize(new_track.frames.size()+1);
    new_track.frames.back().robot_pose = frames.front()->robot_pose;
    new_track.frames.back().stamp = frames.front()->stamp;

    // add all the clouds for that frame
    std::vector<const sensor_msgs::PointCloud2*> pcds;
    pcds.resize(frames.size());
    BOOST_FOREACH(const Frame *frame, frames) {
      pcds.push_back(&frame->cloud);
    }
    concat(new_track.frames.back().cloud, pcds);
  }

  // then delete all the tracks we just merged
  BOOST_FOREACH(const Track::_id_type& id, ids) {
    deleteTrack(tracks, id);
  }

  // finally, add the track we just created
  tracks.tracks.push_back(new_track);
}


} //namespace track_file_io
