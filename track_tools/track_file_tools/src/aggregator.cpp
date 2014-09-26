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


/** This program takes a trk file as input (see track_file_io) and
  * aggregates all the tracks in one single track with a large point cloud.
  *
  * This is basically abusing the trk format, but it allows to get a larger
  * picture of what's in the trk file when viewing with the track visualizer.
  */

#include <vector>
#include <algorithm>
#include <boost/shared_ptr.hpp>
#include <boost/foreach.hpp>
#include <boost/program_options.hpp>
#include <ros/assert.h>
#include <track_file_io/track_file_io.h>
#include <track_file_io/manipulations.h>

namespace bpo = boost::program_options;

bool framecomp(const track_file_io::Frame* f1, const track_file_io::Frame* f2)
{
  return f1->stamp < f2->stamp;
}

void createGroupFrame(track_file_io::Frame& frame,
                      std::vector<const track_file_io::Frame*>::const_iterator begin,
                      std::vector<const track_file_io::Frame*>::const_iterator end)
{
  frame.robot_pose = (*begin)->robot_pose;
  frame.stamp = (*begin)->stamp;

  std::vector<const sensor_msgs::PointCloud2 *> clouds;
  for(std::vector<const track_file_io::Frame*>::const_iterator it=begin; it<end; ++it)
    clouds.push_back( &((*it)->cloud) );
  track_file_io::concat(frame.cloud, clouds);
}

int main(int argc, char **argv)
{
  std::string input_trk_fn, output_trk_fn;
  bpo::options_description opts;
  opts.add_options()
    ("help,h", "produces this help message")
    ("input,i", bpo::value<std::string>(&input_trk_fn)->required(), "input trk file")
    ("output,o", bpo::value<std::string>(&output_trk_fn)->required(), "output trk file")
    ;
  bpo::positional_options_description pd;
  pd.add("input", 1).add("output", 1);
  bpo::variables_map vm;
  bpo::store(bpo::command_line_parser(argc, argv).options(opts).positional(pd).run(), vm);
  if( vm.count("help") ) {
    std::cout << "track_aggregator: takes a trk track file, aggregates all tracks in one, and save to a new trk file." << std::endl;
    std::cout << std::endl;
    std::cout << opts << std::endl;
    return 0;
  }
  bpo::notify(vm);

  track_file_io::Tracks itf;
  track_file_io::load(input_trk_fn, itf);

  // Get all the frames
  std::vector<const track_file_io::Frame*> frames;
  BOOST_FOREACH(const track_file_io::Track& track, itf.tracks) {
    BOOST_FOREACH(const track_file_io::Frame& frame, track.frames) {
      frames.push_back( &frame );
    }
  }
  std::sort(frames.begin(), frames.end(), framecomp);

  typedef std::pair<std::vector<const track_file_io::Frame*>::const_iterator, std::vector<const track_file_io::Frame*>::const_iterator> FrameGroup;
  std::vector<FrameGroup> frame_groups;
  FrameGroup current_frame_group;
  current_frame_group.first = frames.begin();
  for( std::vector<const track_file_io::Frame*>::const_iterator it=frames.begin()+1; it<frames.end(); ++it ) {
    if( (*it)->stamp > (*current_frame_group.first)->stamp || it+1==frames.end() ) {
      current_frame_group.second = it;
      frame_groups.push_back(current_frame_group);
      current_frame_group.first = it;
    }
  }


  track_file_io::Tracks otf;
  otf.velodyne_pose = itf.velodyne_pose;
  otf.tracks.resize(1);
  track_file_io::Track& tr = otf.tracks.back();
  tr.label = "compound";
  tr.frames.resize(frame_groups.size());

  for(unsigned fg=0; fg<frame_groups.size(); ++fg)
  {
    const double progress = (double)fg / frame_groups.size() * 100;
    printf("progress: %d%%    \r", (int)progress);
    createGroupFrame(tr.frames[fg], frame_groups[fg].first, frame_groups[fg].second);
  }

  track_file_io::save(output_trk_fn, otf);
  return 0;
}
