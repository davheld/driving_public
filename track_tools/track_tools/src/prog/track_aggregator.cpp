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


/** This program takes a tm file as input (see track_tools/TrackManager) and
  * aggregates all the tracks in one single track with a large point cloud.
  *
  * This is basically abusing the tm format, but it allows to get a larger
  * picture of what's in the tm file when viewing with track_visualizer.
  */

#include <vector>
#include <algorithm>
#include <boost/shared_ptr.hpp>
#include <boost/foreach.hpp>
#include <boost/program_options.hpp>
#include <track_tools/track_manager.h>

namespace bpo = boost::program_options;

bool framecomp(const track_manager::Frame* f1, const track_manager::Frame* f2)
{
  return f1->timestamp() < f2->timestamp();
}

int main(int argc, char **argv)
{
  std::string input_tm_fn, output_tm_fn;
  bpo::options_description opts;
  opts.add_options()
    ("help,h", "produces this help message")
    ("input,i", bpo::value<std::string>(&input_tm_fn)->required(), "input tm file")
    ("output,o", bpo::value<std::string>(&output_tm_fn)->required(), "output tm file")
    ;
  bpo::positional_options_description pd;
  pd.add("input", 1).add("output", 1);
  bpo::variables_map vm;
  bpo::store(bpo::command_line_parser(argc, argv).options(opts).positional(pd).run(), vm);
  if( vm.count("help") ) {
    std::cout << "track_aggregator: takes a tm track file, aggregates all tracks in one, and save to a new tm file." << std::endl;
    std::cout << std::endl;
    std::cout << opts << std::endl;
    return 0;
  }
  bpo::notify(vm);

  track_manager::TrackManager itmanager, otmanager;

  if( !itmanager.load(input_tm_fn) ) {
    std::cerr <<"Could not load " <<input_tm_fn <<std::endl;
    return 1;
  }

  // Get all the frames
  std::vector<track_manager::Frame*> frames;
  BOOST_FOREACH(boost::shared_ptr<track_manager::Track>& track, itmanager.tracks_) {
    BOOST_FOREACH(boost::shared_ptr<track_manager::Frame>& frame, track->frames_) {
      frames.push_back( frame.get() );
    }
  }
  std::sort(frames.begin(), frames.end(), framecomp);

  boost::shared_ptr<track_manager::Track> tr(new track_manager::Track);
  tr->label_ = "compound";
  boost::shared_ptr<track_manager::Frame> newFrame;

  for(unsigned f=0; f<frames.size(); ++f)
  {
    const double progress = (double)f / frames.size() * 100;
    printf("progress: %d%%    \r", (int)progress);

    const track_manager::Frame* frame = frames[f];
    if( newFrame && frame->timestamp() == newFrame->timestamp() ) {
      const sensor_msgs::PointCloud &pcd = frame->cloud();
      BOOST_FOREACH(const geometry_msgs::Point32& p, pcd.points)
        newFrame->cloud().points.push_back(p);
      for(unsigned i=0; i<newFrame->cloud().channels.size(); ++i)
        newFrame->cloud().channels[i].values.insert(
              newFrame->cloud().channels[i].values.end(),
              pcd.channels[i].values.begin(),
              pcd.channels[i].values.end());
    }
    else {
      if( newFrame ) {
        tr->frames_.push_back(newFrame);
        newFrame.reset();
      }
      if( !newFrame ) {
        newFrame.reset( new track_manager::Frame(*frame) );
        newFrame->cloud() = frame->cloud(); //deep copy
      }
    }
  }

  otmanager.insertTrack(tr);
  otmanager.save(output_tm_fn);
  return 0;
}
