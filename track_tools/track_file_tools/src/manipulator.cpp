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


/** This program is to manipulate trk files.
  *
  * It can select some tracks, optionally merging some of them, and save the
  * resulting trk file.
  */


#include <vector>
#include <set>
#include <string>
#include <exception>

#include <boost/foreach.hpp>
#include <boost/program_options.hpp>
#include <boost/exception/diagnostic_information.hpp>
#include <boost/filesystem.hpp>
#include <boost/algorithm/string/predicate.hpp>

#include <track_file_io/Tracks.h>
#include <track_file_io/track_file_io.h>
#include <track_file_io/manipulations.h>


std::string help(const boost::program_options::options_description &opts_desc)
{
  std::stringstream ss;
  ss <<"Usage: rosrun track_file_tools manipulator [OPTIONS] <track file> <track operations commands>" <<std::endl;
  ss <<std::endl;
  ss <<opts_desc <<std::endl <<std::endl;
  ss <<"track operations commands:" <<std::endl;
  ss <<"--------------------------" <<std::endl;
  ss <<std::endl;
  ss <<"The track operation commands is a sequence of space separated strings," <<std::endl;
  ss <<"one string per operation. Operations are: (d) delete, (s) select, (m) merge." <<std::endl;
  ss <<"Merge and select can be combined as well as several merge together." <<std::endl;
  ss <<"Examples:" <<std::endl;
  ss <<"  s10: select track 10" <<std::endl;
  ss <<"  d10: delete track 10" <<std::endl;
  ss <<"  m10m11m18: merge track 10, 11 and 18" <<std::endl;
  ss <<"  s10m11m18: select track 10 and merge with 11 and 18" <<std::endl;
  ss <<std::endl;
  ss <<"If there is at least one selection, then only the selected tracks will be " <<std::endl;
  ss <<"in the resulting track file." <<std::endl;
  ss <<std::endl;
  ss <<"Additionally operations support some options:" <<std::endl;
  ss <<"  - (b) begin and (e) end can be used with (s) select and (m) merge" <<std::endl;
  ss <<"    to select the tracks between the given frame numbers:" <<std::endl;
  ss <<"      m10m13e30 will merge tracks 10 and 13 and discard all the frames" <<std::endl;
  ss <<"        for those tracks after the 30-th frame" <<std::endl;
  ss <<"      m10m13b5 will merge tracks 10 and 13 and discard all the frames" <<std::endl;
  ss <<"        for those tracks before the 5-th frame" <<std::endl;
  ss <<"    The frame number is counted since the first frame in the whole trk file" <<std::endl;
  ss <<"    as reported by the visualizer." <<std::endl;
  ss <<"    The frame number supplied to (b) begin and (e) end are inclusive," <<std::endl;
  ss <<"    i.e. b5e20 keeps frames from 5 to 20 included." <<std::endl;
  ss <<std::endl;
  ss <<"Note that for the moment (d) delete is not supported (delete commands are ignored)." <<std::endl;
  return ss.str();
}


// types
struct SubCmd
{
  char code;
  union Argument {
    track_file_io::Track::_id_type track_id;
    unsigned frame_nb;
  } argument;
};

typedef std::vector< std::set<track_file_io::Track::_id_type> >::iterator TrMrgIt;

struct TimeRange
{
  ros::Time bgn, end;
  TimeRange() : bgn(ros::TIME_MIN), end(ros::TIME_MAX) { }
};


// global variables
track_file_io::Tracks itracks;
std::set<ros::Time> frame_times;
std::vector< std::set<track_file_io::Track::_id_type> > track_merge;
std::set<track_file_io::Track::_id_type> track_delete;
std::set<track_file_io::Track::_id_type> track_select;
std::map<track_file_io::Track::_id_type, TimeRange> time_ranges;

const std::string op_codes("smd"), frame_nb_codes("eb");


// forward declaration
bool isValidCmdCode(char);
bool isOperation(char);
bool isFrameNb(char);
void parseCmds(const std::vector<std::string> &cmds);
void parseCmd(const std::string &cmd);
SubCmd parseSubCmd(const std::string &cmd, unsigned &idx);
track_file_io::Track::_id_type getTrackNb(const std::string &cmd, unsigned &idx);
TrMrgIt findTrackMerge(track_file_io::Track::_id_type id);
void process();
ros::Time getTimeAtFrameNumber(unsigned);



int main(int argc, char **argv)
{
  namespace bpo = boost::program_options;
  std::string output_filename;
  bpo::options_description opts_desc("Allowed options"), hidden_opts("hidden");
  opts_desc.add_options()
      ("help,h", "produce help message")
      ("output,o", bpo::value<std::string>(&output_filename), "name for the output file");
  hidden_opts.add_options()
      ("args", bpo::value< std::vector<std::string> >(), "the list of files and commands to process");
  bpo::options_description all_options;
  all_options.add(opts_desc).add(hidden_opts);
  bpo::positional_options_description pos_opts_desc;
  pos_opts_desc.add("args", -1);
  bpo::variables_map opts;

  try {
    bpo::store(bpo::command_line_parser(argc, argv).options(all_options).positional(pos_opts_desc).run(), opts);
    if( opts.count("help") ) {
      std::cout << help(opts_desc) << std::endl;
      return 0;
    }
    bpo::notify(opts);
  }
  catch( std::exception & e ) {
    std::cerr << boost::diagnostic_information(e) << std::endl;
    std::cerr << help(opts_desc) << std::endl;
    return 1;
  }

  if( opts.count("args")==0 ) {
    std::cerr << help(opts_desc) << std::endl;
    return 1;
  }

  std::string trk_filename;
  std::vector<std::string> cmds;

  BOOST_FOREACH(std::string arg, opts["args"].as< std::vector<std::string> >()) {
    if( boost::algorithm::ends_with(arg, ".trk") && boost::filesystem::exists(arg) ) {
      if( !trk_filename.empty() ) {
        std::cerr <<"Only accepting one trk file for the moment." <<std::endl;
        return 1;
      }
      trk_filename = arg;
      continue;
    }
    else {
      cmds.push_back(arg);
    }
  }

  if( trk_filename.empty() || cmds.empty() ) {
    std::cerr << "Error: missing track file or command." << std::endl << std::endl;
    std::cerr << help(opts_desc) << std::endl;
    return 1;
  }

  track_file_io::load(trk_filename, itracks);
  BOOST_FOREACH(const track_file_io::Track &track, itracks.tracks) {
    BOOST_FOREACH(const track_file_io::Frame &frame, track.frames) {
      frame_times.insert(frame.stamp);
    }
  }

  try {
    parseCmds(cmds);
    process();
  }
  catch( std::exception &e ) {
    std::cerr <<"Error: " <<e.what() <<std::endl;
    return 1;
  }

  if( output_filename.empty() )
    output_filename = trk_filename.substr(0, trk_filename.size()-4) + "-processed.trk";
  track_file_io::save(output_filename, itracks);

  return 0;
}



bool isValidCmdCode(char c)
{
  return isOperation(c) || isFrameNb(c);
}

bool isOperation(char c)
{
  return op_codes.find(c)!=std::string::npos;
}

bool isFrameNb(char c)
{
  return frame_nb_codes.find(c)!=std::string::npos;
}

// parse all commands
void parseCmds(const std::vector<std::string> &cmds)
{
  BOOST_FOREACH(const std::string &cmd, cmds) {
    parseCmd(cmd);
  }
}

// parse a single command string
void parseCmd(const std::string &cmd)
{
  SubCmd target_cmd = {'u',-1};
  TimeRange time_range;

  for(unsigned i=0; i<cmd.size(); )
  {
    SubCmd c = parseSubCmd(cmd, i);

    if( isOperation(c.code) &&
        track_file_io::find(itracks, c.argument.track_id)==itracks.tracks.end() )
      throw std::runtime_error(
          (boost::format("Track %d from command %s could not be found in input tracks")
           % (c.argument.track_id, cmd.c_str())).str() );

    if( c.code=='s' ) {
      if( target_cmd.code!='u' )
        throw std::runtime_error(std::string("(s) select command must come first in \"") + cmd + "\"");
      target_cmd = c;
      track_select.insert(c.argument.track_id);
      std::set<track_file_io::Track::_id_type> S;
      S.insert(c.argument.track_id);
      track_merge.push_back(S);
    }
    else if( c.code=='m' ) {
      if( target_cmd.code=='u' ) {
        target_cmd = c;
        const TrMrgIt it2 = findTrackMerge(c.argument.track_id);
        if( it2==track_merge.end() ) {
          std::set<track_file_io::Track::_id_type> S;
          S.insert(c.argument.track_id);
          track_merge.push_back(S);
        }
      }
      else if( target_cmd.code=='s' || target_cmd.code=='m' ) {
        const TrMrgIt it1 = findTrackMerge(target_cmd.argument.track_id);
        if( it1==track_merge.end() ) {
          std::set<track_file_io::Track::_id_type> S;
          S.insert(target_cmd.argument.track_id);
          S.insert(c.argument.track_id);
          track_merge.push_back(S);
        }
        else {
          const TrMrgIt it2 = findTrackMerge(c.argument.track_id);
          if( it2==track_merge.end() ) {
            it1->insert(c.argument.track_id);
          }
          else {
            it1->insert(it2->begin(), it2->end());
            track_merge.erase(it2);
          }
        }
      }
    }
    else if( c.code=='d' ) {
      std::cerr <<"Delete command not supported yet" <<std::endl;
      continue;
      if( target_cmd.code!='u' )
        throw std::runtime_error("delete command cannot be combined with another command");
      track_delete.insert(c.argument.track_id);
    }
    else if( c.code=='b' ) {
      time_range.bgn = getTimeAtFrameNumber(c.argument.frame_nb);
    }
    else if( c.code=='e' ) {
      time_range.end = getTimeAtFrameNumber(c.argument.frame_nb);
    }
  }

  if( target_cmd.code=='s' || target_cmd.code=='m' ) {
    const TrMrgIt merge_set_it = findTrackMerge(target_cmd.argument.track_id);
    assert(merge_set_it!=track_merge.end());
    assert(!merge_set_it->empty());
    std::map<track_file_io::Track::_id_type, TimeRange>::iterator time_range_it
        = time_ranges.end();
    BOOST_FOREACH(track_file_io::Track::_id_type id, *merge_set_it) {
      const std::map<track_file_io::Track::_id_type, TimeRange>::iterator it2
          = time_ranges.find(id);
      if( it2!=time_ranges.end() ) {
        time_range_it = it2;
        it2->second.bgn = std::max(it2->second.bgn, time_range.bgn);
        it2->second.end = std::min(it2->second.end, time_range.end);
        break;
      }
    }
    if( time_range_it==time_ranges.end() ) {
      time_ranges[target_cmd.argument.track_id] = time_range;
    }
  }
}

SubCmd parseSubCmd(const std::string &cmd, unsigned &idx)
{
  if( !isValidCmdCode(cmd[idx]) )
    throw std::runtime_error( (boost::format("%c is not a recognized command") % cmd[idx]).str() );
  SubCmd c;
  c.code = cmd[idx];
  ++idx;
  if( isOperation(c.code) )
    c.argument.track_id = getTrackNb(cmd, idx);
  else if( isFrameNb(c.code) )
    c.argument.frame_nb = getTrackNb(cmd, idx); // using the same function for now...
  else
    assert(0);
  return c;
}

track_file_io::Track::_id_type getTrackNb(const std::string &cmd, unsigned &idx)
{
  const char *str = cmd.c_str()+idx;
  char *end = 0;
  const long val = strtol(str, &end, 10);
  if( end==str )
    throw std::runtime_error( (boost::format("Could not get the track number from \"%s\"") % str).str() );
  idx += (end-str);
  return val;
}

TrMrgIt findTrackMerge(track_file_io::Track::_id_type id)
{
  for(TrMrgIt it=track_merge.begin(); it!=track_merge.end(); ++it) {
    if( it->find(id)!=it->end() )
      return it;
  }
  return track_merge.end();
}

void process()
{
  if( !track_delete.empty() ) {
    std::cerr <<"WARNING: not supporting delete yet. Ignoring those." <<std::endl;
  }

  BOOST_FOREACH(const std::set<track_file_io::Track::_id_type> S, track_merge) {
    assert(!S.empty());
    std::vector<track_file_io::Track::_id_type> track_ids;
    track_ids.insert(track_ids.end(), S.begin(), S.end());
    const track_file_io::Tracks::_tracks_type::iterator track_it =
        track_file_io::mergeTracks(itracks, track_ids);

    typedef std::map<track_file_io::Track::_id_type, TimeRange>::const_iterator TimeRangesIt;
    TimeRangesIt time_range_it = time_ranges.end();
    BOOST_FOREACH(track_file_io::Track::_id_type id, track_ids) {
      TimeRangesIt it = time_ranges.find(id);
      if( it!=time_ranges.end() ) {
        assert( time_range_it==time_ranges.end() );
        time_range_it = it;
      }
    }
    assert( time_range_it!=time_ranges.end() );

    while( track_it->frames.front().stamp < time_range_it->second.bgn )
      track_it->frames.erase(track_it->frames.begin());
    while( track_it->frames.back().stamp > time_range_it->second.end )
      track_it->frames.pop_back();
  }

  if( !track_select.empty() ) {
    track_file_io::Tracks stracks;
    stracks.velodyne_pose = itracks.velodyne_pose;
    BOOST_FOREACH(track_file_io::Track::_id_type id, track_select) {
      const track_file_io::Tracks::_tracks_type::const_iterator it =
          track_file_io::find(itracks, id);
      assert( it!= itracks.tracks.end() );
      stracks.tracks.push_back(*it);
    }
    itracks = stracks;
  }
}

ros::Time getTimeAtFrameNumber(unsigned n)
{
  assert(n<frame_times.size());
  std::set<ros::Time>::const_iterator it = frame_times.begin();
  std::advance(it, n);
  return *it;
}
