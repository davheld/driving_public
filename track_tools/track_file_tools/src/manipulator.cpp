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
  ss <<"If there is at least one selection, then only the selected track will be " <<std::endl;
  ss <<"in the resulting track file." <<std::endl;
}


/* TODO:
 * - add a command to trim a track from t1 to t2. when tracks are far away they
 *   usually look bad, so it be great to be able to keep only the middle part.
 */


// global variables
track_file_io::Tracks itracks;
std::vector< std::set<track_file_io::Track::_id_type> > track_merge;
std::set<track_file_io::Track::_id_type> track_delete;
std::set<track_file_io::Track::_id_type> track_select;

struct SubCmd
{
  char code;
  track_file_io::Track::_id_type track_id;
};

typedef std::vector< std::set<track_file_io::Track::_id_type> >::iterator TrMrgIt;

// forward declaration
void parseCmds(const std::vector<std::string> &cmds);
void parseCmd(const std::string &cmd);
SubCmd parseSubCmd(const std::string &cmd, unsigned &idx);
void checkCmd(char c);
track_file_io::Track::_id_type getTrackNb(const std::string &cmd, unsigned &idx);
TrMrgIt findTrackMerge(track_file_io::Track::_id_type id);
void process();


int main(int argc, char **argv)
{
  namespace bpo = boost::program_options;
  bpo::options_description opts_desc("Allowed options");
  opts_desc.add_options()
      ("help,h", "produce help message")
      ("args", bpo::value< std::vector<std::string> >(), "the list of files and commands to process");
  bpo::positional_options_description pos_opts_desc;
  pos_opts_desc.add("args", -1);
  bpo::variables_map opts;

  try {
    bpo::store(bpo::command_line_parser(argc, argv).options(opts_desc).positional(pos_opts_desc).run(), opts);
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

  std::string trk_filename;
  std::vector<std::string> cmds;

  BOOST_FOREACH(std::string arg, opts["args"].as< std::vector<std::string> >()) {
    if( boost::algorithm::ends_with(arg, ".trk") && boost::filesystem3::exists(arg) ) {
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

  track_file_io::load(trk_filename, itracks);

  try {
    parseCmds(cmds);
    process();
  }
  catch( std::exception &e ) {
    std::cerr <<"Error: " <<e.what() <<std::endl;
    return 1;
  }

  const std::string otrk_filename = trk_filename.substr(0, trk_filename.size()-4) + "-processed.trk";
  track_file_io::save(otrk_filename, itracks);

  return 0;
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

  for(unsigned i=0; i<cmd.size(); )
  {
    SubCmd c = parseSubCmd(cmd, i);

    if( c.code=='s' ) {
      c.code = 'm';
      std::cerr <<"Select command not supported yet. Treating as merge command." <<std::endl;
    }

    if( track_file_io::find(itracks, c.track_id)==itracks.tracks.end() )
      throw std::runtime_error( (boost::format("Track %d from command %s could not be found in input tracks") % (c.track_id, cmd.c_str())).str() );

    if( c.code=='m' ) {
      if( target_cmd.code=='u' ) {
        target_cmd = c;
        const TrMrgIt it2 = findTrackMerge(c.track_id);
        if( it2==track_merge.end() ) {
          std::set<track_file_io::Track::_id_type> S;
          S.insert(c.track_id);
          track_merge.push_back(S);
        }
      }
      else if( target_cmd.code=='s' || target_cmd.code=='m' ) {
        const TrMrgIt it1 = findTrackMerge(target_cmd.track_id);
        assert(it1!=track_merge.end());
        const TrMrgIt it2 = findTrackMerge(c.track_id);
        if( it2==track_merge.end() ) {
          it1->insert(c.track_id);
        }
        else {
          it1->insert(it2->begin(), it2->end());
          track_merge.erase(it2);
        }
      }
    }
    else if( c.code=='d' ) {
      std::cerr <<"Delete command not supported yet" <<std::endl;
      continue;
      if( target_cmd.code!='u' )
        throw std::runtime_error("delete command cannot be combined with another command");
      track_delete.insert(c.track_id);
    }
    else if( c.code=='s' ) {

    }
  }
}

SubCmd parseSubCmd(const std::string &cmd, unsigned &idx)
{
  SubCmd c;
  checkCmd(cmd[idx]);
  c.code = cmd[idx];
  ++idx;
  c.track_id = getTrackNb(cmd, idx);
  return c;
}

void checkCmd(char c)
{
  static const std::string cmd_codes("smd");
  if( cmd_codes.find(c)==std::string::npos )
    throw std::runtime_error( (boost::format("%c is not a recognized command") % c).str() );
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
  if( !track_delete.empty() || !track_select.empty() ) {
    std::cerr <<"Not supporting select or delete. Ignoring those and processing only the merges." <<std::endl;
  }

  BOOST_FOREACH(const std::set<track_file_io::Track::_id_type> S, track_merge) {
    assert(!S.empty());
    std::vector<track_file_io::Track::_id_type> track_ids;
    track_ids.insert(track_ids.end(), S.begin(), S.end());
    track_file_io::mergeTracks(itracks, track_ids);
  }
}
