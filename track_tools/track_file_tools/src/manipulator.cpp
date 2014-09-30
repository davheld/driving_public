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


#include <boost/program_options.hpp>
#include <boost/exception/diagnostic_information.hpp>

#include <ros/console.h>

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

track_file_io::Tracks itracks, otracks;


int main(int argc, char **argv)
{
  namespace bpo = boost::program_options;
  bpo::options_description opts_desc("Allowed options");
  opts_desc.add_options()("help,h", "produce help message");
  bpo::positional_options_description pos_opts_desc;
  pos_opts_desc.add("trk_file", 1);
  pos_opts_desc.add("track_ops", -1);
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
    ROS_FATAL_STREAM(boost::diagnostic_information(e));
    std::cerr << help(opts_desc) << std::endl;
    return 1;
  }

  return 0;
}
