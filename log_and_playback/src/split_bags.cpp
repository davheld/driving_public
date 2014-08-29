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

#include <iostream>
#include <vector>
#include <string>
#include <algorithm>
#include <boost/program_options.hpp>
#include <boost/foreach.hpp>
#include <boost/algorithm/string.hpp>
#include <rosbag/bag.h>
#include <rosbag/view.h>

using namespace std;
namespace bpo = boost::program_options;


/** Tool to split the bags into smaller chunks

Smaller chunks are easier to manipulate, move around, etc...

Matching bags will be split synchronously: matching chunks will contain the
same time range.
*/

/* TODO:
- check space on device first
- resume previously aborted job
*/

int main(int argc, char **argv)
{
  bpo::options_description opts_desc("Allowed options");
  opts_desc.add_options()
      ("help,h", "produce help message")
      ("size,s", bpo::value<double>(), "split so that the chunks from the heaviest bag have the given size (in MB)")
      ("duration,d", bpo::value<double>(), "split in chunks with the given duration (in seconds)")
      ("evaluate,e", "Print what would be done only")
      ("bags", bpo::value< std::vector<std::string> >()->required(), "bags to process")
      ;
  boost::program_options::positional_options_description pos_opts_desc;
  pos_opts_desc.add("bags", -1);
  bpo::variables_map opts;

  try {
    bpo::store(bpo::command_line_parser(argc, argv).options(opts_desc).positional(pos_opts_desc).run(), opts);
    if( opts.count("help") ) {
      cout << "Usage: split_bags [OPTS] bags" << endl;
      cout << endl;
      cout << opts_desc << endl;
      return 0;
    }
    bpo::notify(opts);
  }
  catch( exception & e ) {
    cerr << e.what() << endl;
    cerr << "Usage: split_bags [OPTS] bags" << endl;
    cerr << endl;
    cerr << opts_desc << endl;
    return 1;
  }

  if(!opts.count("duration") && !opts.count("size")) {
    cerr << "Specify either the size of the duration of the chunks." << endl;
    exit(1);
  }
  else if( opts.count("duration") && opts.count("size") ) {
    cerr << "Specify either the size of the duration of the chunks, not both." << endl;
    exit(1);
  }



  vector<string> bags;
  vector<ros::Time> times;
  vector<uint64_t> sizes;
  BOOST_FOREACH(const std::string& bagfn, opts["bags"].as< vector<string> >()) {
    rosbag::Bag bag;
    try {
      bag.open(bagfn);
    }
    catch(rosbag::BagException& e) {
      cerr <<"Skipping " <<bagfn <<endl;
      continue;
    }

    bags.push_back(bagfn);
    sizes.push_back(bag.getSize());
    rosbag::View view(bag);
    times.push_back(view.getBeginTime());
    times.push_back(view.getEndTime());
  }

  if( bags.empty() ) {
    cerr <<"No bag files to process. Aborting." <<endl;
    exit(1);
  }
  const ros::Time min_time = *std::min_element(times.begin(), times.end());
  const ros::Time max_time = *std::max_element(times.begin(), times.end());
  const uint64_t max_size = *std::max_element(sizes.begin(), sizes.end());



  ros::Duration chunk_duration;
  if( opts.count("duration") ) {
    chunk_duration.fromSec(opts["duration"].as<double>());
  }
  else if( opts.count("size") ) {
    const unsigned n_chunks = max_size / (opts["size"].as<double>() * 1e6);
    chunk_duration.fromSec((max_time - min_time).toSec() / n_chunks);
  }

  vector<ros::Time> chunk_times;
  ros::Time t(min_time);
  for( ; t < max_time; t += chunk_duration )
    chunk_times.push_back(t);
  chunk_times.push_back(max_time);

  cout <<"Will split in " <<(chunk_times.size()-1) <<" chunks of "
      <<(int)chunk_duration.toSec() <<" sec, with a max size of "
     <<(int)(max_size / (chunk_times.size()-1) / 1e6) <<"MB" <<endl;

  if( chunk_times.size()<=2 ) {
    cout <<"Only one chunk, aborting." <<endl;
    exit(0);
  }


  if( opts.count("evaluate") )
    return 0;


  BOOST_FOREACH(const std::string& bagfn, opts["bags"].as< vector<string> >()) {
    rosbag::Bag bag(bagfn);

    for(unsigned chunk_id=0; chunk_id<chunk_times.size()-1; ++chunk_id) {
      rosbag::View view(bag, chunk_times[chunk_id], chunk_times[chunk_id+1]);

      vector<string> tokens;
      boost::split(tokens, bagfn, boost::is_any_of("."));
      assert(tokens.size()>=2);
      ostringstream ss;
      ss <<tokens[0];
      ss <<"-chunk" <<setfill('0') <<setw(ceil(log10(chunk_times.size()-1))) <<chunk_id;
      for(unsigned i=1; i<tokens.size(); ++i)
        ss <<'.' <<tokens[i];
      const string chunckbagfn = ss.str();

      cout <<"Processing " <<chunckbagfn <<". Expected size: "
          <<(int)(bag.getSize() / (chunk_times.size()-1) / 1e6) <<"MB" <<endl;

      rosbag::Bag chunkbag(chunckbagfn, rosbag::bagmode::Write);
      BOOST_FOREACH(const rosbag::MessageInstance& m, view) {
        chunkbag.write(m.getTopic(), m.getTime(), m);
      }
    }
  }

  return 0;
}
