#include "log_and_playback/imu_kitti.h"
#include <iostream>
#include <boost/filesystem.hpp>
#include "boost/filesystem/fstream.hpp"
#include <fstream>
#include <boost/algorithm/string.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <string>

using namespace std;
using namespace boost;


int main(int argc, char *argv[]){
    std::ifstream file_;
    boost::iostreams::filtering_istream stream_;
    std::string line_;
    if (argc < 2){
      return -1;
    }

    file_.open(argv[1], std::ios_base::in);


}
