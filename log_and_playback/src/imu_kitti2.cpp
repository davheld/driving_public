#include <iostream>
#include <boost/filesystem.hpp>
#include "boost/filesystem/fstream.hpp"
#include <fstream>
#include <boost/algorithm/string.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <string>

using namespace std;
using namespace boost;

/***
        This File Converts the Imu data folder(oxts) from the Kitti dataset into
        a single file (.imu) that can be used for future calls to SpinReader.

        Input:
       (path/oxts) the OXTS Folder of Kitti Data
       (intendedfilename.imu) Intended File name of new data

       ***/

int main(int argc, char *argv[]){


  if (argc < 3){
      cout << "Usage: imu_kitt path/oxts  target_file\n";
      return 1;
    }

  boost::filesystem::path p (argv[1]);
  boost::filesystem::path p_ts (argv[1]);
  boost::filesystem::path p_data (argv[1]);
  ofstream kitfile;
  kitfile.open (argv[2], ios::out | ios::app | ios::binary);

  if (boost::filesystem::exists(p)) {
      if (!boost::filesystem::is_regular_file(p)){
          cout << p << ": is not a regular file";
        }
    } else {
      cout << p << ": does not exist \n";
      return 1;
    }



  int idx =0;
  uint64_t epoch_time = 100000;

  boost::filesystem::ifstream imufile(p);
  string line;
  double data[25];
  int satdata[5];
  if (imufile.is_open()){
      while(getline(imufile, line)){
          kitfile << epoch_time <<" ";
          epoch_time += 100000;
          vector<string> fields;
          boost::algorithm::split(fields, line, is_any_of(" "));
          for(int n = 0; n < fields.size(); n++){
              if (n < 25){
                  data[n] = atof(fields[n].c_str());
                  kitfile << data[n] << " ";
                } else {
                  satdata[n-25] = atoi(fields[n].c_str());
                }
            }
          kitfile << "\n";
        }
    }
  imufile.close();
  kitfile.close();
}


