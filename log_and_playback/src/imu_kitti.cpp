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
    if (boost::filesystem::is_directory(p)) {
      p_ts /= "/timestamps.txt";
      p_data /= "/data";
      if (boost::filesystem::exists(p_ts)){ // check if timestamps file exists
        if (!boost::filesystem::is_regular_file(p_ts)){
          cout << p_ts << ": is not a regular file \n";
          return 1;
        }
      } else {
        cout << p_ts << ": does not exist \n";
        return 1;
      }
      if (boost::filesystem::exists(p_data)){
        if (!boost::filesystem::is_directory(p_data)){
          cout << p_data << "is not a directory \n";
          return 1;
        }
      }
    } else{
      cout << p << ": is not a directory \n";
      return 1;
    }
  } else {
    cout << p << ": does not exist \n";
    return 1;
  }

  typedef vector<boost::filesystem::path> vec;             // store paths,
  vec v;                                // so we can sort them later

  copy(boost::filesystem::directory_iterator(p_data),
       boost::filesystem::directory_iterator(),
       back_inserter(v));

  sort(v.begin(), v.end());             // sort, since directory iteration
  // is not ordered on some file systems


  boost::filesystem::ifstream tsfile(p_ts);
  vector<string> timestamps;
  string timestamp;

  // saving timestamps
  if (tsfile.is_open()){
    while(getline(tsfile, timestamp)){
      timestamps.push_back(timestamp);
    }
  }

  int idx =0;
  for (vec::const_iterator it (v.begin()); it != v.end(); ++it)
  {

    boost::posix_time::ptime t(boost::posix_time::time_from_string(timestamps[idx]));
    boost::posix_time::ptime start(gregorian::date(1970,1,1));
    boost::posix_time::time_duration dur = t - start;\
    uint64_t epoch = dur.total_seconds();
    uint64_t ep2 = dur.total_nanoseconds() / 1e3;
    kitfile << ep2 <<" ";

    boost::filesystem::ifstream imufile(*it);
    string line;
    double data[25];
    int satdata[5];
    if (imufile.is_open()){

      while(getline(imufile, line)){
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

    idx++;
    imufile.close();
  }
  kitfile.close();

}
