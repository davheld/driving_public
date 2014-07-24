#include "log_and_playback/imu_kitti.h"
#include <iostream>
#include <boost/filesystem.hpp>
#include "boost/filesystem/fstream.hpp"
#include <fstream>
#include <boost/algorithm/string.hpp>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <string>
#include <boost/iostreams/filtering_stream.hpp>

using namespace std;
using namespace boost;


int main3(int argc, char *argv[]){
    std::ifstream file_;
    boost::iostreams::filtering_istream stream_;
    std::string line_;

    time_t epoch_time;
    double data[25];
    char space;

    if (argc < 2){
        return -1;
    }

    file_.open(argv[1], std::ios_base::in);
    stream_.push(file_);


    while( true )
    {
        if(std::getline(stream_, line_) ) {
            //if( ok_ && std::getline(file_, line_) ) {
            std::stringstream ss(line_);
            ss >> epoch_time >> space;
            cout << "Timestamp: "<< epoch_time <<" ";

            for(int i=0; i<25; i++){
                ss >> data[i] >> space;
                cout << "\"" << data[i] << "\"\n";
            }


        }
        else {
            return 0;
        }
    }


}
