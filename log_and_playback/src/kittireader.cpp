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

#include "log_and_playback/kittireader.h"
#include <boost/foreach.hpp>
#include <boost/make_shared.hpp>

#include <stdr_lib/exception.h>
#include <blf/vlf.h>
#include <stdr_velodyne/conversion.h>
#include <ladybug_playback/frame_separator.h>

#include <velodyne_pointcloud/rawdata.h>
#include <stdr_velodyne/pointcloud.h>
#include <stdr_velodyne/point_type.h>
#include <stdr_velodyne/config.h>

namespace log_and_playback
{


void KittiApplanixReader::open(const std::string & filename)
{

    file_.open(filename.c_str(), std::ios_base::in);
    stream_.push(file_);
    ok_ = true;
    old_hw_timestamp_ = 0;
}


KittiApplanixReader::~KittiApplanixReader(){
    close();
}

void KittiApplanixReader::close(){
    file_.close();
}

stdr_msgs::ApplanixPose::Ptr KittiApplanixReader::parseApplanix(const std::string & line){
    uint64_t epoch_time;
    double ep_time;
    double data[25];
    char space;
    std::stringstream ss(line);

    ep_time = static_cast<double>(epoch_time) * 1e-9;
    for(int i=0; i<25; i++){
        ss >> data[i];
    }

    stdr_msgs::ApplanixPose::Ptr pose( new stdr_msgs::ApplanixPose );
    pose->latitude = data[0];
    pose->longitude = data[1];
    pose->altitude = data[2];
    pose->roll = data[3];
    pose->pitch = data[4];
    pose->yaw = data[5];
    pose->vel_north = (float)(data[6]);
    pose->vel_east = (float)(data[7]);
    pose->vel_up = (float) (data[10]);
    pose->rate_roll = data[17];
    pose->rate_pitch = data[18];
    pose->rate_yaw = data[19];
    pose->accel_x = data[11];
    pose->accel_y = data[12];
    pose->accel_z = data[13];
    pose->wander =0;
    pose->id = 0;
    pose->speed = (float)(sqrt(pose->vel_north * pose->vel_north
                               + pose->vel_east * pose->vel_east));
    pose->track = (float)(atan2(pose->vel_north, pose->vel_east));

    if (old_hw_timestamp_ !=0){
        double dt = ep_time - old_hw_timestamp_;
        pose->smooth_x += pose->vel_east * dt;
        pose->smooth_y += pose->vel_north * dt;
        pose->smooth_z += pose->vel_up * dt;

    }else{
        pose->smooth_x = 0;
        pose->smooth_y = 0;
        pose->smooth_z = 0;
    }
    old_hw_timestamp_ = ep_time;
    pose->hardware_timestamp = ep_time;
    time_ = ros::Time(ep_time);
    pose->header.stamp = time_;
    return pose;
}

bool KittiApplanixReader::next(){
    pose_.reset();
    gps_.reset();
    rms_.reset();

    while( true )
    {
        if( ok_ && std::getline(stream_, line_) ) {
            pose_ = parseApplanix(line_);
            if( pose_ ) {
                time_ = pose_->header.stamp;
                return true;
            }
        }
        else {
            ok_ = false;
            return ok_;
        }
    }

    return ok_;
    return false;
}

stdr_msgs::ApplanixPose::ConstPtr
KittiApplanixReader::instantiateApplanixPose() const
{
    return pose_;
}

stdr_msgs::ApplanixGPS::ConstPtr
KittiApplanixReader::instantiateApplanixGPS() const
{
    return gps_;
}

stdr_msgs::ApplanixRMS::ConstPtr
KittiApplanixReader::instantiateApplanixRMS() const
{
    return rms_;
}

KittiVeloReader::~KittiVeloReader()
{
    close();
}

KittiVeloReader::KittiVeloReader(){
    config_ = stdr_velodyne::Configuration::getStaticConfigurationInstance();
    ok_ = false;
    spin_ = boost::make_shared<stdr_velodyne::PointCloud>();
}

void KittiVeloReader::open(const std::string & filename)
{
    vfile_.open(filename.c_str());
    ok_ = true;
    config_ = stdr_velodyne::Configuration::getStaticConfigurationInstance();
    ROS_ASSERT(config_);
}

void KittiVeloReader::close()
{
    vfile_.close();
    ok_ = false;
}

bool KittiVeloReader::next()
{

    ROS_ASSERT(config_);

    if( !vfile_ && vfile_.good())
        return false;

    unsigned int num_points;
    uint64_t t_start, t_end;

    try {
        vfile_.read((char *)(&num_points), sizeof(num_points));
        vfile_.read((char *)(&t_start), sizeof(t_start));
        vfile_.read((char *)(&t_end), sizeof(t_end));

        if(!vfile_.good()){
            ok_ = false;
            spin_->clear();
            return ok_;
        }

        spin_->clear();
        spin_->reserve(spin_->size() + num_points);

        spin_->header.frame_id = "velodyne";
        spin_->header.seq = 14;

        spin_->header.stamp = t_start;
        // Recent Additions
        time_ =ros::Time(t_start *1E-6);
        stdr_velodyne::PointType pt;

        float x,y,z;
        float intensity;
        float distance;
        float h_angle, v_angle;
        double timestamp;
        uint8_t beam_id, beam_nb;
        uint16_t encoder;
        uint32_t rgb;

        for( int i =0; i< num_points; i++){

            // load point info from .kit file
            vfile_.read((char *)(&x), sizeof(x));
            vfile_.read((char *)(&y), sizeof(y));
            vfile_.read((char *)(&z), sizeof(z));
            vfile_.read((char *)(&intensity), sizeof(intensity));
            vfile_.read((char *)(&h_angle), sizeof(h_angle));
            vfile_.read((char *)(&beam_id), sizeof(beam_id));
            vfile_.read((char *)(&distance), sizeof(distance));

            // load configuration data
            ROS_ASSERT(config_);

            const stdr_velodyne::RingConfig & rcfg = config_->getRingConfig(beam_id);
            v_angle = rcfg.vert_angle_.getRads();
            beam_nb = config_->getInvBeamOrder(beam_id);
            encoder = (uint16_t)(h_angle* 100);

            // update point data
            pt.x = x;
            pt.y = y;
            pt.z = z;
            pt.intensity = intensity;
            pt.h_angle = h_angle;
            pt.encoder = encoder;
            pt.v_angle = v_angle;
            pt.beam_id = beam_id -1 ;
            pt.beam_nb = beam_id - 1;//beam_nb;
            pt.timestamp = static_cast<double>(t_start) * 1e-6;
            pt.distance = distance;
            // add to pointcloud
            spin_->push_back(pt);
        }

    }catch (stdr::ex::IOError& e) {
        ok_ = false;
        return ok_;
    }
}

CombinedKittiReader::CombinedKittiReader(){
}


bool data_reader_time_compare2(const AbstractDataReader *a, const AbstractDataReader *b)
{
    return a->time() < b->time();
}


void CombinedKittiReader::load_logs(const std::vector<std::string> & logs, ros::Duration skip){
    ok_ = false;
    readers_.clear();

    BOOST_FOREACH(std::string const & path, logs) {
        if( (path.length()>4 && path.substr(path.length()-4).compare(".kit")==0) )
            vlf_reader_.open(path);
        else if( path.length()>4 && path.substr(path.length()-4).compare(".imu")==0 )
            loggz_reader_.open(path);
    }

    if( loggz_reader_.ok() ) {
        loggz_reader_.next();
        readers_.push_back(&loggz_reader_);
        ok_ = true;
    }
    if( vlf_reader_.ok() ) {
        vlf_reader_.next();
        readers_.push_back(&vlf_reader_);
        ok_ = true;
    }

    std::sort(readers_.begin(), readers_.end(), data_reader_time_compare2);

    time_ = readers_.front()->time();
    const ros::Time start_time = time_ + skip;
    while( time_ < start_time && next() );
}

bool CombinedKittiReader::next(){
    static ros::Time last_time=ros::TIME_MIN;
    for( Readers::iterator it = readers_.begin(); it!=readers_.end(); ) {
        if( ! (*it)->ok() )
            it = readers_.erase(it);
        else
            ++it;
    }

    if( readers_.empty() ) {
        ok_ = false;
        return false;
    }

    std::sort(readers_.begin(), readers_.end(), data_reader_time_compare2);
    readers_.front()->next();
    std::sort(readers_.begin(), readers_.end(), data_reader_time_compare2);

    time_ = readers_.front()->time();
    if( time_ < last_time )
        ROS_WARN("negative time change");
    last_time = time_;
    ok_ = true;
    return true;

}

stdr_velodyne::PointCloudPtr CombinedKittiReader::instantiateVelodyneSpins(){
    if (vlf_reader_.ok()){
        return vlf_reader_.instantiateVelodyneSpins();
    } else {
        return stdr_velodyne::PointCloudPtr();
    }

}

#define FUNC_BODY(T, fn) return readers_.empty() ? T::ConstPtr() : readers_.front()->fn()

stdr_msgs::ApplanixPose::ConstPtr CombinedKittiReader::instantiateApplanixPose() const
{
   if (loggz_reader_.ok()){
    return loggz_reader_.instantiateApplanixPose();
    } else {
      return stdr_msgs::ApplanixPose::Ptr();
    }
}

stdr_msgs::ApplanixGPS::ConstPtr CombinedKittiReader::instantiateApplanixGPS() const
{
    return loggz_reader_.instantiateApplanixGPS();
}

stdr_msgs::ApplanixRMS::ConstPtr CombinedKittiReader::instantiateApplanixRMS() const
{
    return loggz_reader_.instantiateApplanixRMS();
}

velodyne_msgs::VelodyneScan::ConstPtr CombinedKittiReader::instantiateVelodyneScans() const
{
    return vlf_reader_.instantiateVelodyneScans();
}



}
