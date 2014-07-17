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


#include <inttypes.h>

#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp>

#include <ros/ros.h>

#include <angles/angles.h>
#include <stdr_velodyne/config.h>
#include <stdr_lib/rosparam_helpers.h>

namespace stdr_velodyne {


unsigned getSpinStart()
{
  static boost::mutex get_spin_start_mutex_;
  static unsigned get_spin_start_value_;
  static bool first = true;

  boost::unique_lock<boost::mutex> lock(get_spin_start_mutex_);

  if(first)
  {
    ros::NodeHandle nh("/driving/velodyne");
    int spin_start;
    GET_ROS_PARAM_WARN(nh, "spin_start", spin_start, stdr_velodyne::NUM_TICKS / 2);
    ROS_ASSERT_MSG(spin_start>=0 && spin_start<stdr_velodyne::NUM_TICKS,
                   "Invalid value for rosparam /driving/velodyne/spin_start");
    get_spin_start_value_ = spin_start;
    first = false;
  }
  return get_spin_start_value_;
}


void AngleVal::fromDegrees(double v)
{
  fromRads(angles::from_degrees(v));
}

void AngleVal::fromRads(double v)
{
  angle_ = angles::normalize_angle(v);
  cos_ = ::cos(angle_);
  sin_ = ::sin(angle_);
}

double AngleVal::getDegrees() const
{
  return angles::to_degrees(angle_);
}



RingConfig::RingConfig()
  : rot_angle_(0), range_offset_(0), range_offsetX_(0),
    range_offsetY_(0), laser_enabled_(true), v_offset_(0), h_offset_(0)
{

}

float RingConfig::range2dist(uint16_t range) const
{
  return range * TICKS_TO_METER + range_offset_;
}

void RingConfig::project(float distance, PointType *p) const
{
  const AngleVal & e_angle = enc_rot_angle_[p->encoder];
  p->distance = distance;
  const double xyDistance = p->distance * vert_angle_.cos();
  // profiler says that too much time is spent retrieving e_angle.cos and e_angle.sin
  p->x = xyDistance * e_angle.cos() - h_offset_ * e_angle.sin();
  p->y = xyDistance * e_angle.sin() + h_offset_ * e_angle.cos();
  p->z = xyDistance / vert_angle_.cos() * vert_angle_.sin() + v_offset_;
}

void RingConfig::origin(uint16_t e, double *x, double *y, double *z) const
{
  const AngleVal & e_angle = enc_rot_angle_[e];
  const double xyDistance = range_offset_ * vert_angle_.cos();
  *x = xyDistance * e_angle.cos() - h_offset_ * e_angle.sin();
  *y = xyDistance * e_angle.sin() + h_offset_ * e_angle.cos();
  *z = xyDistance / vert_angle_.cos() * vert_angle_.sin() + v_offset_;
}

uint16_t RingConfig::h_angle_to_encoder(double h_angle) const
{
  static const double rad2enc = NUM_TICKS / (M_PI * 2);
  return (rot_angle_ - h_angle) * rad2enc;
}




Configuration::Ptr Configuration::static_configuration;

Configuration::Ptr Configuration::getStaticConfigurationInstance()
{
  if( ! static_configuration )
    static_configuration.reset(new Configuration);
  return static_configuration;
}

Configuration::Configuration()
  : valid_(false)
{
  for (unsigned i = 0; i < NUM_LASERS; i++)
    for (unsigned j = 0; j < 256; j++)
      intensity_map_[i][j] = j;
}

struct beam_angle_t
{
  double angle;
  unsigned idx;
};

bool operator< (const beam_angle_t& a, const beam_angle_t& b)
{
  return a.angle < b.angle;
}


void Configuration::recompute()
{
  std::vector<beam_angle_t> beam_angles(NUM_LASERS);

  static const double enc2rad = 2*M_PI/NUM_TICKS;

  for (unsigned i = 0; i < NUM_LASERS; i++) {
    RingConfig & rc = ring_config_[i];

    beam_angles[i].angle = rc.vert_angle_.getRads();
    beam_angles[i].idx = i;

    for (unsigned j = 0; j < NUM_TICKS; j++)
      rc.enc_rot_angle_[j].fromRads(rc.rot_angle_ - enc2rad*j);
  }

  // sort the beams from top to bottom
  std::sort(beam_angles.begin(), beam_angles.end()); //first from bottom to top
  std::reverse(beam_angles.begin(), beam_angles.end()); //then reverse

  for (unsigned i = 0; i < NUM_LASERS; i++) {
    hardware_indexes_[i] = beam_angles[i].idx;
    beam_numbers_[beam_angles[i].idx] = i;
  }

  //verify that we got things right, i.e. that rings numbers are sorted from
  //top to bottom.
  for(unsigned b=1; b<NUM_LASERS; ++b) {
    const double a_prev = getRingConfig(getHardwareIndex(b-1)).vert_angle_.getRads();
    const double a = getRingConfig(getHardwareIndex(b)).vert_angle_.getRads();
    ROS_ASSERT(a<a_prev);
  }

  v_angle_max_ = beam_angles[0].angle;
  const double v_angle_min = beam_angles[NUM_LASERS-1].angle;
  ROS_ASSERT(v_angle_min<v_angle_max_);
  v_angle_mult_ = (double)V_ANGLE_TO_BEAM_NB_RES_N / (v_angle_max_-v_angle_min);
  for(unsigned i=0, j=0; i<V_ANGLE_TO_BEAM_NB_RES_N; ++i) {
    const double a = v_angle_max_ - i / v_angle_mult_;
    if( j+1<NUM_LASERS &&
        fabs(angles::shortest_angular_distance(a,beam_angles[j].angle)) >
        fabs(angles::shortest_angular_distance(a,beam_angles[j+1].angle)) )
      ++j;
    v_angle_to_beam_number_table_[i] = j;
  }
}

#define MAX_LINE_LENGTH    512

void Configuration::readIntensity(const std::string& filename)
{
  FILE *iop = 0;
  if ((iop = fopen(filename.c_str(), "r")) == 0) {
    ROS_FATAL("could not open velodyne intensity calibration file %s", filename.c_str());
    ROS_BREAK(); //TODO: throw exception?
  }
  ROS_INFO("read velodyne intensity calibration file %s", filename.c_str());

  int min_intensity = 0;
  int max_intensity = 255;
  int dummy = fscanf(iop, "%d %d\n", &min_intensity, &max_intensity);

  for (unsigned i = 0; i < NUM_LASERS; i++) {
    for (unsigned j = 0; j < 256; j++) {
      float ival = 0.0f;
      dummy = fscanf(iop, "%f ", &ival);
      float expanded = (ival - min_intensity) / (max_intensity - min_intensity);
      if (expanded < 0.0f) expanded = 0.0f;
      if (expanded > 1.0f) expanded = 1.0f;

      // in the file they are organized by reverse beam order so we invert it here
      intensity_map_[NUM_LASERS -1 - i][j] = (unsigned char) (255 * expanded);
    }
  }
  fclose(iop);
  //printf("New min:%d    New max: %d\n", min_intensity, max_intensity);
}

void Configuration::readCalibration(const std::string& filename)
{
  FILE * iop;

  int linectr = 0;
  int id, enabled, int_val;
  float rcf, hcf, hoff, voff, dist, distX, distY, rm;

  char line[MAX_LINE_LENGTH];
  range_multiplier_ = 1.0;
  spin_start_ = 18000;

  if ((iop = fopen(filename.c_str(), "r")) == 0) {
    ROS_FATAL("could not open velodyne calibration file %s", filename.c_str());
    ROS_BREAK(); //TODO: throw exception?
  }
  ROS_INFO("read velodyne calibration file %s", filename.c_str());

  while( fgets(line, MAX_LINE_LENGTH, iop) ) {
    linectr++;

    // strip of leading whitespaces
    char *cp = line, *end=line+strlen(line)-1;
    for( ; cp<end && *cp==' '; cp++ );

    // strip the trailing new line
    if( *end=='\n' ) *end = '\0';

    // skip lines starting with a '#' (comments) or empty lines.
    if (strlen(cp)==0 || *cp == '#') continue;

    if (sscanf(line, "%d %f %f %f %f %f %f %f %d", &id, &rcf, &hcf, &dist, &distX, &distY, &voff, &hoff, &enabled) == 9)
    {
      if (id < 0 || id > 63) {
        ROS_ERROR("wrong id '%d' in line %d", id, linectr);
        fclose(iop);
        ROS_BREAK(); //TODO: throw exception?
      }
      ring_config_[id].rot_angle_ = angles::from_degrees(rcf);
      ring_config_[id].vert_angle_.fromDegrees(hcf);
      ring_config_[id].range_offset_ = dist;
      ring_config_[id].range_offsetX_ = distX;
      ring_config_[id].range_offsetY_ = distY;
      ring_config_[id].laser_enabled_ = enabled;
      ring_config_[id].v_offset_ = voff;
      ring_config_[id].h_offset_ = hoff;
    }
    else if (sscanf(line, "RANGE_MULTIPLIER %f", &rm) == 1)
        range_multiplier_ = rm;
    else if (sscanf(line, "SPIN_START %d", &int_val) == 1) {
        spin_start_ = int_val;
    }
    else {
      ROS_ERROR("error in line %d: %s", linectr, line);
      fclose(iop);
      ROS_BREAK(); //TODO: throw exception?
    }
  }

  fclose(iop);

  valid_ = true;
  recompute();
}

unsigned Configuration::v_angle_to_beam_number(double v_angle) const
{
  int n = (v_angle_max_-v_angle) * v_angle_mult_;
  if( n <= 0 ) n = 0;
  if( n >= V_ANGLE_TO_BEAM_NB_RES_N ) n = V_ANGLE_TO_BEAM_NB_RES_N - 1;
  return v_angle_to_beam_number_table_[n];
}

void Configuration::printCalibrationData() const
{
  int i;

  printf("\ndouble VELO_ROT_ANGLE2[64] = { \n");
  for (i = 0; i < 16; i++) {
    printf("                                    %12.9f,%12.9f,%12.9f,%12.9f,\n",
        angles::to_degrees(ring_config_[4 * i + 0].rot_angle_),
        angles::to_degrees(ring_config_[4 * i + 1].rot_angle_),
        angles::to_degrees(ring_config_[4 * i + 2].rot_angle_),
        angles::to_degrees(ring_config_[4 * i + 3].rot_angle_));
  }
  printf("                                  };\n");

  printf("double VELO_VERT_ANGLE2[64] = { \n");
  for (i = 0; i < 16; i++) {
    printf("                                    %12.9f,%12.9f,%12.9f,%12.9f,\n",
        ring_config_[4 * i + 0].vert_angle_.getDegrees(),
        ring_config_[4 * i + 1].vert_angle_.getDegrees(),
        ring_config_[4 * i + 2].vert_angle_.getDegrees(),
        ring_config_[4 * i + 3].vert_angle_.getDegrees());
  }
  printf("                                  };\n");

  printf("int VELO_RANGE_OFFSET2[64] = { \n");
  for (i = 0; i < 16; i++) {
    printf("                                    %3f,%3f,%3f,%3f,\n",
        ring_config_[4 * i + 0].range_offset_,
        ring_config_[4 * i + 1].range_offset_,
        ring_config_[4 * i + 2].range_offset_,
        ring_config_[4 * i + 3].range_offset_);
  }
  printf("                                  };\n");

  printf("char VELO_LASER_ENABLED2[64] = { \n");
  for (i = 0; i < 16; i++) {
    printf("                                    %2d,%2d,%2d,%2d,\n",
        ring_config_[4 * i + 0].laser_enabled_,
        ring_config_[4 * i + 1].laser_enabled_,
        ring_config_[4 * i + 2].laser_enabled_,
        ring_config_[4 * i + 3].laser_enabled_);
  }
  printf("                                  };\n");
}

} // namespace stdr_velodyne
