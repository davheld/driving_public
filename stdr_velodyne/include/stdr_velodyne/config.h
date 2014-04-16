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


#ifndef __STDR_VELODYNE__CONFIG_H__
#define __STDR_VELODYNE__CONFIG_H__

#include <boost/noncopyable.hpp>
#include <velodyne_pointcloud/rawdata.h>
#include <stdr_velodyne/point_type.h>


namespace stdr_velodyne {

static const unsigned NUM_LASERS = 64;
static const unsigned NUM_TICKS = 36000;
static const float TICKS_TO_METER = 0.002f;

/** Returns the encoder value of the first scan in a whole spin (Reads it from
  * rosparam /driving/velodyne/spin_start, aborts if not available).
  */
unsigned getSpinStart();


/// \brief A structure to hold an angle's value, its cos and sin.
class AngleVal
{
  double angle_; // in radians
  double sin_;
  double cos_;

public:
  AngleVal() : angle_(0), sin_(0), cos_(1) { }

  void fromDegrees(double v);
  void fromRads(double v);

  double getDegrees() const;
  double getRads() const { return angle_; }
  double cos() const { return cos_; }
  double sin() const { return sin_; }
};

/// \brief Configuration parameters for one ring
struct RingConfig
{
  /// the angular position of this beam with respect to the encoder's position
  double rot_angle_;

  /// lookup table for the true encoder value for this beam, given the global
  /// encoder value.
  AngleVal enc_rot_angle_[NUM_TICKS];

  /// vertical angle
  AngleVal vert_angle_;

  /// range offset
  double range_offset_;

  /// range offset X (unused)
  double range_offsetX_;

  /// range offset Y (unused)
  double range_offsetY_;

  /// whether this beam is enabled (used for instance to turn off broken beams)
  bool laser_enabled_;

  /// vertical offset
  double v_offset_;

  /// horizontal offset
  double h_offset_;


  /// Default constructor
  RingConfig();

  /// Gets the (x,y,z) coordinates from the polar coordinates
  void project(uint16_t range, PointType *p) const;
};


/** \brief Velodyne configuration parameters.
  *
  * Velodyne gives a set of values for those parameters. But we use the parameters
  * from CLAMS (Jesse Levinson). We load them from a file.
  *
  * Using this model we can correct the range and intensity, and also get
  * the (x,y,z) coordinates of each point.
  *
  * This class in non-copyable because the intended use is to create one static
  * instance, load the configuration parameters from file, and use that static
  * instance everywhere.
  */
class Configuration : boost::noncopyable
{
private:
  /// Default constructor. Private: use getStaticConfigurationInstance instead.
  Configuration();

public:
  typedef boost::shared_ptr<Configuration> Ptr;
  typedef boost::shared_ptr<Configuration const> ConstPtr;

  /// Gets the static configuration instance (creates it if necessary)
  static Ptr getStaticConfigurationInstance();

  /// Reads configuration data from file
  void readCalibration(const std::string& filename);

  /// Reads intensity data from file
  void readIntensity(const std::string& filename);

  void printCalibrationData() const;

  /// Transforms (block id and index is Scan) into a beam index in the [0-64] range.
  inline unsigned getBeamIndex(uint16_t header, unsigned i) const
  { return i + ((header==velodyne_rawdata::LOWER_BANK) ? 32 : 0); }

  /// Given a beam number (0 is top most), returns the corresponding index.
  inline int getBeamOrder(unsigned i) const { return beam_order_[i]; }

  /// Given a beam index (as in the Scan message), returns the corresponding beam number (0 is top most).
  inline int getInvBeamOrder(unsigned i) const { return inv_beam_order_[i]; }

  inline const RingConfig & getRingConfig(unsigned i) const { return ring_config_[i]; }

  /// Returns the true intensity corresponding to the raw intensity returned by beam indexed by i.
  inline unsigned correctIntensity(unsigned i, unsigned raw_val) const { return intensity_map_[inv_beam_order_[i]][raw_val]; }

private:
  /// The static configuration instance (see getStaticConfigurationInstance())
  static Ptr static_configuration;

  unsigned        spin_start_;
  double          range_multiplier_;
  RingConfig      ring_config_[NUM_LASERS];
  uint8_t         beam_order_[NUM_LASERS];
  uint8_t         inv_beam_order_[NUM_LASERS];
  uint8_t         intensity_map_[NUM_LASERS][256];

  void recompute();
};

typedef boost::shared_ptr< ::stdr_velodyne::Configuration> ConfigurationPtr;
typedef boost::shared_ptr< ::stdr_velodyne::Configuration const> ConfigurationConstPtr;

} // namespace stdr_velodyne

#endif //__STDR_VELODYNE__CONFIG_H__
