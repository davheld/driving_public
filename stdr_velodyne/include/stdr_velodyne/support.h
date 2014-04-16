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

#ifndef __STDR_VELODYNE__VELO_SUPPORT_H__
#define __STDR_VELODYNE__VELO_SUPPORT_H__

#include <stdr_msgs/RawScans.h>

/** \brief Functions and data structures to read velodyne data
 *
 * The velodyne spits raw strings of bytes. Each of these contains 12
 * measurements. A measurements contains 32 points.
 *
 * The VLF file format is made of a sequence of those raw strings of bytes
 * plus headers and checksum.
 */

namespace stdr_velodyne
{


/** \brief Packets sent by the velodyne are made of 12 Measurements as a string
 * of bytes:
 *
 * data layout: <measurement 0> ... <measurement 11> status
 * where each measurement is:
 *   blockid encoder range0 intensity0 ... range11 intensity11
 *   with:
 *     blockid   : unsigned short (2 bytes)
 *     encoder   : unsigned short (2 bytes)
 *     range     : unsigned short (2 bytes)
 *     intensity : unsigned char  (1 byte)
 *     --> total length: 3*32 + 4 = 100
 * status is made of 6 bytes (unknown meaning)
 *
 * Hence the total length is 12 * 100 + 6 = 1206
 */
namespace raw
{

typedef uint8_t Byte;

/// A single measurement (one block, 32 points)
struct Measurement
{
  static const uint16_t UPPER_BLOCK;
  static const uint16_t LOWER_BLOCK;
  static const unsigned NPOINTS = 32;

  uint16_t encoder;
  uint16_t block;
  uint16_t range[NPOINTS];
  uint8_t  intensity[NPOINTS];
};

/// Velodyne spits data packet per packet, 12 Measurements per packet
struct MeasurementPacket
{
  // number of bytes to represent a MeasurementPacket
  static const unsigned short NBYTES;

  /// number of measurements in packet
  static const unsigned NUM_MEASUREMENTS = 12;

  Measurement measurements[NUM_MEASUREMENTS];
  uint8_t     status[6];

  void parseBytes(const Byte *);
};

}

// ---------------------------------------------------------------------------


/// The different file types we can handle
typedef enum { UNKNOWN, PCAP, VLF } VELODYNE_FILE_TYPE;

struct VelodyneFile
{
  VELODYNE_FILE_TYPE    format;
  FILE*                 fp;
  std::string           filename;
  int                   buffer_len;
  raw::Byte *           msg_buffer;
  // variables that change infrequently
  int                   sweep_number;

  VelodyneFile() : fp(NULL), msg_buffer(NULL) { }
};

struct StampedPacket : public raw::MeasurementPacket
{
  double timestamp;
  stdr_msgs::RawScans toRawScans() const;
  void toRawScans(stdr_msgs::RawScans *) const;
};

/// Opens a PCAP file
VelodyneFile openPcapFile(const std::string& filename);

/// Opens a VLF file
VelodyneFile openVlfFile(const std::string& filename);

/// Opens a file, detecting which format to use based on extension
VelodyneFile openFile(const std::string& filename);

void pcapReadPacket(VelodyneFile & velodyne, StampedPacket* pkt);


/** VLF format
 *
 * data layout: sequence of vlf packets
 *    startbyte  timestamp length MeasurementPacket checksum
 * where:
 *    startbyte: 0x1b (1 byte)
 *    timestamp: double (8 bytes)
 *    length:    uint16_t (2 bytes): number of bytes in the MeasurementPacket, must be 1206
 *    checksum:  uint8_t (1 byte): the checksum of the MeasurementPacket
 */
namespace vlf
{
/// magic number marking the beginning of a raw packet
static const uint8_t START_BYTE = 0x1b;
static const unsigned NBYTES = sizeof(raw::MeasurementPacket) + 12;
void readPacket(VelodyneFile & velodyne, StampedPacket* pkt);
void stream(const stdr_msgs::RawScans &, std::ostream &);
}

inline void readPacket(VelodyneFile & velodyne, StampedPacket* pkt)
{
  if (velodyne.format == PCAP) pcapReadPacket(velodyne, pkt);
  else vlf::readPacket(velodyne, pkt);
}

/** Handles accumulating packets and forming the RawScans messages */
class PacketCollector
{
public:

  /** Constructor
   *
   * @param[in] bundle_sz: how many packets to accumulate before
   * the RawScans message is ready.
   */
  explicit PacketCollector(unsigned bundle_sz = 100);

  /** Add a packet.
   *
   * Returns the RawScans message when it's ready, a NULL ptr otherwise.
   */
  stdr_msgs::RawScans::Ptr addPacket(const StampedPacket & pkt);

private:
  stdr_msgs::RawScans::Ptr raw_scans_;
  unsigned bundle_sz_;
  unsigned msg_counter_;
};



} // namespace stdr_velodyne


#endif // __STDR_VELODYNE__VELO_SUPPORT_H__
