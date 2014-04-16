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


#include <cstdio>
#include <fnmatch.h> //filename regex matching

#include <stdr_lib/exception.h>
#include <stdr_velodyne/support.h>


#define FILE_VLF_EXT            ".vlf"
#define FILE_PCAP_EXT           ".pcap"
#define MAX_NAME_LENGTH       256


namespace stdr_velodyne
{

namespace raw
{

const uint16_t Measurement::UPPER_BLOCK = 0xeeff;
const uint16_t Measurement::LOWER_BLOCK = 0xddff;

const unsigned short MeasurementPacket::NBYTES = 1206;

#define READBYTES(dst) memcpy(&dst, bytes, sizeof(dst)); bytes+=sizeof(dst)

void MeasurementPacket::parseBytes(const Byte *bytes)
{
  for( unsigned i=0; i<NUM_MEASUREMENTS; ++i ) {
    READBYTES(measurements[i].block);
    READBYTES(measurements[i].encoder);
    for( unsigned j=0; j<raw::Measurement::NPOINTS; ++j ) {
      READBYTES(measurements[i].range[j]);
      READBYTES(measurements[i].intensity[j]);
    }
  }
  memcpy(&status, bytes, 6);
}

#undef READBYTES

} //namespace raw



VelodyneFile openPcapFile(const std::string & filename)
{
  uint8_t header[24];
  uint32_t magic;
  uint16_t major, minor;
  uint32_t sigfigs, snaplen, network;
  int zone;

  VelodyneFile velodyne_file;
  velodyne_file.filename = filename;

  velodyne_file.fp = fopen(filename.c_str(), "r");
  if (velodyne_file.fp == NULL)
    BOOST_THROW_EXCEPTION(stdr::ex::IOError() <<stdr::ex::MsgInfo(std::string("Could not open file ") + filename + " for reading."));

  if (fread(header, 24, 1, velodyne_file.fp) != 1) {
    fclose(velodyne_file.fp);
    BOOST_THROW_EXCEPTION(stdr::ex::IOError() <<stdr::ex::MsgInfo(std::string("Could not read header of file ") + filename + "."));
  }

  magic = *((uint32_t *) header);
  if (magic != 0xa1b2c3d4) {
    fclose(velodyne_file.fp);
    BOOST_THROW_EXCEPTION(stdr::ex::IOError() <<stdr::ex::MsgInfo(std::string("File ") + filename + " is not a pcap file."));
  }

  major = *((uint16_t*) (header + 4));
  minor = *((uint16_t*) (header + 6));
  zone = *((int *) (header + 8));
  sigfigs = *((uint32_t *) (header + 12));
  snaplen = *((uint32_t *) (header + 16));
  network = *((uint32_t *) (header + 20));

  velodyne_file.msg_buffer = new raw::Byte[snaplen+1];
  velodyne_file.sweep_number = 0;
  return velodyne_file;
}

void pcapReadPacket(VelodyneFile & velodyne_file, StampedPacket* pkt)
{
  uint8_t header[16];
  uint32_t sec, usec;
  uint32_t len1, len2;

  /* read pcap packet header */
  if (fread(header, 16, 1, velodyne_file.fp) != 1)
    BOOST_THROW_EXCEPTION(stdr::ex::IOError() <<stdr::ex::MsgInfo("Could not read log message."));
  sec = *((uint32_t*) header);
  usec = *((uint32_t*) (header + 4));
  pkt->timestamp = sec + usec / 1e6;
  len1 = *((uint32_t*) (header + 8));
  len2 = *((uint32_t*) (header + 12));

  /* read pcap packet data */
  if (fread(velodyne_file.msg_buffer, len1, 1, velodyne_file.fp) != 1)
    BOOST_THROW_EXCEPTION(stdr::ex::IOError() <<stdr::ex::MsgInfo("Could not read log message."));

  /* skip over the IP and UDP header and checksum*/
  //data = velodyne_file->msg_buffer + 40 + 4;
  raw::Byte * data = velodyne_file.msg_buffer + 40 + 2;

  pkt->parseBytes(data);
}

VelodyneFile openVlfFile(const std::string& filename)
{
  VelodyneFile velodyne_file;
  velodyne_file.filename = filename;

  velodyne_file.fp = fopen(filename.c_str(), "r");
  if (velodyne_file.fp == NULL)
    BOOST_THROW_EXCEPTION(stdr::ex::IOError() <<stdr::ex::MsgInfo(std::string("Could not open file ") + filename + " for reading."));

  velodyne_file.buffer_len = 2*vlf::NBYTES; // why times 2?
  velodyne_file.msg_buffer = new raw::Byte[velodyne_file.buffer_len];
  velodyne_file.sweep_number = 0;

  // start reading until the first scan is found:
  // search for the VLF_START_BYTE, followed 8 bytes later by the correct number of scans
  // TODO: is it really necessary, or can we assume that vlf files are well formed...
  // TODO: instead of returning an invalid VelodyneFile upon failure, shouldn't
  // we throw an exception instead?
  char data[16];
  uint16_t len;
  raw::Byte n;
  long pos;
  while( true )
  {
    // search for the start byte, record it position in the file
    do {
      if( feof(velodyne_file.fp) ) return velodyne_file;
      pos = ftell( velodyne_file.fp );
      n = fgetc(velodyne_file.fp);
    } while(n != vlf::START_BYTE);

    // read 8 bytes
    if( fread(data, 8, 1, velodyne_file.fp) != 1 )
      return velodyne_file;

    // read 2 bytes that should encode the number of scans
    if( fread(data, 2, 1, velodyne_file.fp) != 1 )
      return velodyne_file;
    memcpy(&len, data, 2);

    // test that the number of scans is correct
    if(len == raw::MeasurementPacket::NBYTES) {
      fseek(velodyne_file.fp, pos, SEEK_SET);
      return velodyne_file;
    }
    else {
      // keep searching, starting one byte after the start byte we found earlier
      fseek(velodyne_file.fp, pos, SEEK_SET);
      fgetc(velodyne_file.fp);
    }
  }
  return velodyne_file;
}


//=============================================================================
// Brice Dec 2013
// Some old functions that I was too lazy to rewrite


#include <sys/resource.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <dirent.h>
#include <wordexp.h>

bool dgc_file_exists(const std::string& filename)
{
  FILE *fp = fopen64(filename.c_str(), "r");
  if (fp == NULL)
    return false;
  fclose(fp);
  return true;
}

std::string dgc_complete_filename(const std::string& filename, const std::string& extension)
{
  if (dgc_file_exists(filename)
      && filename.length() >= extension.length()
      && filename.substr(filename.length() - extension.length()).compare(extension)==0 )
  {
    return filename;
  }

  std::string dirname, name;
  const char* p = strrchr(filename.c_str(), '/');
  if (p != NULL) {
    name = p + 1;
    dirname = filename.substr(0, filename.length() - name.length());
  }
  else {
    dirname = ".";
    name = filename;
  }

  struct dirent **namelist;
  const int n = scandir(dirname.c_str(), &namelist, NULL, NULL);

  int last_match = -1, count = 0;
  for (int i = 0; i < n; i++) {
    if (strlen(namelist[i]->d_name) >= name.length()
        && strncmp(namelist[i]->d_name, name.c_str(), name.length()) == 0
        && strlen(namelist[i]->d_name) >= extension.length()
        && extension.compare(namelist[i]->d_name + strlen(namelist[i]->d_name) - extension.length()) == 0)
    {
      count++;
      last_match = i;
    }
  }

  std::string new_filename; // the result

  if (count == 1) {
    new_filename = dirname;
    new_filename += '/';
    new_filename += namelist[last_match]->d_name;
  }

  if (n > 0) {
    for (int i = 0; i < n; i++)
      free(namelist[i]);
    free(namelist);
  }

  return new_filename;
}

//=============================================================================


VelodyneFile openFile(const std::string & filename)
{
  VelodyneFile velodyne_file;
  VELODYNE_FILE_TYPE inp_type = UNKNOWN;
  char fname[4 * MAX_NAME_LENGTH];
  std::string completed_filename;

  if (!fnmatch("pcap:*", filename.c_str(), 0)) {
    fprintf(stderr, "# INFO: use pcap file type!\n");
    strncpy(fname, &(filename[5]), MAX_NAME_LENGTH);
    inp_type = PCAP;
  }
  else if (!fnmatch("vlf:*", filename.c_str(), 0)) {
    fprintf(stderr, "# INFO: read vlf file type!\n");
    strncpy(fname, &(filename[4]), MAX_NAME_LENGTH);
    inp_type = VLF;
  }
  else if (!fnmatch("*" FILE_PCAP_EXT, filename.c_str(), 0)) {
    fprintf(stderr, "# INFO: read pcap file type!\n");
    strncpy(fname, filename.c_str(), MAX_NAME_LENGTH);
    inp_type = PCAP;
  }
  else if (!fnmatch("*" FILE_VLF_EXT, filename.c_str(), 0)) {
    fprintf(stderr, "# INFO: read vlf file type!\n");
    strncpy(fname, filename.c_str(), MAX_NAME_LENGTH);
    inp_type = VLF;
  }
  else if ( (completed_filename = dgc_complete_filename(filename, FILE_VLF_EXT)).length()>0 ) {
    ROS_ASSERT(completed_filename.length()<MAX_NAME_LENGTH);
    strncpy(fname, completed_filename.c_str(), MAX_NAME_LENGTH);
    inp_type = VLF;
  }
  else if ( (completed_filename = dgc_complete_filename(filename, FILE_PCAP_EXT)).length()>0 ) {
    ROS_ASSERT(completed_filename.length()<MAX_NAME_LENGTH);
    strncpy(fname, completed_filename.c_str(), MAX_NAME_LENGTH);
    inp_type = PCAP;
  }

  switch (inp_type) {
    case PCAP:
      velodyne_file = openPcapFile(fname);
      velodyne_file.format = PCAP;
      break;

    case VLF:
      velodyne_file = openVlfFile(fname);
      velodyne_file.format = VLF;
      break;

    default:
      BOOST_THROW_EXCEPTION(stdr::ex::IOError() <<stdr::ex::MsgInfo("Unknown file type"));
  }
  return velodyne_file;
}


namespace vlf {

raw::Byte computeChecksum(const raw::Byte * bytes)
{
  raw::Byte c = 0;
  for (size_t i = 0; i < raw::MeasurementPacket::NBYTES; ++i)
    c += *(bytes++);
  return c;
}

void readPacket(VelodyneFile & velodyne_file, StampedPacket *pkt)
{
  raw::Byte data[16];
  uint16_t len;

  if (fgetc(velodyne_file.fp) != START_BYTE)
    BOOST_THROW_EXCEPTION(stdr::ex::IOError() <<stdr::ex::MsgInfo("# ERROR: wrong start byte."));

  if (fread(data, 8, 1, velodyne_file.fp) != 1)
    BOOST_THROW_EXCEPTION(stdr::ex::IOError() <<stdr::ex::MsgInfo("Could not read time stamp."));
  memcpy(&(pkt->timestamp), data, 8);

  if (fread(data, 2, 1, velodyne_file.fp) != 1)
    BOOST_THROW_EXCEPTION(stdr::ex::IOError() <<stdr::ex::MsgInfo("Could not read packet length."));

  memcpy(&(len), data, 2);
  if (len != raw::MeasurementPacket::NBYTES)
    BOOST_THROW_EXCEPTION(stdr::ex::IOError() <<stdr::ex::MsgInfo("# ERROR: packet has wrong size."));

  if (fread(velodyne_file.msg_buffer, len, 1, velodyne_file.fp) != 1)
    BOOST_THROW_EXCEPTION(stdr::ex::IOError() <<stdr::ex::MsgInfo("Read error."));

  pkt->parseBytes(velodyne_file.msg_buffer);

  raw::Byte c;
  if (fread(&c, 1, 1, velodyne_file.fp) != 1)
    BOOST_THROW_EXCEPTION(stdr::ex::IOError() <<stdr::ex::MsgInfo("Could not read checksum."));

  if( c!=computeChecksum(velodyne_file.msg_buffer) )
    BOOST_THROW_EXCEPTION(stdr::ex::IOError() <<stdr::ex::MsgInfo("Checksum mismatch."));
}


#define WRITE_BYTES(dst, val) memcpy(dst, &val, sizeof(val)); dst+=sizeof(val)

void stream(const stdr_msgs::RawScans & scans, std::ostream & os)
{
  ROS_ASSERT(scans.scans.size() % raw::MeasurementPacket::NUM_MEASUREMENTS==0);
  raw::Byte bytes[vlf::NBYTES];
  raw::Byte status[6]; bzero(status, 6);
  for( unsigned scan_idx=0; scan_idx<scans.scans.size(); scan_idx+=12 )
  {
    double timestamp = scans.scans[scan_idx].stamp.toSec();
    raw::Byte * b = bytes;
    WRITE_BYTES(b, START_BYTE);
    WRITE_BYTES(b, timestamp);
    WRITE_BYTES(b, raw::MeasurementPacket::NBYTES);

    raw::Byte * mb = b;
    for( int n=0; n<12; ++n ) {
      const stdr_msgs::RawScan & scan = scans.scans[scan_idx+n];
      ROS_ASSERT(timestamp == scan.stamp.toSec());

      if( scan.block_id==stdr_msgs::RawScan::BLOCK_LOWER ) {
        WRITE_BYTES(b, raw::Measurement::LOWER_BLOCK);
      }
      else {
        WRITE_BYTES(b, raw::Measurement::UPPER_BLOCK);
      }

      WRITE_BYTES(b, scan.encoder);
      for( int j=0; j<raw::Measurement::NPOINTS; ++j ) {
        WRITE_BYTES(b, scan.range[j]);
        WRITE_BYTES(b, scan.intensity[j]);
      }
    }
    memcpy(b, status, 6); b+=6;
    raw::Byte c = computeChecksum(mb);
    WRITE_BYTES(b, c);
    os.write(reinterpret_cast<const char*>(bytes), sizeof(bytes));
  }
}

#undef WRITE_BYTES

} //namespace vlf



stdr_msgs::RawScans StampedPacket::toRawScans() const
{
  stdr_msgs::RawScans scans;
  toRawScans(&scans);
  return scans;
}

void StampedPacket::toRawScans(stdr_msgs::RawScans * scans) const
{
  stdr_msgs::RawScan raw_scan;

  scans->scans.clear();
  scans->scans.reserve(raw::MeasurementPacket::NUM_MEASUREMENTS);
  scans->header.stamp = ros::Time(timestamp);
  scans->header.frame_id = "velodyne";

  for (unsigned i = 0; i < raw::MeasurementPacket::NUM_MEASUREMENTS; i++) {
    const raw::Measurement & measurement = measurements[i];
    raw_scan.encoder = measurement.encoder;
    raw_scan.stamp = scans->header.stamp;

    if( measurement.block==raw::Measurement::LOWER_BLOCK )
      raw_scan.block_id = stdr_msgs::RawScan::BLOCK_LOWER;
    else if( measurement.block==raw::Measurement::UPPER_BLOCK )
      raw_scan.block_id = stdr_msgs::RawScan::BLOCK_UPPER;
    else
      BOOST_THROW_EXCEPTION(stdr::ex::IOError() <<stdr::ex::MsgInfo("Undefined block code in raw measurement"));

    for (unsigned j = 0; j < raw::Measurement::NPOINTS; j++) {
      raw_scan.intensity[j] = measurement.intensity[j];
      raw_scan.range[j] = measurement.range[j];
    }

    scans->scans.push_back(raw_scan);
  }
}


PacketCollector::PacketCollector(unsigned bundle_sz)
: bundle_sz_(bundle_sz), msg_counter_(0)
{

}

stdr_msgs::RawScans::Ptr PacketCollector::addPacket(const StampedPacket & pkt)
{
  stdr_msgs::RawScans::Ptr ret_scans;
  if( ! raw_scans_ ) {
    raw_scans_ = stdr_msgs::RawScans::Ptr(new stdr_msgs::RawScans);
    raw_scans_->scans.reserve(bundle_sz_*raw::MeasurementPacket::NUM_MEASUREMENTS);
    raw_scans_->header.frame_id = "velodyne";
  }

  ros::Time pkt_time(pkt.timestamp);

  stdr_msgs::RawScan raw_scan;
  for (unsigned i = 0; i < raw::MeasurementPacket::NUM_MEASUREMENTS; i++) {
    const raw::Measurement & measurement = pkt.measurements[i];
    raw_scan.encoder = measurement.encoder;
    raw_scan.stamp = pkt_time;

    if( measurement.block==raw::Measurement::LOWER_BLOCK )
      raw_scan.block_id = stdr_msgs::RawScan::BLOCK_LOWER;
    else if( measurement.block==raw::Measurement::UPPER_BLOCK )
      raw_scan.block_id = stdr_msgs::RawScan::BLOCK_UPPER;
    else
      BOOST_THROW_EXCEPTION(stdr::ex::IOError() <<stdr::ex::MsgInfo("Undefined block code in raw measurement"));

    for (unsigned j = 0; j < raw::Measurement::NPOINTS; j++) {
      raw_scan.intensity[j] = measurement.intensity[j];
      raw_scan.range[j] = measurement.range[j];
    }

    raw_scans_->scans.push_back(raw_scan);
  }

  if (raw_scans_->scans.size() == bundle_sz_*raw::MeasurementPacket::NUM_MEASUREMENTS) {
    raw_scans_->header.stamp = pkt_time;
    raw_scans_->header.seq = msg_counter_++;
    ret_scans = raw_scans_;
    raw_scans_.reset();
  }

  return ret_scans;
}

} //namespace stdr_velodyne
