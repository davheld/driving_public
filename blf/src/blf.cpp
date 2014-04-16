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


/* See http://www.gnu.org/software/libc/manual/html_node/Opening-Streams.html
 * and http://www.gnu.org/software/libc/manual/html_node/File-Positioning.html
 * For notes on how to portably open 64-bit streams on 32-bit machines
 */
#define _FILE_OFFSET_BITS 64

#include <iostream>
#include <fstream>
#include <boost/iostreams/filtering_stream.hpp>
#include <boost/iostreams/filter/gzip.hpp>

#include <ros/ros.h>

#include <blf/convert.h>
#include <blf/blf.h>


namespace blf
{

const unsigned char START_BYTE = 0x27;
const unsigned INDEX_SIZE_STEP = 2000;


unsigned char compute_chksum(const std::vector<unsigned char> & data)
{
  unsigned char chk = 0;
  for(unsigned i = 0; i < data.size(); i++)
    chk += data[i];
  return chk;
}

bool Index::load(const char *filename)
{
  block.clear();

  const std::string index_filename = std::string(filename) + ".idx.gz";
  std::ifstream file;
  file.open(index_filename.c_str(), std::ios_base::in | std::ios_base::binary);
  if( !file )
    return false;
  
  boost::iostreams::filtering_istream stream;
  stream.push(boost::iostreams::gzip_decompressor());
  stream.push(file);

  std::string line;
  IndexEntry ie;
  while( std::getline(stream, line) )
  {
    const int n = sscanf(line.c_str(), "%d %lf %lld",
                         &(ie.id), &(ie.timestamp),
                         (long long int *) &(ie.offset));
    if(n == 3) {
      block.push_back(ie);
    }
    else {
      ROS_FATAL("BLF index format has changed. Please reindex file.");
      ROS_BREAK();
    }
  }

  return true;
}

BLF::BLF()
  : status_(UNINITIALIZED)
{
}

BLF::~BLF()
{
  close();
}

void BLF::open(const char *fname, const char *mode, bool async)
{
  if(status_ != UNINITIALIZED)
    close();

  if( strcasecmp(mode, "w")==0 && (strcasecmp(fname, "-")==0 ||
      strcasecmp(fname, "stdout")==0 || strcasecmp(fname, "/dev/stdout")==0) ) {
    fp_ = stdout;
    status_ = WRITE;
  } 
  else if( strcasecmp(mode, "r")==0 && (strcasecmp(fname, "-")==0 ||
      strcasecmp(fname, "stdin")==0 || strcasecmp(fname, "/dev/stdin")==0) ) {
    fp_ = stdin;
    status_ = READ;
  } 
  else {
    if(!strcasecmp(mode, "w")) {
      if((fp_ = fopen(fname, "w")) ==0 )
        BOOST_THROW_EXCEPTION(stdr::ex::IOError() << ::boost::errinfo_errno(errno) <<::boost::errinfo_file_name(fname));
      status_ = WRITE;
    }
    else if(!strcasecmp(mode, "r")) {
      fp_ = fopen(fname, "r");
      if(fp_ == NULL)
        BOOST_THROW_EXCEPTION(stdr::ex::IOError() << ::boost::errinfo_errno(errno) <<::boost::errinfo_file_name(fname));
      status_ = READ;
    } 
    else {
      BOOST_THROW_EXCEPTION(stdr::ex::IOError() << ::boost::errinfo_errno(errno) <<::boost::errinfo_file_name(fname));
    }
  }
  found_first_timestamp_ = false;
  partial_read_in_progress_ = false;
  partial_write_in_progress_ = false;
}

void BLF::close()
{
  if( fp_ != stdin && fp_ != stdout && fp_ != NULL ) {
    fclose(fp_);
    fp_ = NULL;
  }
  status_ = UNINITIALIZED;
}

void BLF::start_partial_read(unsigned short *id, double *timestamp,
    unsigned int *len)
{
  unsigned char buf[32];
  unsigned int  r;

  if(status_ != READ)
    BOOST_THROW_EXCEPTION(stdr::ex::LogicError() <<stdr::ex::MsgInfo("BLF file is not open for reading"));

  if(partial_read_in_progress_)
    BOOST_THROW_EXCEPTION(stdr::ex::LogicError() <<stdr::ex::MsgInfo("Partial read already in progress"));

  /* read byte by byte looking for a start byte */
  do {
    if(fread(buf, 1, 1, fp_) != 1) {
      if( feof(fp_) )
        BOOST_THROW_EXCEPTION(stdr::ex::EOFError());
      else
        BOOST_THROW_EXCEPTION(stdr::ex::IOError() <<::boost::errinfo_errno(errno));
    }
  } while(buf[0] != START_BYTE);

  /* read timestamp */
  r = fread(buf, 1, 8, fp_);
  if(r != 8) {
    if( feof(fp_) )
      BOOST_THROW_EXCEPTION(stdr::ex::EOFError());
    else
      BOOST_THROW_EXCEPTION(stdr::ex::IOError() <<::boost::errinfo_errno(errno));
  }
  *timestamp = uchar2double(buf);
  if(*timestamp < 0)
    BOOST_THROW_EXCEPTION(blf::ex::BadTimestamp());

  /* read id */
  r = fread(buf, 1, 2, fp_);
  if(r != 2) {
    if( feof(fp_) )
      BOOST_THROW_EXCEPTION(stdr::ex::EOFError());
    else
      BOOST_THROW_EXCEPTION(stdr::ex::IOError() <<::boost::errinfo_errno(errno));
  }
  *id = uchar2ushort(buf);

  /* read length */
  r = fread(buf, 1, 4, fp_);
  if(r != 4) {
    if( feof(fp_) )
      BOOST_THROW_EXCEPTION(stdr::ex::EOFError());
    else
      BOOST_THROW_EXCEPTION(stdr::ex::IOError() <<::boost::errinfo_errno(errno));
  }
  *len = uchar2uint(buf);

  partial_chksum_ = 0;
  partial_bytes_left_ = *len;
  partial_bytes_read_ = 0;
  partial_read_in_progress_ = true;

  if(!found_first_timestamp_) {
    first_timestamp_ = *timestamp;
    found_first_timestamp_ = true;
  }
}

void BLF::partial_read(std::vector<unsigned char> * data)
{
  unsigned int r;

  if(status_ != READ)
    BOOST_THROW_EXCEPTION(stdr::ex::LogicError() <<stdr::ex::MsgInfo("BLF file is not open for reading"));

  if(!partial_read_in_progress_)
    BOOST_THROW_EXCEPTION(stdr::ex::LogicError() <<stdr::ex::MsgInfo("Partial read has not been started"));

  if(data->size() > partial_bytes_left_) {
    boost::format fmt("%d bytes requested : only %d left in partial read");
    fmt % data->size() % partial_bytes_left_;
    BOOST_THROW_EXCEPTION(stdr::ex::LogicError() <<stdr::ex::MsgInfo(fmt.str()));
  }

  /* read actual data */
  std::vector<unsigned char> & data_ref = *data;
  r = fread(&data_ref[0], 1, data_ref.size(), fp_);
  if(r != data->size()) {
    if( feof(fp_) )
      BOOST_THROW_EXCEPTION(stdr::ex::EOFError());
    else
      BOOST_THROW_EXCEPTION(stdr::ex::IOError() <<::boost::errinfo_errno(errno));
  }

  partial_bytes_left_ -= data->size();
  partial_chksum_ += compute_chksum(*data);
}

void BLF::finish_partial_read()
{
  unsigned int r;
  unsigned char chksum;

  if(status_ != READ)
    BOOST_THROW_EXCEPTION(stdr::ex::LogicError() <<stdr::ex::MsgInfo("BLF file is not open for reading"));

  if(!partial_read_in_progress_)
    BOOST_THROW_EXCEPTION(stdr::ex::LogicError() <<stdr::ex::MsgInfo("Partial read has not been started"));

  if(partial_bytes_left_ > 0) {
    boost::format fmt("%d bytes leftover in partial read");
    fmt % partial_bytes_left_;
    BOOST_THROW_EXCEPTION(stdr::ex::LogicError() <<stdr::ex::MsgInfo(fmt.str()));
  }

  partial_read_in_progress_ = false;

  /* read the checksum */
  r = fread(&chksum, 1, 1, fp_);
  if(r != 1) {
    if( feof(fp_) )
      BOOST_THROW_EXCEPTION(stdr::ex::EOFError());
    else
      BOOST_THROW_EXCEPTION(stdr::ex::IOError() <<::boost::errinfo_errno(errno));
  }

  /* make sure the checksum is correct */
  if(chksum != partial_chksum_)
    BOOST_THROW_EXCEPTION(blf::ex::BadChecksum());
}

void BLF::abort_partial_read()
{
  if(status_ != READ)
    BOOST_THROW_EXCEPTION(stdr::ex::LogicError() <<stdr::ex::MsgInfo("BLF file is not open for reading"));

  if(!partial_read_in_progress_)
    BOOST_THROW_EXCEPTION(stdr::ex::LogicError() <<stdr::ex::MsgInfo("Partial read has not been started"));

  seek(partial_bytes_left_ + 1, SEEK_CUR);
  partial_read_in_progress_ = false;
}

void BLF::reset_partial_read()
{
  if(status_ != READ)
    BOOST_THROW_EXCEPTION(stdr::ex::LogicError() <<stdr::ex::MsgInfo("BLF file is not open for reading"));

  if(!partial_read_in_progress_)
    BOOST_THROW_EXCEPTION(stdr::ex::LogicError() <<stdr::ex::MsgInfo("Partial read has not been started"));

  seek(-partial_bytes_read_-15LL, SEEK_CUR);
  partial_read_in_progress_ = false;
}

void BLF::read_data(unsigned short *id, double *timestamp,
    std::vector<unsigned char> *data)
{
  unsigned int len;
  start_partial_read(id, timestamp, &len);
  data->resize(len);
  partial_read(data);
  finish_partial_read();
}

void BLF::read_pkt(Packet *pkt)
{
  read_data(&(pkt->id), &(pkt->timestamp), &(pkt->data));
}

void BLF::start_partial_write(unsigned int len, double timestamp,
    unsigned short id)
{
  unsigned char bytes[32];

  if(status_ != WRITE)
    BOOST_THROW_EXCEPTION(stdr::ex::LogicError() <<stdr::ex::MsgInfo("BLF file is not open for writing"));

  if(partial_write_in_progress_)
    BOOST_THROW_EXCEPTION(stdr::ex::LogicError() <<stdr::ex::MsgInfo("Cannot be called while partial write in progress"));

  /* start byte */
  bytes[0] = START_BYTE;

  /* timestamp */
  double2uchar(timestamp, bytes + 1);

  /* sensor local id */
  ushort2uchar(id, bytes + 9);

  /* data length */
  uint2uchar(len, bytes + 11);

  if( fwrite(bytes, 1, 15, fp_)<15 )
    BOOST_THROW_EXCEPTION(stdr::ex::IOError() <<::boost::errinfo_errno(errno));

  partial_chksum_ = 0;
  partial_bytes_left_ = len;
  partial_write_in_progress_ = true;
}

void BLF::partial_write(const std::vector<unsigned char> & data)
{
  if(status_ != WRITE)
    BOOST_THROW_EXCEPTION(stdr::ex::LogicError() <<stdr::ex::MsgInfo("BLF file is not open for writing"));

  if(!partial_write_in_progress_)
    BOOST_THROW_EXCEPTION(stdr::ex::LogicError() <<stdr::ex::MsgInfo("Partial write has not been started"));

  if(data.size() > partial_bytes_left_) {
    boost::format fmt("%d bytes requested : only %d left in partial write");
    fmt % data.size() % partial_bytes_left_;
    BOOST_THROW_EXCEPTION(stdr::ex::LogicError() <<stdr::ex::MsgInfo(fmt.str()));
  }

  if( fwrite(&data[0], 1, data.size(), fp_) < data.size() )
    BOOST_THROW_EXCEPTION(stdr::ex::IOError() <<::boost::errinfo_errno(errno));

  partial_bytes_left_ -= data.size();
  partial_chksum_ += compute_chksum(data);
}

void BLF::finish_partial_write()
{
  if(status_ != WRITE)
    BOOST_THROW_EXCEPTION(stdr::ex::LogicError() <<stdr::ex::MsgInfo("BLF file is not open for writing"));

  if(!partial_write_in_progress_)
    BOOST_THROW_EXCEPTION(stdr::ex::LogicError() <<stdr::ex::MsgInfo("Partial write has not been started"));

  if(partial_bytes_left_ > 0) {
    boost::format fmt("%d bytes leftover in partial write");
    fmt % partial_bytes_left_;
    BOOST_THROW_EXCEPTION(stdr::ex::LogicError() <<stdr::ex::MsgInfo(fmt.str()));
  }

  partial_write_in_progress_ = false;
  if( fwrite(&partial_chksum_, 1, 1, fp_)<1 )
    BOOST_THROW_EXCEPTION(stdr::ex::IOError() <<::boost::errinfo_errno(errno));
}

void BLF::write_data(const std::vector<unsigned char> & data,
    double timestamp, unsigned short id)
{
  start_partial_write(data.size(), timestamp, id);
  partial_write(data);
  finish_partial_write();
}

void BLF::write_pkt(const Packet & pkt)
{
  write_data(pkt.data, pkt.timestamp, pkt.id);
}

void BLF::seek_timestamp(double t)
{
  unsigned short int pkt_id;
  unsigned int pkt_len;
  double pkt_timestamp;

  if(status_ != READ)
    BOOST_THROW_EXCEPTION(stdr::ex::LogicError() <<stdr::ex::MsgInfo("BLF file is not open for reading"));

  rewind();

  if(found_first_timestamp_ && t < first_timestamp_)
    return;

  do {
    start_partial_read(&pkt_id, &pkt_timestamp, &pkt_len);

    if(pkt_timestamp > t) {
      reset_partial_read();
      break;
    }

    abort_partial_read();
  } while(pkt_timestamp < t);
}

void BLF::seek(long long offset, int whence)
{
  if(status_ != READ)
    BOOST_THROW_EXCEPTION(stdr::ex::LogicError() <<stdr::ex::MsgInfo("BLF file is not open for reading"));
  if(fseeko(fp_, offset, whence) != 0)
    BOOST_THROW_EXCEPTION(stdr::ex::IOError() <<::boost::errinfo_errno(errno));
}

void BLF::tell(long long * pos)
{
  if(status_ != READ)
    BOOST_THROW_EXCEPTION(stdr::ex::LogicError() <<stdr::ex::MsgInfo("BLF file is not open for reading"));
  off_t o = ftello(fp_);
  if( o==-1 )
    BOOST_THROW_EXCEPTION(stdr::ex::IOError() <<::boost::errinfo_errno(errno));
  *pos = o;
}

void BLF::rewind()
{
  if(status_ != READ)
    BOOST_THROW_EXCEPTION(stdr::ex::LogicError() <<stdr::ex::MsgInfo("BLF file is not open for reading"));
  ::rewind(fp_);
}

unsigned Index::seek_nearest_timestamp(double timestamp) const
{
  if( block.size() == 0)
    return 0;

  const double interpolated_position  =
      (timestamp - block.front().timestamp)
      / (block.back().timestamp - block.front().timestamp);
  const int interpolated_block  = floor(interpolated_position * block.size());

  return std::max(interpolated_block, 0);
}

} //namespace blf
