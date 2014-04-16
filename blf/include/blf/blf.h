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


#ifndef __BLF__BLF_H__
#define __BLF__BLF_H__

/* See http://www.gnu.org/software/libc/manual/html_node/Opening-Streams.html
 * and http://www.gnu.org/software/libc/manual/html_node/File-Positioning.html
 * For notes on how to portably open 64-bit streams on 32-bit machines
 */
#define _FILE_OFFSET_BITS 64

#include <cstdio>
#include <vector>
#include <stdr_lib/exception.h>

namespace blf
{

namespace ex
{
struct ExceptionBase : virtual stdr::ex::ExceptionBase { };
struct BadTimestamp : virtual ExceptionBase { };
struct BadChecksum : virtual ExceptionBase { };
}

extern const unsigned char START_BYTE;

struct Packet
{
  unsigned short id;
  double timestamp;
  std::vector<unsigned char> data;
};

class BLF
{
public:
  BLF();
  ~BLF();

  void open(const char *filename, const char *mode, bool async = true);

  void close();

  /* reading */

  void read_data(unsigned short *id, double *timestamp,
    std::vector<unsigned char> *data);

  void read_pkt(Packet *pkt);

  /// Reads until the start byte is found, then read the header
  void start_partial_read(unsigned short *id, double *timestamp,
      unsigned int *len);

  /// Read the data (must be called after start_partial_read)
  void partial_read(std::vector<unsigned char> *data);

  /// Computes and verify the checksum (must be called after partial_read)
  void finish_partial_read();

  /// Aborts a partial read and places the file pointer at the start of the next
  /// block (must be called after start_partial_read)
  void abort_partial_read();

  /// Resets the file pointer at the begining of the current block (must be
  /// called after start_partial_read)
  void reset_partial_read();

  void seek(long long offset, int whence);

  /// Reads until a timestamp >=t is found
  void seek_timestamp(double t);

  void tell(long long * pos);

  void rewind();

  /* writing */

  void write_data(const std::vector<unsigned char> & data,
    double timestamp, unsigned short id);

  void write_pkt(const Packet &pkt);

  void start_partial_write(unsigned int len, double timestamp,
        unsigned short id);

  void partial_write(const std::vector<unsigned char> & data);

  void finish_partial_write();

private:
  enum status_t         { UNINITIALIZED, READ, WRITE };

  FILE                  * fp_;
  status_t              status_;
  unsigned int          partial_bytes_left_;
  unsigned int          partial_bytes_read_;
  bool                  partial_read_in_progress_;
  bool                  partial_write_in_progress_;
  unsigned char         partial_chksum_;

  bool                  found_first_timestamp_;
  double                first_timestamp_;
};



/* BLF Index files */

struct IndexEntry
{
  int id;
  off_t offset;
  double timestamp;
};

struct Index
{
  std::vector<IndexEntry> block;

  bool load(const char* filename);
  unsigned seek_nearest_timestamp(double timestamp) const;
};

} // namespace blf

#endif //__BLF__BLF_H__
