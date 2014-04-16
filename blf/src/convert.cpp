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


#include <cstring>
#include <blf/convert.h>

namespace blf
{

uint16_t uchar2ushort(unsigned char * bytes)
{
  uint16_t i = 256 * bytes[1] + bytes[0];
  return i;
}

void ushort2uchar(uint16_t i, unsigned char * bytes)
{
  memcpy(bytes, &i, 2);
}

uint32_t uchar2uint(unsigned char *bytes)
{
  uint32_t i;
  memcpy(&i, bytes, 4);
  return i;
}

void uint2uchar(uint32_t i, unsigned char * bytes)
{
  memcpy(bytes, &i, 4);
}

uint64_t uchar2ulong(unsigned char * bytes)
{
  uint64_t l;
  memcpy(&l, bytes, 8);
  return l;
}

void ulong2uchar(uint64_t l, unsigned char * bytes)
{
  memcpy(bytes, &l, 8);
}

double uchar2double(unsigned char *bytes)
{
  double v;
  memcpy(&v, bytes, 8);
  return v;
}

void double2uchar(double d, unsigned char *bytes)
{
  memcpy(bytes, &d, 8);
}

unsigned short twoByteLoHiShort(unsigned char a, unsigned char b)
{
  unsigned int r = (unsigned short)b * 256 + (unsigned short)a;
  return r;
}

unsigned short twoByteHiLoShort(unsigned char a, unsigned char b)
{
  unsigned int r = (unsigned short)a * 256 + (unsigned short)b;
  return r;
}

short twoByteLoHiSignedShort(unsigned char a, unsigned char b)
{
  unsigned int r = (unsigned short)b * 256 + (unsigned short)a;
  return r;
}

short twoByteHiLoSignedShort(unsigned char a, unsigned char b)
{
  unsigned int r = (unsigned short)a * 256 + (unsigned short)b;
  return r;
}

unsigned int convertBytes2UInt8(unsigned char *bytes)
{
  unsigned int i;
  memcpy(&i, bytes, 1);
  return i;
}

int convertBytes2Int16(unsigned char *bytes)
{
  int i;
  memcpy(&i, bytes, 2);
  return i;
}

unsigned int convertBytes2UInt16(unsigned char *bytes)
{
  unsigned int i;
  memcpy(&i, bytes, 2);
  return i;
}

unsigned int convertRevBytes2UInt16(unsigned char *bytes)
{
  unsigned int i;
  unsigned char b[2];
  b[1] = *bytes; b[0] = *(bytes + 1);
  memcpy(&i, b, 2);
  return i;
}

} //namespace blf
