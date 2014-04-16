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

#ifndef __BLF__CONVERT_H__
#define __BLF__CONVERT_H__

#include <stdint.h>

namespace blf
{

uint16_t uchar2ushort(unsigned char *bytes);

void ushort2uchar(uint16_t i, unsigned char *bytes);

uint32_t uchar2uint(unsigned char *bytes);

void uint2uchar(uint32_t i, unsigned char *bytes);

uint64_t uchar2ulong(unsigned char * bytes);

void ulong2uchar(uint64_t l, unsigned char *bytes);

double uchar2double(unsigned char *bytes);

void double2uchar(double d, unsigned char *bytes);

unsigned short twoByteLoHiShort(unsigned char a, unsigned char b);

unsigned short twoByteHiLoShort(unsigned char a, unsigned char b);

short twoByteLoHiSignedShort(unsigned char a, unsigned char b);

short twoByteHiLoSignedShort(unsigned char a, unsigned char b);

unsigned int convertBytes2UInt8(unsigned char *bytes);

int convertBytes2Int16(unsigned char *bytes);

} //namespace blf

#endif //__BLF__CONVERT_H__
