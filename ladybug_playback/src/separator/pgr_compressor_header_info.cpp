//=============================================================================
// Copyright ? 2004 Point Grey Research, Inc. All Rights Reserved.
// 
// This software is the confidential and proprietary information of Point
// Grey Research, Inc. ("Confidential Information").  You shall not
// disclose such Confidential Information and shall use it only in
// accordance with the terms of the license agreement you entered into
// with Point Grey Research, Inc. (PGR).
// 
// PGR MAKES NO REPRESENTATIONS OR WARRANTIES ABOUT THE SUITABILITY OF THE
// SOFTWARE, EITHER EXPRESSED OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
// PURPOSE, OR NON-INFRINGEMENT. PGR SHALL NOT BE LIABLE FOR ANY DAMAGES
// SUFFERED BY LICENSEE AS A RESULT OF USING, MODIFYING OR DISTRIBUTING
// THIS SOFTWARE OR ITS DERIVATIVES.
//=============================================================================


#include <cassert>
#include <cstdlib>
#include <cstring>

#include <ros/ros.h>

#include "pgr_compressor_header_info.h"


LadybugCompressorHeaderInfo::LadybugCompressorHeaderInfo()
  : m_arInfo(LADYBUG_COMPRESSOR_HEADER_MAX_IMAGES)
{
  m_iTotalSize = 0;
  m_iNumImages = 0;
  memset( &(m_arInfo[0]), 0x0, sizeof( ImageInfo ) * LADYBUG_COMPRESSOR_HEADER_MAX_IMAGES );
  m_pDataStart = NULL;
}

int
LadybugCompressorHeaderInfo::totalSize() const
{
  return m_iTotalSize;
}

int
LadybugCompressorHeaderInfo::images() const
{
  return m_iNumImages;
}

void
LadybugCompressorHeaderInfo::parse( const unsigned char*   pData)
{
  ROS_ASSERT( pData );
  
  m_iTotalSize = 0;
  m_iNumImages = 0;
  memset( &(m_arInfo[0]), 0x0, sizeof( ImageInfo ) * LADYBUG_COMPRESSOR_HEADER_MAX_IMAGES );
  
  m_pDataStart = pData;
  
  //
  // Revisit: the header at offset 16 is the same as LadybugImageInfo
  // version 2.
  //
  
  unsigned signature = 
    swab( *(unsigned*)&pData[ 16 + 0 ] );
  
  unsigned version  = swab( *(unsigned*)&pData[ 16 + 4 ] );
  //unsigned sequence = swab( *(unsigned*)&pData[ 16 + 16 ] );
  
  ROS_ASSERT( signature == 0xCAFEBABE || version == 2 );
  
  const unsigned* p = (unsigned*)(pData + 0x0340); //1024 - 24*8
  for( int i = 0; i < LADYBUG_COMPRESSOR_HEADER_MAX_IMAGES; i++ ) {
    unsigned offset = swab( *(p++) );
    unsigned size = swab( *(p++) );
    
    m_arInfo[ i ].offset = offset;
    m_arInfo[ i ].size   = size;

    if( (size==0 || offset==0) && i>=20 )
      continue; // the top image was not transmitted

    ROS_ASSERT( size!=0 );

    m_iNumImages++;

    if( offset + size > (unsigned)m_iTotalSize )
      m_iTotalSize = offset + size;

    m_arInfo[ i ].pData = pData + offset;

    m_arInfo[ i ].jpegCols =
        ( m_arInfo[ i ].pData[ 0x60 ] << 8 ) + m_arInfo[ i ].pData[ 0x61 ];

    m_arInfo[ i ].jpegRows =
        ( m_arInfo[ i ].pData[ 0x5E ] << 8 ) + m_arInfo[ i ].pData[ 0x5F ];
  }

  ROS_ASSERT(m_iNumImages>0);
}

 
const LadybugCompressorHeaderInfo::ImageInfo &
LadybugCompressorHeaderInfo::getInfo( int index ) const
{
  return m_arInfo[ index ];
}

