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

#ifndef __LADYBUG_PLAYBACK__RECOMBINE__H__
#define __LADYBUG_PLAYBACK__RECOMBINE__H__

#include <dc1394/conversions.h>

#include <sensor_msgs/Image.h>

#include <fastjpeg/fastjpeg.h>
#include <stdr_msgs/LadybugImages.h>

namespace ladybug_playback
{

/** \brief A class to create the bayer image from the 4 independent,
 *  compressed, rotated and half size images that the ladybug produces.
 */
class Recombiner
{
public:
  /// The width and height of a ladybug image (after all image processing
  /// operations: decompression, rotation, demosaicing)
  static const unsigned FULL_WIDTH, FULL_HEIGHT;

public:
  Recombiner();

  bool debayer_; //default to false
  dc1394bayer_method_t debayer_alg_; //defaults to DC1394_BAYER_METHOD_HQLINEAR

  /// if selector_[i] is false then skip image i. Defaults to all true.
  bool selector_[6];

  ~Recombiner();

  /** \brief Creates the bayer coded raw images
    *
    * For each selected camera, performs decompression of each of the 4 bayer
    * channel images, rotates them, and recombines them to form the bayer coded
    * image.
    *
    * \param raw_images the compressed and separated images
    * \returns the resulting bayer coded raw images.
    */
  std::vector<sensor_msgs::Image::Ptr>
  operator() (const stdr_msgs::LadybugImages & raw_images);

  /** \brief Creates the bayer coded raw images from the separated compressed images.
    *
    * For each selected camera, performs decompression of each of the 4 bayer
    * channel images, rotates them, and recombines them to form the bayer coded
    * image.
    *
    * \param raw_images the compressed and separated images
    * \param bayer_images the resulting bayer coded raw images. Re-uses the memory
    * when possible.
    */
  void
  recombine(const stdr_msgs::LadybugImages & raw_images,
            std::vector<sensor_msgs::Image::Ptr> & bayer_images);

private:
  unsigned frame_count_; //used for saving the intermediate images to disk (debug)

  static const unsigned H2, W2;
  jpeg_decompress_struct *fj_[24];
  fastjpeg::Image bayer_image_[24];
  std::vector<unsigned char> combined_images_[6];

  bool decompress4(const stdr_msgs::LadybugImages & raw_images, unsigned c4);
  void combine(unsigned c4, unsigned char *data);
};

} //namespace ladybug_playback

#endif //__LADYBUG_PLAYBACK__RECOMBINE__H__
