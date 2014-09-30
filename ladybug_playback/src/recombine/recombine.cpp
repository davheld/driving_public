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

#include <dc1394/conversions.h>

#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>

#include <timer/timer.h>
#include <ladybug_playback/recombine.h>


// save individual bayer channel images (debug)
#define SAVE_INTERMEDIATE_IMAGES 0

#if SAVE_INTERMEDIATE_IMAGES
#include <fstream>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#endif



namespace ladybug_playback
{

const unsigned Recombiner::FULL_WIDTH = 1232;
const unsigned Recombiner::FULL_HEIGHT = 1616;
const unsigned Recombiner::W2 = Recombiner::FULL_WIDTH/2;
const unsigned Recombiner::H2 = Recombiner::FULL_HEIGHT/2;

Recombiner::Recombiner()
  : frame_count_(0)
  , debayer_(false)
  , debayer_alg_(DC1394_BAYER_METHOD_HQLINEAR)
{
  for(int i = 0; i < 24; i++) {
    fj_[i] = fastjpeg::init_decompress();

    // These are rotated, so h and w are flipped on purpose
    bayer_image_[i].width = Recombiner::FULL_HEIGHT / 2;
    bayer_image_[i].height = Recombiner::FULL_WIDTH / 2;
    bayer_image_[i].nchannels = 1;
    bayer_image_[i].pix.resize(bayer_image_[i].width * bayer_image_[i].height * bayer_image_[i].nchannels);
  }

  for(int i = 0; i <6; ++i) {
    combined_images_[i].resize(Recombiner::FULL_HEIGHT * Recombiner::FULL_WIDTH);
    selector_[i] = true;
  }
}

Recombiner::~Recombiner()
{
  for(int i = 0; i < 24; i++)
    fastjpeg::release( fj_+i );
}

std::vector<sensor_msgs::Image::Ptr>
Recombiner::operator() (const stdr_msgs::LadybugImages & raw_images)
{
  std::vector<sensor_msgs::Image::Ptr> bayer_images;
  recombine(raw_images, bayer_images);
  return bayer_images;
}

// Decompress the images and place the pixels in the recombined image.
// Take care of camera that were excluded (size of data is 0)
bool Recombiner::decompress4(const stdr_msgs::LadybugImages & raw_images,
                             unsigned c4)
{
  //HighResTimer timer;
  //timer.reset("decompress"); timer.start();

  for( unsigned j = 0; j < 4; j ++)
  {
    const unsigned J = c4+j;
    if( raw_images.images[J].data.empty() )
      return false;

    fastjpeg::decompress_memory( fj_[J],
                                 &(raw_images.images[J].data[0]),
                                 raw_images.images[J].data.size(),
                                 bayer_image_+J,
                                 NULL );

#if SAVE_INTERMEDIATE_IMAGES
    // save JPEG data
    {
      const std::string name = (boost::format("frame%04d_compressed_%02d.jpg") % frame_count_ % J).str();
      std::ofstream f(name.c_str());
      f.write((const char *) &(raw_images.images[J].data[0]), raw_images.images[J].data.size());
    }

    // save the decompressed image (and show it)
    {
      cv::Mat m(W2, H2, CV_8UC1, &(bayer_image_[J].pix[0]));
      const std::string name = (boost::format("frame%04d_decompressed_%02d.bmp") % frame_count_ % J).str();
      cv::imwrite(name, m);
      //cv::imshow("frame", m);
      //cv::waitKey();
    }
#endif
  }

  //timer.stop(); std::cout <<timer.reportMilliseconds() <<std::endl;

  return true;
}

// Rotate and combine the 4 images to create one image with the bayer
// pattern. The images come as RGGB from the sensor. However, since we are
// rotating the image, we are creating a GRBG pattern
void Recombiner::combine(unsigned c4, unsigned char *data)
{
  //timer.reset("combine"); timer.start();

  // we will read the bayer images in their natural order
  for( unsigned y = 0; y < W2; ++y ) {
    for( unsigned x = 0; x < H2; ++x ) {

      // index in the individual images
      const unsigned j = y * H2 + x;
      //ROS_ASSERT(j < H2 * W2);

      // index in the bayer image
      //   *2 at the end is because we fill 2 rows and 2 cols at the time
      const unsigned i = (x * FULL_WIDTH + (W2-1 - y)) * 2;
      //ROS_ASSERT(i + FULL_WIDTH + 1 < FULL_HEIGHT * FULL_WIDTH);

      data[i] = bayer_image_[c4+2].pix[j];
      data[i+1] = bayer_image_[c4+0].pix[j];
      data[i + FULL_WIDTH] = bayer_image_[c4+3].pix[j];
      data[i + FULL_WIDTH + 1] = bayer_image_[c4+1].pix[j];
    }
  }

  //timer.stop(); std::cout <<timer.reportMilliseconds() <<std::endl;

#if SAVE_INTERMEDIATE_IMAGES
  //save bayer image
  cv::Mat m(FULL_HEIGHT, FULL_WIDTH, CV_8UC1, &(data[0]));
  const std::string name = (boost::format("frame%04d_bayer_%02d.bmp") % frame_count_ % (c4/4)).str();
  cv::imwrite(name, m);
#endif

}

void Recombiner::recombine(const stdr_msgs::LadybugImages & raw_images,
                           std::vector<sensor_msgs::Image::Ptr> & bayer_images)
{
  ROS_ASSERT( raw_images.images.size() == 24 );
  bayer_images.resize(6);

#if !SAVE_INTERMEDIATE_IMAGES
#pragma omp parallel for
#else
  std::cout <<"timestamp: " <<std::setprecision(std::numeric_limits<double>::digits10) <<raw_images.header.stamp.toSec() <<std::endl;
  std::cout <<"Saving frame " <<frame_count_ <<std::endl;
#endif
  for( unsigned cam=0; cam<6; ++cam )
  {
    const unsigned c4 = cam * 4;
    if( !selector_[cam] )
      continue;


    const bool valid_camera_img = decompress4(raw_images, c4);


    // Prepare the output
    // Make sure the image is properly defined and allocated
    if( !valid_camera_img ) {
      bayer_images[cam].reset();
      continue;
    }

    if( ! bayer_images[cam] )
      bayer_images[cam].reset( new sensor_msgs::Image );

    sensor_msgs::Image & img = *(bayer_images[cam]);
    img.header = raw_images.header;
    std::stringstream ss;
    ss << "/ladybug/camera" <<cam;
    img.header.frame_id = ss.str();

    img.height = FULL_HEIGHT;
    img.width = FULL_WIDTH;
    img.is_bigendian = false;
    if( debayer_ ) {
      img.encoding = sensor_msgs::image_encodings::RGB8;
      img.step = img.width * 3;
    }
    else {
      img.encoding = sensor_msgs::image_encodings::BAYER_GRBG8;
      img.step = img.width;
    }
    img.data.resize(img.height * img.step);

    // select the output of the recombining stage
    std::vector<unsigned char> & data = debayer_ ? combined_images_[cam] : img.data;


    combine(c4, &(data[0]));


    if( debayer_ ) {
      // TODO: implement debayering with dc1394
      dc1394_bayer_decoding_8bit( &(combined_images_[cam][0]), &(img.data[0]),
                                  FULL_WIDTH, FULL_HEIGHT,
                                  DC1394_COLOR_FILTER_GRBG,
                                  debayer_alg_);

#if SAVE_INTERMEDIATE_IMAGES
      //save debayered image
      cv::Mat mrgb(FULL_HEIGHT, FULL_WIDTH, CV_8UC3, &(img.data[0]));
      cv::Mat mbgr(FULL_HEIGHT, FULL_WIDTH, CV_8UC3);
      cv::cvtColor(mrgb, mbgr, CV_RGB2BGR);
      const std::string name = (boost::format("frame%04d_final_%02d.bmp") % frame_count_ % (c4/4)).str();
      cv::imwrite(name, mbgr);
      cv::imshow("frame", mbgr);
      cv::waitKey();
#endif
    }
  }

  ++frame_count_;
}

} //namespace ladybug_playback
