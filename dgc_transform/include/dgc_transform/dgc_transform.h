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

#ifndef __DGC_TRANSFORM__DGC_TRANSFORM__H__
#define __DGC_TRANSFORM__DGC_TRANSFORM__H__

#include <tf/tf.h>


namespace dgc_transform
{

/** 3D transformation - can represent a combined translation and rotation */
typedef double dgc_transform_t[4][4];

/** Print a 3D transform as a 4x4 matrix
    @param t - transform to be printed
    @param str - name of transform, which will be printed above matrix
  */
void init(dgc_transform_t& t);

/** Initializes a transform as the identity transform */
void identity(dgc_transform_t t);

/** Left multiply transform t1 by transform t2 */
void left_multiply(dgc_transform_t t1, dgc_transform_t t2);

/** dest = left * src */
void left_multiply_nc(dgc_transform_t dest,
                      dgc_transform_t src,
                      dgc_transform_t left);

/** Rotate tranform around global x axis */
void rotate_x(dgc_transform_t t, double theta);

/** Rotate transform around global y axis */
void rotate_y(dgc_transform_t t, double theta);

/** Rotate transform around global z axis */
void rotate_z(dgc_transform_t t, double theta);

/** Add translation to transform */
void translate(dgc_transform_t t, double x, double y, double z);

/** Apply transform to 3D point */
inline void transform_point(double *x, double *y, double *z, dgc_transform_t t)
{
  double x2, y2, z2;

  x2 = t[0][0] * *x + t[0][1] * *y + t[0][2] * *z + t[0][3];
  y2 = t[1][0] * *x + t[1][1] * *y + t[1][2] * *z + t[1][3];
  z2 = t[2][0] * *x + t[2][1] * *y + t[2][2] * *z + t[2][3];
  *x = x2;
  *y = y2;
  *z = z2;
}

/** Copy transform src to dest */
void copy(dgc_transform_t dest, const dgc_transform_t src);

/** Read a transform from file */
int read(dgc_transform_t t, const char *filename);

/** Read a transform from a string */
int read_string(dgc_transform_t t, const char *str);

/** Get translation */
void get_translation(const dgc_transform_t t,
                     double *x, double *y, double *z);

/** Get rotation */
void get_rotation(const dgc_transform_t t,
                  double *x, double *y, double *z);

void rotate_rpy(dgc_transform_t dest, const dgc_transform_t src,
         double roll, double pitch, double yaw);

void inverse(const dgc_transform_t in, dgc_transform_t out);




tf::Transform as_tf(const dgc_transform_t & t);

/// Reads a transform from file. .tfm and .eig formats supported.
tf::Transform read(const std::string& filename);



/// Represents a pose (x, y, z and roll, pitch, yaw)
struct dgc_pose_t
{
  double x;
  double y;
  double z;
  double roll;
  double pitch;
  double yaw;
};


tf::Transform as_tf(const dgc_pose_t & t);

dgc_pose_t dgc_pose_from_tf(const tf::Transform & t);


} //namespace dgc_transform

#endif // __DGC_TRANSFORM__DGC_TRANSFORM__H__
