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

/* This is a program that generates a point cloud with a structured intensity
 * pattern. This is used to test the intensity display in rviz.
 */

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <angles/angles.h>
#include <sensor_msgs/PointCloud2.h>
#include <velodyne_pointcloud/point_types.h>
#include <stdr_velodyne/point_type.h>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "fake_velodyne_pcd", ros::init_options::AnonymousName);

  ros::NodeHandle nh;
  ros::Publisher pcd_pub_xyzir = nh.advertise< pcl::PointCloud< velodyne_pointcloud::PointXYZIR > >("points_xyzir", 100);
  ros::Publisher pcd_pub_stdr = nh.advertise<stdr_velodyne::PointCloud>("points_stdr", 100);

  std_msgs::Header header;
  header.frame_id = "velodyne";

  pcl::PointCloud< velodyne_pointcloud::PointXYZIR > points_xyzir;
  velodyne_pointcloud::PointXYZIR pt_xyzir;
  pt_xyzir.z = 0;

  stdr_velodyne::PointCloud points_stdr;
  stdr_velodyne::PointType pt_stdr;
  pt_stdr.z = 0;

  ros::Rate rate(10);
  double offset = 0;

  while( ros::ok() )
  {
    points_xyzir.clear();
    points_stdr.clear();

    for( unsigned i = 0; i < 3600; ++i ) { // 10 points per degree
      pt_stdr.intensity = pt_xyzir.intensity = double(i) / 3600 * 255;
      const double h_angle = offset + angles::from_degrees(double(i)/10);
      const double c = cos(h_angle);
      const double s = sin(h_angle);
      for( unsigned r=0; r<64; ++r ) {
        pt_stdr.x = pt_xyzir.x = r*c;
        pt_stdr.y = pt_xyzir.y = r*s;
        points_xyzir.push_back(pt_xyzir);
        points_stdr.push_back(pt_stdr);
      }
    }

    header.stamp = ros::Time::now();
    pcl_conversions::toPCL(header, points_xyzir.header);
    points_stdr.header = points_xyzir.header;
    pcd_pub_xyzir.publish(points_xyzir);
    pcd_pub_stdr.publish(points_stdr);

    offset = angles::normalize_angle(offset + angles::from_degrees(1));
    rate.sleep();
    ros::spinOnce();
  }

  return 0;
}
