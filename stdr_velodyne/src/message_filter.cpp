#include <ros/ros.h>
#include <angles/angles.h>

#include <stdr_velodyne/message_filter.h>
#include <stdr_velodyne/config.h>


namespace stdr_velodyne
{

SpinCollector::SpinCollector()
: spin_start_( getSpinStart() )
, prev_encoder_(-1)
{

}

bool SpinCollector::isFirstPoint(const PointType & p)
{
  if( p.beam_id!=0 )
    return false;

  if( prev_encoder_ < spin_start_ && p.encoder >= spin_start_ )
    return true;

  if( prev_encoder_ > p.encoder && prev_encoder_ < spin_start_ + NUM_TICKS && p.encoder >= spin_start_ )
    return true;

  return false;
}

PointCloudPtr SpinCollector::add(const PointCloud & packet)
{
  PointCloudPtr spin( new PointCloud );
  if( add(packet, *spin) )
    return spin;
  return PointCloudPtr();
}

bool SpinCollector::add(const PointCloud & packet, PointCloud & spin)
{
  scopeLockWrite;
  bool complete = false;

  const double a1 = angles::from_degrees(packet.back().encoder*.01);
  const double a2 = angles::from_degrees(packet.front().encoder*.01);
  ROS_DEBUG_NAMED("SpinCollector",
                  "Received a partial spin (%d points, %f degrees)."
                  " Last point stamp=%f. Delay: %fms.",
                  packet.points.size(),
                  angles::to_degrees(angles::shortest_angular_distance(a2, a1)),
                  packet.back().timestamp,
                  (ros::Time::now().toSec()-packet.back().timestamp)*1000);

  for( unsigned i=0; i<packet.size(); ++i )
  {
    if( prev_encoder_>=0 ) {
      if( points_.empty() ) {
        if( isFirstPoint( packet[i] ) )
          points_.push_back(packet[i]);
      }
      else {
        if( isFirstPoint( packet[i] ) ) {
          spin.clear();
          std_msgs::Header h;
          h.frame_id = packet.header.frame_id;
          h.stamp.fromSec(points_.back().timestamp); // Set stamp with last stamp from scan
          spin.header = pcl_conversions::toPCL(h);
          spin.swap(points_);
          complete = true;
        }
        points_.push_back(packet[i]);
      }
    }
    if( packet[i].beam_id==0 )
      prev_encoder_ = packet[i].encoder;
  }
  if( complete )
    ROS_DEBUG_NAMED("SpinCollector", "Created a whole spin. Delay: %fms.",
                    (ros::Time::now()-pcl_conversions::fromPCL(spin.header).stamp).toSec()*1000);
  return complete;
}

} // namespace stdr_velodyne
