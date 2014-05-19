#ifndef TRACK_MANAGER_H
#define TRACK_MANAGER_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <float.h>
#include <ros/assert.h>
#include <Eigen/Eigen>

#include <sensor_msgs/PointCloud.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <dgc_transform/dgc_transform.h>


#define POINTCLOUD_SERIALIZATION_VERSION 0
#define TRACK_SERIALIZATION_VERSION 2
#define TRACKMANAGER_SERIALIZATION_VERSION 2
#define FRAME_SERIALIZATION_VERSION 0

namespace track_manager { 

  class Frame {
  private:
    //! For caching of getCentroid() call.
    boost::shared_ptr<Eigen::Vector3f> centroid_;
    //! For caching of getBoundingBox() call.
    //! bounding_box_.col(0) are the small x and y coords; .col(1) are the large.
    boost::shared_ptr<Eigen::MatrixXf> bounding_box_;
    //! For caching of estimateSpinOffset().
    double spin_offset_;
    
  public:
    int serialization_version_;
    //! Assumed to be stored in smooth coordinates!
    boost::shared_ptr<sensor_msgs::PointCloud> cloud_;
    //! dgc format timestamp.
    double timestamp_;
    //! x, y, z, roll, pitch, yaw.
    dgc_transform::dgc_pose_t robot_pose_;
    //! smooth_frame_point^T * smooth_to_velo_ = velo_frame_point.
    Eigen::Matrix4f smooth_to_velo_;

    Frame(boost::shared_ptr<sensor_msgs::PointCloud> cloud,
          double timestamp,
          dgc_transform::dgc_pose_t robot_pose,
          dgc_transform::dgc_transform_t velodyne_offset);
    Frame(std::istream& istrm, dgc_transform::dgc_transform_t velodyne_offset);
    void serialize(std::ostream& out) const;
    bool deserialize(std::istream& istrm);
    Eigen::Matrix4f getSmoothToVeloTransform(dgc_transform::dgc_transform_t velodyne_offset) const;
    //! Returns false if there were no points.  eig is filled with numpts rows and 4 cols (homogeneous coords).
    bool getCloudInVeloCoords(Eigen::MatrixXd* eig) const;
    bool operator!=(const Frame& fr);
    bool operator==(const Frame& fr);
    void getVelodyneXYZ(dgc_transform::dgc_transform_t velodyne_offset, double* x, double* y, double* z) const;
    Eigen::Vector3f getCentroid();
    Eigen::MatrixXf getBoundingBox();
    double getDistance(dgc_transform::dgc_transform_t velodyne_offset);
    double estimateAdjustedTimestamp();
    //! Returns a number between 0 and 1 estimating how far into the spin this frame was observed.
    double estimateSpinOffset();
  };
 
  class Track {
  public:
    int serialization_version_;
    std::string label_;
    // double[4][4].
    dgc_transform::dgc_transform_t velodyne_offset_;
    std::vector< boost::shared_ptr<Frame> > frames_;
    
    //! Initializes with label == "unknown", and that's it.
    Track();
    Track(std::istream& istrm);
    Track(const std::string& label,
          const dgc_transform::dgc_transform_t& velodyne_offset,
          const std::vector< boost::shared_ptr<Frame> >& frames);

    //! Reserves space in the vectors of velo centers, timestamps, and clouds.
    void setVelodyneOffset(const dgc_transform::dgc_transform_t& velodyne_offset);
    void reserve(size_t num);
    void insertFrame(boost::shared_ptr<sensor_msgs::PointCloud> cloud,
                     double timestamp,
                     dgc_transform::dgc_pose_t robot_pose);
    bool operator==(const Track& tr);
    bool operator!=(const Track& tr);
    void serialize(std::ostream& out) const;
    bool deserialize(std::istream& istrm);
    //! Returns false if timestamp < timestamps_.front() || timestamp > timestamps_.back(), otherwise fills *idx with the index of the cloud with the nearest timestamp.
    bool seek(double timestamp, double max_time_difference, size_t* idx);
    //! Like seek, returns false if timestamp < timestamps_.front() || timestamp > timestamps_.back().  Otherwise, fills *idx with the index of the cloud with nearest timestamp less than timestamp, and fills *interpolation with a value in [0, 1] indicating how much to weight cloud *idx+1 vs cloud *idx.
    bool interpolatedSeek(double timestamp, double max_time_difference, size_t* idx, double* interpolation);
    double getMeanNumPoints() const;
    double getMeanDistance();
  };

  class TrackManager {
  public:
    int serialization_version_;
    std::vector< boost::shared_ptr<Track> > tracks_;

    //! Returns the maximum number of clouds in any track.
    size_t getMaxNumClouds() const;
    size_t getNumClouds() const;
    size_t getNumLabeledClouds() const;
    bool operator==(const TrackManager& tm);
    bool operator!=(const TrackManager& tm);
    bool save(const std::string& filename);
    bool load(const std::string& filename);
    void serialize(std::ostream& out);
    bool deserialize(std::istream& istrm);
    //! Put the tracks in descending order of track length.
    void sortTracks();
    //! Sort based on some other criteria.  Descending.
    void sortTracks(double (*rateTrack)(const Track&));
    void sortTracks(const std::vector<double>& track_ratings);
    void insertTrack(boost::shared_ptr<Track> track);
    void reserve(size_t size);
    void getFramesNear(double timestamp, double tol,
		       std::vector< boost::shared_ptr<Frame> >* frames,
		       std::vector<std::string>* class_names,
		       std::vector<int>* track_ids) const;
    
    TrackManager();
    TrackManager(const std::string& filename);
    TrackManager(std::istream& istrm);
    TrackManager(const std::vector< boost::shared_ptr<Track> >& tracks);
  };

  void serializePointCloud(const sensor_msgs::PointCloud& cloud, std::ostream& out);
  bool deserializePointCloud(std::istream& istrm, sensor_msgs::PointCloud* cloud);
  bool cloudsEqual(const sensor_msgs::PointCloud& c1, const sensor_msgs::PointCloud& c2);
  bool streamTrack(std::string track_manager_filename, const Track& tr); 
  double getTrackLength(const Track& tr);
  bool deserializePointCloudROS(std::istream& istrm, sensor_msgs::PointCloud* cloud);
  void serializePointCloudROS(const sensor_msgs::PointCloud& cloud, std::ostream& out);
  void smpToPCL(const sensor_msgs::PointCloud& smp, pcl::PointCloud<pcl::PointXYZRGB>* pcd);
  
  // -- Useful functions for testing.
  void getRandomTransform(dgc_transform::dgc_transform_t trans);
  boost::shared_ptr<sensor_msgs::PointCloud> getRandomCloud();
  boost::shared_ptr<Frame> getRandomFrame(dgc_transform::dgc_transform_t velodyne_offset);
  boost::shared_ptr<Track> getRandomTrack();
  boost::shared_ptr<TrackManager> getRandomTrackManager();
  bool floatEq(float x, float y, int maxUlps = 5);
}


   
#endif //TRACK_MANAGER_H
