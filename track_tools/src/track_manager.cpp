#include <track_tools/track_manager.h>

using namespace std;
using namespace Eigen;

/************************************************************/
/******************** TrackManager ********************/
/************************************************************/
namespace track_manager { 

TrackManager::TrackManager() :
  serialization_version_(TRACKMANAGER_SERIALIZATION_VERSION),
  tracks_(vector< boost::shared_ptr<Track> >())
{
}

TrackManager::TrackManager(std::istream& istrm)  :
  serialization_version_(TRACKMANAGER_SERIALIZATION_VERSION),
  tracks_(vector< boost::shared_ptr<Track> >())
{
  bool success = deserialize(istrm);
  ROS_ASSERT(success);
}

TrackManager::TrackManager(const std::vector< boost::shared_ptr<Track> >& tracks)  :
  serialization_version_(TRACKMANAGER_SERIALIZATION_VERSION),
  tracks_(tracks)
{
}

TrackManager::TrackManager(const string& filename) :
  serialization_version_(TRACKMANAGER_SERIALIZATION_VERSION),
  tracks_(vector< boost::shared_ptr<Track> >())
{
  ifstream ifs(filename.c_str(), ios::in);
  bool success = deserialize(ifs);
  ROS_ASSERT(success);
}

bool TrackManager::save(const string& filename) {
  ofstream ofs(filename.c_str(), ios::out);
  serialize(ofs);
  ofs.close();
  return true;
}

bool TrackManager::load(const string& filename) {
  ifstream ifs(filename.c_str(), ios::in);
  const bool res = deserialize(ifs);
  ifs.close();
  return res;
}

size_t TrackManager::getMaxNumClouds() const {
  size_t max_num_clouds = 0;
  for(size_t i = 0; i < tracks_.size(); ++i) {
    if(tracks_[i]->frames_.size() > max_num_clouds)
      max_num_clouds = tracks_[i]->frames_.size();
  }
  return max_num_clouds;
}


size_t TrackManager::getNumClouds() const {
  size_t num = 0;
  for(size_t i = 0; i < tracks_.size(); ++i)
    num += tracks_[i]->frames_.size();
  return num;
}

size_t TrackManager::getNumLabeledClouds() const {
  size_t num = 0;
  for(size_t i = 0; i < tracks_.size(); ++i) {
    if(tracks_[i]->label_.compare("unlabeled") != 0)
      num += tracks_[i]->frames_.size();
  }
  return num;
}

void TrackManager::serialize(ostream& out) {
  out << "TrackManager" << endl;
  out << "serialization_version_" << endl;
  out << serialization_version_ << endl;
  //   out << "num_tracks" << endl;
  //   out << tracks_.size() << endl;
  for(size_t i=0; i<tracks_.size(); ++i) {
    tracks_[i]->serialize(out);
  }
}

bool TrackManager::deserialize(istream& istrm) {
  tracks_.clear();
  string line;

  getline(istrm, line);
  if(line.compare("TrackManager") != 0) {
    return false;
  }

  getline(istrm, line);
  if(line.compare("serialization_version_") != 0)
    return false;

  istrm >> serialization_version_;
  if(serialization_version_ != TRACKMANAGER_SERIALIZATION_VERSION) {
    cerr << "Expected TrackManager serialization_version_ == " << TRACKMANAGER_SERIALIZATION_VERSION;
    cerr << ".  This file is vs " << serialization_version_ << ", aborting." << endl;
    return false;
  }
  getline(istrm, line);

  while(true) {
    boost::shared_ptr<Track> tr(new Track());
    if(tr->deserialize(istrm))
      tracks_.push_back(tr);
    else
      break;
  }
  
  return true;
}

bool TrackManager::operator!=(const TrackManager& tm) {
  return !operator==(tm);
}

bool TrackManager::operator==(const TrackManager& tm) {
  if(serialization_version_ != tm.serialization_version_)
    return false;
  if(tracks_.size() != tm.tracks_.size())
    return false;
  for(size_t i=0; i<tracks_.size(); ++i) {
    if(*tracks_[i] != *tm.tracks_[i])
      return false;
  }
  return true;
}

double getTrackLength(const Track& tr) {
  double z = tr.getMeanNumPoints() + tr.frames_[0]->cloud_->points.size() + tr.frames_[0]->robot_pose_.roll + tr.frames_[0]->robot_pose_.pitch + tr.frames_[0]->robot_pose_.yaw;
  return (double) tr.frames_.size() + 1.0 / (1.0 + exp(-z / 10000.0)); // Break ties consistently with high probability.
}

void TrackManager::sortTracks() {
  sortTracks(&getTrackLength);
}

void TrackManager::sortTracks(double (*rateTrack)(const Track&)) {
  vector< pair<double, boost::shared_ptr<Track> > > length_idx(tracks_.size());
  for(size_t i=0; i<tracks_.size(); ++i) {
    length_idx[i].first = rateTrack(*tracks_[i]);
    length_idx[i].second = tracks_[i];
  }
  greater< pair<double, boost::shared_ptr<Track> > > emacs = greater< pair<double, boost::shared_ptr<Track> > >();
  sort(length_idx.begin(), length_idx.end(), emacs); //Descending.

  for(size_t i=0; i<tracks_.size(); ++i) {
    tracks_[i] = length_idx[i].second;
  }
}

void TrackManager::sortTracks(const vector<double>& track_ratings) {
  assert(track_ratings.size() == tracks_.size());
  vector< pair<double, boost::shared_ptr<Track> > > length_idx(tracks_.size());
  for(size_t i=0; i<tracks_.size(); ++i) {
    length_idx[i].first = track_ratings[i];
    length_idx[i].second = tracks_[i];
  }
  greater< pair<double, boost::shared_ptr<Track> > > emacs = greater< pair<double, boost::shared_ptr<Track> > >();
  sort(length_idx.begin(), length_idx.end(), emacs); //Descending.

  for(size_t i=0; i<tracks_.size(); ++i) {
    tracks_[i] = length_idx[i].second;
  }
}

void TrackManager::insertTrack(boost::shared_ptr<Track> track) {
  tracks_.push_back(track);
}

void TrackManager::reserve(size_t size) {
  tracks_.reserve(size);
}


void TrackManager::getFramesNear(double timestamp, double tol,
                                 std::vector< boost::shared_ptr<Frame> >* frames,
                                 std::vector<std::string>* class_names,
                                 std::vector<int>* track_ids) const
{
  assert(frames->empty());
  assert(class_names->empty());
  assert(track_ids->empty());

  frames->reserve(tracks_.size());
  class_names->reserve(tracks_.size());
  track_ids->reserve(tracks_.size());

  for(size_t i = 0; i < tracks_.size(); ++i) {
    size_t idx;
    bool valid = tracks_[i]->seek(timestamp, tol, &idx);
    if(valid) {
      frames->push_back(tracks_[i]->frames_[idx]);
      class_names->push_back(tracks_[i]->label_);
      track_ids->push_back(i);
    }
  }
}



/************************************************************/
/******************** Track ********************/
/************************************************************/

void Track::serialize(ostream& out) const {
  out << "Track" << endl;
  out << "serialization_version_" << endl;
  out << TRACK_SERIALIZATION_VERSION << endl;
  out << "label_" << endl;
  out << label_ << endl;
  out << "velodyne_offset_" << endl;
  for(int i = 0; i < 4; ++i) {
    for(int j = 0; j < 4; ++j) {
      out.write((char*)&velodyne_offset_[i][j], sizeof(double));
    }
  }
  out << endl;
  
  out << "num_frames_" << endl;
  out << frames_.size() << endl;
  for(size_t i=0; i<frames_.size(); ++i)
    frames_[i]->serialize(out);
}

bool Track::deserialize(istream& istrm) {
  if(istrm.eof())
    return false;

  long begin = istrm.tellg();
  frames_.clear();
  string line;

  getline(istrm, line);
  if(line.compare("Track") != 0) {
    istrm.seekg(begin);
    return false;
  }

  getline(istrm, line);
  if(line.compare("serialization_version_") != 0)
    return false;
  istrm >> serialization_version_;
  if(serialization_version_ != TRACK_SERIALIZATION_VERSION
     && serialization_version_ != 0) {
    cerr << "Track serialization version is wrong, aborting." << endl;
    return false;
  }
  getline(istrm, line);

  getline(istrm, line);
  if(line.compare("label_") != 0)
    return false;
  getline(istrm, label_);

  getline(istrm, line);
  if(line.compare("velodyne_offset_") != 0)
    return false;
  for(int i = 0; i < 4; ++i)
    for(int j = 0; j < 4; ++j)
      istrm.read((char*) &velodyne_offset_[i][j], sizeof(double));
  getline(istrm, line);

  getline(istrm, line);
  if(line.compare("num_frames_") != 0)
    return false;
  size_t num_frames = 0;
  istrm >> num_frames;
  getline(istrm, line);
  
  frames_.resize(num_frames);
  for(size_t i=0; i<num_frames; ++i) {
    assert(!frames_[i]);
    frames_[i] = boost::shared_ptr<Frame>(new Frame(istrm, velodyne_offset_));
  }
  
  return true;
}

bool Track::seek(double timestamp, double max_time_difference, size_t* idx) {
  assert(idx);
  
  if(timestamp < frames_.front()->timestamp_ || timestamp > frames_.back()->timestamp_)
    return false;

  double min_delta = FLT_MAX;
  size_t best_idx = 0;
  for(size_t i = 0; i < frames_.size(); ++i) {
    //double delta = fabs(frames_[i]->timestamp_ - timestamp);
    double delta = fabs(frames_[i]->estimateAdjustedTimestamp() - timestamp);
    if(delta < min_delta) {
      min_delta = delta;
      best_idx = i;
    }
  }

  if(min_delta <= max_time_difference) {
    *idx = best_idx;
    return true;
  }
  else {
    return false;
  }
}

bool Track::interpolatedSeek(double timestamp, double max_time_difference, size_t* idx, double* interpolation) {
  assert(idx);
  assert(interpolation);

  size_t nearest = 0;
  bool success = seek(timestamp, max_time_difference, &nearest);
  if(!success)
    return false;

  if(timestamp < frames_[nearest]->timestamp_)
    *idx = nearest - 1;
  else
    *idx = nearest;

  *interpolation = (timestamp - frames_[*idx]->timestamp_) / (frames_[*idx + 1]->timestamp_ - frames_[*idx]->timestamp_);
  
  return true;
}

double Track::getMeanNumPoints() const {
  double total = 0;
  for(size_t i = 0; i < frames_.size(); ++i) {
    total += frames_[i]->cloud_->points.size();
  }
  return total / (double)frames_.size();
}

double Track::getMeanDistance() {
  double total = 0;
  for(size_t i = 0; i < frames_.size(); ++i) {
    total += frames_[i]->getDistance(velodyne_offset_);
  }
  return total / (double)frames_.size();
}

void Track::setVelodyneOffset(const dgc_transform::dgc_transform_t& velodyne_offset) {
  for(int i = 0; i < 4; ++i)
    for(int j = 0; j < 4; ++j)
      velodyne_offset_[i][j] = velodyne_offset[i][j];
}

Track::Track(const std::string& label,
             const dgc_transform::dgc_transform_t& velodyne_offset,
             const std::vector< boost::shared_ptr<Frame> >& frames) :
  serialization_version_(TRACK_SERIALIZATION_VERSION),
  label_(label),
  frames_(frames)
{
  setVelodyneOffset(velodyne_offset);
}

Track::Track() :
  serialization_version_(TRACK_SERIALIZATION_VERSION),
  label_("unlabeled")
{
}

Track::Track(istream& istrm) :
  serialization_version_(TRACK_SERIALIZATION_VERSION),
  label_("unlabeled")
{
  long begin = istrm.tellg();
  istrm.seekg(begin);

  bool success = deserialize(istrm);
  ROS_ASSERT(success);
}

void Track::reserve(size_t num) {
  frames_.reserve(num);
}

void Track::insertFrame(boost::shared_ptr<sensor_msgs::PointCloud> cloud,
                        double timestamp,
                        dgc_transform::dgc_pose_t robot_pose)
{
  frames_.push_back(boost::shared_ptr<Frame>(new Frame(cloud, timestamp, robot_pose, velodyne_offset_)));
}

bool Track::operator!=(const Track& tr) {
  return !operator==(tr);
}

bool Track::operator==(const Track& tr) {
  if(tr.frames_.size() != frames_.size())
    return false;
  if(tr.label_.compare(label_) != 0)
    return false;
  for(size_t i=0; i<frames_.size(); ++i) {
    if(! (*frames_[i] == *tr.frames_[i])) {
      //cout << "Frame " << i << " differs." << endl;
      return false;
    }
  }
  return true;
}

/************************************************************/
/******************** Frame  ********************/
/************************************************************/

bool Frame::operator!=(const Frame& fr) {
  return !operator==(fr);
}

bool Frame::operator==(const Frame& fr) {
  if(!floatEq(timestamp_, fr.timestamp_)) {
    //cout << "Timestamps differ: " << timestamp_ << " " << fr.timestamp_ << endl;
    return false;
  }

  if(!floatEq(robot_pose_.x, fr.robot_pose_.x) ||
     !floatEq(robot_pose_.y, fr.robot_pose_.y) ||
     !floatEq(robot_pose_.z, fr.robot_pose_.z) ||
     !floatEq(robot_pose_.roll, fr.robot_pose_.roll) ||
     !floatEq(robot_pose_.pitch, fr.robot_pose_.pitch) ||
     !floatEq(robot_pose_.yaw, fr.robot_pose_.yaw)) {
    //cout << "Robot pose differs" << endl;
    //     cout << robot_pose_.x << " " << fr.robot_pose_.x << endl;
    //     cout << robot_pose_.y << " " << fr.robot_pose_.y << endl;
    //     cout << robot_pose_.z << " " << fr.robot_pose_.z << endl;
    //     cout << robot_pose_.roll << " " << fr.robot_pose_.roll << endl;
    //     cout << robot_pose_.pitch << " " << fr.robot_pose_.pitch << endl;
    //     cout << robot_pose_.yaw << " " << fr.robot_pose_.yaw << endl;
    return false;
  }
  
  if(!cloudsEqual(*cloud_, *fr.cloud_)) {
    //cout << "Clouds differ." << endl;
    return false;
  }

  return true;
}


Frame::Frame(boost::shared_ptr<sensor_msgs::PointCloud> cloud,
             double timestamp,
             dgc_transform::dgc_pose_t robot_pose,
             dgc_transform::dgc_transform_t velodyne_offset) :
  spin_offset_(-1),
  serialization_version_(FRAME_SERIALIZATION_VERSION),
  cloud_(cloud),
  timestamp_(timestamp),
  robot_pose_(robot_pose)
{
  smooth_to_velo_ = getSmoothToVeloTransform(velodyne_offset);
}

Frame::Frame(istream& istrm, dgc_transform::dgc_transform_t velodyne_offset) :
  spin_offset_(-1),
  serialization_version_(FRAME_SERIALIZATION_VERSION)
{
  assert(deserialize(istrm));
  assert(!centroid_);
  smooth_to_velo_ = getSmoothToVeloTransform(velodyne_offset);
}


bool Frame::deserialize(std::istream& istrm) {
  string line;
  getline(istrm, line);
  if(line.compare("Frame") != 0) {
    cout << "Expected 'Frame', got " << line << endl;
    return false;
  }
  
  getline(istrm, line);
  if(line.compare("serialization_version_") != 0) {
    cout << "Expected 'serialization_version_', got " << line << endl;
    return false;
  }
  istrm >> serialization_version_;
  if(serialization_version_ != FRAME_SERIALIZATION_VERSION) {
    cerr << "Frame serialization version is " << serialization_version_ << ", expected " << FRAME_SERIALIZATION_VERSION << ", aborting." << endl;
    return false;
  }
  getline(istrm, line);

  getline(istrm, line);
  if(line.compare("timestamp_") != 0) {
    cout << "Expected 'timestamp', got " << line << endl;
    return false;
  }
  istrm.read((char*)&timestamp_, sizeof(double));
  getline(istrm, line);


  getline(istrm, line);
  if(line.compare("robot_pose_") != 0) {
    cout << "Expected 'robot_pose_', got " << line << endl;
    return false;
  }
  istrm.read((char*)&robot_pose_.x, sizeof(double));
  istrm.read((char*)&robot_pose_.y, sizeof(double));
  istrm.read((char*)&robot_pose_.z, sizeof(double));
  istrm.read((char*)&robot_pose_.roll, sizeof(double));
  istrm.read((char*)&robot_pose_.pitch, sizeof(double));
  istrm.read((char*)&robot_pose_.yaw, sizeof(double));
  getline(istrm, line);

  cloud_ = boost::shared_ptr<sensor_msgs::PointCloud>(new sensor_msgs::PointCloud());
  cloud_->header.stamp = ros::Time(1); //Avoid a warning about timestamps from ROS.  We aren't using them anyway.
  bool success = deserializePointCloudROS(istrm, cloud_.get());
  return success;
}


void Frame::serialize(std::ostream& out) const{
  out << "Frame" << endl;
  out << "serialization_version_" << endl;
  out << FRAME_SERIALIZATION_VERSION << endl;
  out << "timestamp_" << endl;
  out.write((char*) &timestamp_, sizeof(double));
  out << endl;
  out << "robot_pose_" << endl;
  out.write((char*) &robot_pose_.x, sizeof(double));
  out.write((char*) &robot_pose_.y, sizeof(double));
  out.write((char*) &robot_pose_.z, sizeof(double));
  out.write((char*) &robot_pose_.roll, sizeof(double));
  out.write((char*) &robot_pose_.pitch, sizeof(double));
  out.write((char*) &robot_pose_.yaw, sizeof(double));
  out << endl;
  serializePointCloudROS(*cloud_, out);
  out << endl;
}
//     out << "timestamp" << endl;
//     out.write((char*)&timestamps_[i], sizeof(double));
//     out << endl;
//     assert(velodyne_centers_[i].size() == 3);
//     out << "velo_center" << endl;
//     out.write((char*)&velodyne_centers_[i][0], sizeof(float));
//     out.write((char*)&velodyne_centers_[i][1], sizeof(float));
//     out.write((char*)&velodyne_centers_[i][2], sizeof(float));
//     out << endl;
//     serializePointCloudROS(*clouds_[i], out);
//     out << endl;


bool Frame::getCloudInVeloCoords(Eigen::MatrixXd* eig) const {
  if(cloud_->points.size() < 1)
    return false;

  *eig = MatrixXd(cloud_->points.size(), 4);
  for(size_t i = 0; i < cloud_->points.size(); ++i) {
    eig->coeffRef(i, 0) = cloud_->points[i].x;
    eig->coeffRef(i, 1) = cloud_->points[i].y;
    eig->coeffRef(i, 2) = cloud_->points[i].z;
    eig->coeffRef(i, 3) = 1;
  }

  *eig = *eig * smooth_to_velo_.cast<double>();
  return true;
}

Matrix4f Frame::getSmoothToVeloTransform(dgc_transform::dgc_transform_t velodyne_offset) const {
  assert(!isnan(-robot_pose_.x));
  Matrix4f eig_smooth_to_rotated_body = Matrix4f::Identity();
  
  eig_smooth_to_rotated_body(3, 0) = -robot_pose_.x;
  eig_smooth_to_rotated_body(3, 1) = -robot_pose_.y;
  eig_smooth_to_rotated_body(3, 2) = -robot_pose_.z;

  dgc_transform::dgc_transform_t original;
  dgc_transform::dgc_transform_t inv;
  dgc_transform::rotate_rpy(original, velodyne_offset, robot_pose_.roll, robot_pose_.pitch, robot_pose_.yaw);
  dgc_transform::inverse(original, inv);

  Matrix4f eig_rotated_body_to_velodyne = Matrix4f::Identity();
  for(int r = 0; r < 4; ++r)
    for(int c = 0; c < 4; ++c)
      eig_rotated_body_to_velodyne(r, c) = inv[c][r]; // dgc uses T*pt, we're using pt*T.
  //   cout << "dgc rotated_body_to_velo " << endl;
  //   cout << eig_rotated_body_to_velodyne << endl;

  //  Matrix4f eig_rotated_body_to_body = Matrix4f::Identity();
  //   dgc_transform::dgc_transform_t transform;
  //   dgc_transform::dgc_transform_t id;
  //   dgc_transform::identity(id);
  //   dgc_transform::rotate_rpy(transform, id,
  // 		    -robot_pose_.roll,
  // 		    -robot_pose_.pitch,
  // 		    -robot_pose_.yaw);
  
  //   for(int r = 0; r < 4; ++r)
  //     for(int c = 0; c < 4; ++c)
  //       eig_rotated_body_to_body(r, c) = transform[c][r]; // dgc uses T*pt, we're using pt*T.

  //   Matrix4f eig_body_to_velodyne = Matrix4f::Identity();
  //   eig_body_to_velodyne(3, 0) = -velodyne_offset[0][3];
  //   eig_body_to_velodyne(3, 1) = -velodyne_offset[1][3];
  //   eig_body_to_velodyne(3, 2) = -velodyne_offset[2][3];
  //   cout << "eig rotated_body_to_velo" << endl;
  //   cout << eig_rotated_body_to_body * eig_body_to_velodyne << endl;
  //   cin.ignore();

  Matrix4f smooth_to_velo = eig_smooth_to_rotated_body * eig_rotated_body_to_velodyne;
  if(isnan(smooth_to_velo(0, 0))) {
    cout << eig_smooth_to_rotated_body << endl;
    cout << endl;
    cout << eig_rotated_body_to_velodyne << endl;
    assert(0);
  }
  return smooth_to_velo; //eig_rotated_body_to_body * eig_body_to_velodyne;
}

void Frame::getVelodyneXYZ(dgc_transform::dgc_transform_t velodyne_offset, double* x, double* y, double* z) const {
  // -- Get the velodyne center.
  dgc_transform::dgc_transform_t rotation;
  dgc_transform::dgc_transform_t id2;
  dgc_transform::identity(id2);
  dgc_transform::rotate_rpy(rotation, id2,
                                  robot_pose_.roll,
                                  robot_pose_.pitch,
                                  robot_pose_.yaw);

  double laser_x = velodyne_offset[0][3];
  double laser_y = velodyne_offset[1][3];
  double laser_z = velodyne_offset[2][3];
  dgc_transform::transform_point(&laser_x, &laser_y, &laser_z, rotation);
  laser_x += robot_pose_.x;
  laser_y += robot_pose_.y;
  laser_z += robot_pose_.z;

  *x = laser_x;
  *y = laser_y;
  *z = laser_z;
}

Vector3f Frame::getCentroid() {
  if(centroid_)
    return *centroid_;

  centroid_ = boost::shared_ptr<Vector3f>(new Vector3f());
  *centroid_ = Vector3f::Zero();
  for(size_t i = 0; i < cloud_->points.size(); ++i) {
    centroid_->coeffRef(0) += cloud_->points[i].x;
    centroid_->coeffRef(1) += cloud_->points[i].y;
    centroid_->coeffRef(2) += cloud_->points[i].z;
  }
  *centroid_ /= (double)cloud_->points.size();

  return *centroid_;
}


MatrixXf Frame::getBoundingBox() {
  if(bounding_box_)
    return *bounding_box_;

  bounding_box_ = boost::shared_ptr<MatrixXf>(new MatrixXf(2, 2));
  MatrixXf& bb = *bounding_box_;
  bb(0, 0) = FLT_MAX; // Small x.
  bb(1, 0) = FLT_MAX; // Small y.
  bb(0, 1) = -FLT_MAX; // Big x.
  bb(1, 1) = -FLT_MAX; // Big y.
  for(size_t i = 0; i < cloud_->points.size(); ++i) {
    double x = cloud_->points[i].x;
    double y = cloud_->points[i].y;
    if(x < bb(0, 0))
      bb(0, 0) = x;
    if(x > bb(0, 1))
      bb(0, 1) = x;
    if(y < bb(1, 0))
      bb(1, 0) = y;
    if(y > bb(1, 1))
      bb(1, 1) = y;
  }

  return *bounding_box_;
}

double Frame::getDistance(dgc_transform::dgc_transform_t velodyne_offset) {
  Vector3f centroid = getCentroid();
  Vector3d velo;
  getVelodyneXYZ(velodyne_offset, &velo(0), &velo(1), &velo(2));
  return (centroid.cast<double>() - velo).norm();
}

double Frame::estimateSpinOffset() {
  if(spin_offset_ != -1)
    return spin_offset_;

  if(cloud_->points.size() == 0)
    return 0;

  MatrixXd cloud;
  bool valid = getCloudInVeloCoords(&cloud);
  VectorXd mean = cloud.colwise().sum() / (double)cloud.rows();

  double x = mean(0);
  double y = mean(1);
  double angle = atan2(-y, -x);
  if(angle < 0)
    angle += 2.0 * M_PI;
  if(!(angle >= 0.0 && angle <= 2.0 * M_PI)) {
    cout << "npts: " << cloud.rows() << endl;
    cout << "valid: " << valid << endl;
    cout << cloud << endl;
    cout << "--- " << endl;
    cout << smooth_to_velo_ << endl;
    cout << "--- " << endl;
    cout << cloud_ << endl;
    cout << "angle: " << angle << endl;
    cout << "x: " << x << ", y: " << y << endl;
  }

  spin_offset_ = angle / (2.0 * M_PI);
  if(!(spin_offset_ >= 0.0 && spin_offset_ <= 1.0))
    cout << "spin_offset_: " << spin_offset_ << endl;

  return spin_offset_;
}

double Frame::estimateAdjustedTimestamp() {
  static const double spin_time = 0.1;
  const double adjusted_timestamp = timestamp_ + estimateSpinOffset() * spin_time;
  return adjusted_timestamp;
}

/************************************************************/
/******************** Helper Functions ********************/
/************************************************************/

void serializePointCloudROS(const sensor_msgs::PointCloud& cloud, ostream& out) {
  assert(sizeof(char*) == sizeof(uint8_t*));
  uint32_t serial_size = ros::serialization::serializationLength(cloud);
  boost::shared_array<uint8_t> buffer(new uint8_t[serial_size]);
  ros::serialization::OStream stream(buffer.get(), serial_size);
  ros::serialization::serialize(stream, cloud);

  out << "Cloud" << endl;
  out << "serialization_length" << endl;
  out << serial_size << endl;
  out.write((char*)buffer.get(), serial_size);
}

bool deserializePointCloudROS(std::istream& istrm, sensor_msgs::PointCloud* cloud) {
  string line;

  getline(istrm, line);
  if(line.compare("Cloud") != 0) {
    cout << "Expected 'Cloud', got " << line << endl;
    return false;
  }

  getline(istrm, line);
  if(line.compare("serialization_length") != 0) {
    cout << "Expected 'serialization_length', got " << line << endl;
    return false;
  }

  uint32_t serialization_length = 0;
  istrm >> serialization_length;
  getline(istrm, line);
  
  boost::shared_array<uint8_t> buffer(new uint8_t[serialization_length]);
  istrm.read((char*)buffer.get(), serialization_length);
  ros::serialization::IStream stream(buffer.get(), serialization_length);
  ros::serialization::Serializer<sensor_msgs::PointCloud>::read(stream, *cloud);
  getline(istrm, line);

  return true;
}


bool deserializePointCloud(istream& istrm, sensor_msgs::PointCloud* cloud) {
  string line;

  getline(istrm, line);
  if(line.compare("Cloud") != 0)
    return false;

  getline(istrm, line);
  if(line.compare("serialization_version_") != 0)
    return false;

  int serialization_version = 0;
  istrm >> serialization_version;
  if(serialization_version != POINTCLOUD_SERIALIZATION_VERSION)
    return false;
  getline(istrm, line);

  getline(istrm, line);
  if(line.compare("num_points") != 0)
    return false;

  size_t num_points = 0;
  istrm >> num_points;
  getline(istrm, line);
  
  getline(istrm, line);
  if(line.compare("points") != 0)
    return false;

  float* buf = (float*)malloc(sizeof(float)*num_points*3);
  istrm.read((char*)buf, sizeof(float)*num_points*3);
  cloud->points.resize(num_points);
  for(size_t i=0; i<num_points; ++i) {
    cloud->points[i].x = buf[i*3];
    cloud->points[i].y = buf[i*3+1];
    cloud->points[i].z = buf[i*3+2];
  }
  free(buf);
  return true;
}

void serializePointCloud(const sensor_msgs::PointCloud& cloud, ostream& out) {
  out << "Cloud" << endl;
  out << "serialization_version_" << endl;
  out << POINTCLOUD_SERIALIZATION_VERSION << endl;
  out << "num_points" << endl;
  out << cloud.points.size() << endl;
  out << "points" << endl;

  float* buf = (float*)malloc(sizeof(float)*cloud.points.size()*3);
  for(size_t i=0; i<cloud.points.size(); ++i) {
    buf[i*3] = cloud.points[i].x;
    buf[i*3+1] = cloud.points[i].y;
    buf[i*3+2] = cloud.points[i].z;
  }
  out.write((char*)buf, sizeof(float)*cloud.points.size()*3);
  free(buf);
}


bool cloudsEqual(const sensor_msgs::PointCloud& c1, const sensor_msgs::PointCloud& c2) {

  // -- Check the points.
  if(c1.points.size() != c2.points.size()) {
    //cout << "Different number of points: " << c1.points.size() << " " << c2.points.size() << endl;
    return false;
  }

  for(size_t i=0; i<c1.points.size(); ++i) {
    if(!floatEq(c1.points[i].x, c2.points[i].x) ||
       !floatEq(c1.points[i].y, c2.points[i].y) ||
       !floatEq(c1.points[i].z, c2.points[i].z)) {
      //cout << "Points are different" << endl;
      return false;
    }
  }

  // -- Check the channels.
  if(c1.channels.size() != c2.channels.size()) {
    //cout << "Different number of channels." << endl;
    return false;
  }

  for(size_t i=0; i<c1.channels.size(); ++i) {
    if(c1.channels[i].values.size() != c1.channels[i].values.size())
      return false;
    for(size_t j=0; j<c1.channels[i].values.size(); ++j) {
      if(!floatEq(c1.channels[i].values[j], c2.channels[i].values[j]))
        return false;
    }
  }

  return true;
}


bool streamTrack(std::string track_manager_filename, const Track& tr) {
  // -- Stick the track on the end.
  ofstream out;
  out.open(track_manager_filename.c_str(), ios::out | ios::app | ios::binary);
  tr.serialize(out);
  out.close();
  
  return true;
}

boost::shared_ptr<sensor_msgs::PointCloud> getRandomCloud() {
  int num_pts = 100;

  boost::shared_ptr<sensor_msgs::PointCloud> cloud(new sensor_msgs::PointCloud());
  cloud->points.resize(num_pts);
  cloud->channels.resize(1);
  cloud->channels[0].values.resize(num_pts);
  double mean_x = rand() % 1000;
  double mean_y = rand() % 1000;
  double mean_z = rand() % 1000;
  for(size_t i = 0; i < cloud->points.size(); ++i) {
    cloud->points[i].x = mean_x + (double)(rand() % 1000) / 1000.0;
    cloud->points[i].y = mean_y + (double)(rand() % 1000) / 1000.0;
    cloud->points[i].z = mean_z + (double)(rand() % 1000) / 1000.0;
    cloud->channels[0].values[i] = rand() % 256;
  }

  return cloud;
}

void getRandomTransform(dgc_transform::dgc_transform_t trans) {
  dgc_transform::identity(trans);
  dgc_transform::translate(trans, rand()%1000, rand()%1000, rand()%1000);
}

boost::shared_ptr<Frame> getRandomFrame(dgc_transform::dgc_transform_t velodyne_offset) {
  boost::shared_ptr<sensor_msgs::PointCloud> cloud = getRandomCloud();
  double timestamp = (double)(rand() % 1000000) / (double)1e3;

  dgc_transform::dgc_pose_t robot_pose;
  robot_pose.x = rand() % 1000;
  robot_pose.y = rand() % 1000;
  robot_pose.z = rand() % 1000;
  robot_pose.roll = (double)(rand() % 2 * M_PI * 1e6) / (double)1e6;
  robot_pose.pitch = (double)(rand() % 2 * M_PI * 1e6) / (double)1e6;
  robot_pose.yaw = (double)(rand() % 2 * M_PI * 1e6) / (double)1e6;

  return boost::shared_ptr<Frame>(new Frame(cloud, timestamp, robot_pose, velodyne_offset));
}

boost::shared_ptr<Track> getRandomTrack() {
  int num_frames = 10;

  int cl = rand() % 3;
  string label;
  switch(cl) {
  case 0:
    label = "bicyclist";
    break;
  case 1:
    label = "pedestrian";
    break;
  case 2:
    label = "car";
    break;
  }

  dgc_transform::dgc_transform_t velodyne_offset;
  getRandomTransform(velodyne_offset);

  vector< boost::shared_ptr<Frame> > frames(num_frames);
  for(size_t i = 0; i < frames.size(); ++i)
    frames[i] = getRandomFrame(velodyne_offset);

  return boost::shared_ptr<Track>(new Track(label, velodyne_offset, frames));
}

boost::shared_ptr<TrackManager> getRandomTrackManager() {
  int num_tracks = 15;

  vector< boost::shared_ptr<Track> > tracks(num_tracks);
  for(size_t i = 0; i < tracks.size(); ++i)
    tracks[i] = getRandomTrack();

  return boost::shared_ptr<TrackManager>(new TrackManager(tracks));
}

// See http://www.cygnus-software.com/papers/comparingfloats/comparingfloats.htm.
bool floatEq(float x, float y, int maxUlps)
{
  // Make sure maxUlps is non-negative and small enough that the
  // default NAN won't compare as equal to anything.
  assert(maxUlps > 0 && maxUlps < 4 * 1024 * 1024);
  int aInt = *(int*)&x;
  // Make aInt lexicographically ordered as a twos-complement int
  if (aInt < 0)
    aInt = 0x80000000 - aInt;
  // Make bInt lexicographically ordered as a twos-complement int
  int bInt = *(int*)&y;
  if (bInt < 0)
    bInt = 0x80000000 - bInt;
  int intDiff = abs(aInt - bInt);
  if (intDiff <= maxUlps)
    return true;
  return false;
}  

void smpToPCL(const sensor_msgs::PointCloud& smp, pcl::PointCloud<pcl::PointXYZRGB>* pcd)
{
  ROS_ASSERT(smp.channels.size() == 1);
  
  pcd->resize(smp.points.size());
  pcd->height = 1;
  pcd->width = smp.points.size();
  pcd->is_dense = true;
  for(size_t i = 0; i < pcd->size(); ++i) {
    (*pcd)[i].x = smp.points[i].x;
    (*pcd)[i].y = smp.points[i].y;
    (*pcd)[i].z = smp.points[i].z;
    (*pcd)[i].r = smp.channels[0].values[i];
    (*pcd)[i].g = smp.channels[0].values[i];
    (*pcd)[i].b = smp.channels[0].values[i];

    ROS_ASSERT(!isnan((*pcd)[i].x) && !isinf((*pcd)[i].x));
    ROS_ASSERT(!isnan((*pcd)[i].y) && !isinf((*pcd)[i].y));
    ROS_ASSERT(!isnan((*pcd)[i].z) && !isinf((*pcd)[i].z));
  }
}


}// namespace track_manager
