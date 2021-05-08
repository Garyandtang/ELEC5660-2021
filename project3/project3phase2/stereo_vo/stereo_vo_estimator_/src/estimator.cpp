#include "estimator.h"


void drawImage(const cv::Mat& img, const vector<cv::Point2f>& pts, string name) {
  auto draw = img.clone();
  for (unsigned int i = 0; i < pts.size(); i++) {
    cv::circle(draw, pts[i], 2, cv::Scalar(0, 255, 0), -1, 8);
  }
  cv::imshow(name, draw);
  cv::waitKey(1);
}
// based on vins mono, details is shown in following link
// https://github.com/HKUST-Aerial-Robotics/VINS-Mono/blob/master/feature_tracker/src/feature_tracker.cpp
// In vins-mono ransacReprojThreshold (F_threshold) is set to be 1, here I set it to 0.01
vector<uchar> rejectWithF(vector<cv::Point2f> &ud1_pts, vector<cv::Point2f> &ud2_pts){
  if(ud2_pts.size()>8){
    vector<uchar> status;
    cv::findFundamentalMat(ud1_pts, ud2_pts, cv::FM_RANSAC, 0.2*1.0/min(COL,ROW), 0.99, status);
    return status;
  }
  return vector<uchar>();

}
Estimator::Estimator() {
  ROS_INFO("Estimator init begins.");
  prev_frame.frame_time = ros::Time(0.0);
  prev_frame.w_t_c = Eigen::Vector3d(0, 0, 0);
  prev_frame.w_R_c = Eigen::Matrix3d::Identity();
  fail_cnt = 0;
  init_finish = false;
}

void Estimator::reset() {
  ROS_ERROR("Lost, reset!");
  key_frame = prev_frame;
  fail_cnt = 0;
  init_finish = false;
}

void Estimator::setParameter() {
  for (int i = 0; i < 2; i++) {
    tic[i] = TIC[i];
    ric[i] = RIC[i];
    cout << " exitrinsic cam " << i << endl << ric[i] << endl << tic[i].transpose() << endl;
  }

  prev_frame.frame_time = ros::Time(0.0);
  prev_frame.w_t_c = tic[0];
  prev_frame.w_R_c = ric[0];
  key_frame = prev_frame;

  readIntrinsicParameter(CAM_NAMES);

  // transform between left and right camera
  Matrix4d Tl, Tr;
  Tl.setIdentity();
  Tl.block(0, 0, 3, 3) = ric[0];
  Tl.block(0, 3, 3, 1) = tic[0];
  Tr.setIdentity();
  Tr.block(0, 0, 3, 3) = ric[1];
  Tr.block(0, 3, 3, 1) = tic[1];
  Tlr = Tl.inverse() * Tr;
}


void Estimator::readIntrinsicParameter(const vector<string>& calib_file) {
  for (size_t i = 0; i < calib_file.size(); i++) {
    ROS_INFO("reading paramerter of camera %s", calib_file[i].c_str());
    camodocal::CameraPtr camera =
        camodocal::CameraFactory::instance()->generateCameraFromYamlFile(calib_file[i]);
    m_camera.push_back(camera);
  }
}

bool Estimator::inputImage(ros::Time time_stamp, const cv::Mat& _img, const cv::Mat& _img1) {

  if(fail_cnt > 20){
    reset();
  }
  std::cout << "receive new image===========================" << std::endl;

  Estimator::frame cur_frame;
  cur_frame.frame_time = time_stamp;
  cur_frame.img = _img;

  // cv::imshow("img", _img);
  // cv::waitKey(1);

  vector<cv::Point2f> left_pts_2d, right_pts_2d;
  vector<cv::Point2f> ud_left_pts_2d, ud_right_pts_2d; // undistorted points
  vector<cv::Point3f> key_pts_3d;
  vector<cv::Point3f> cur_pts_3d; //result of generate3dPoints
  
  c_R_k.setIdentity(); // this is key frame under camera frame
  c_t_k.setZero(); // this is key frame under camera frame

  if (init_finish) {
    vector<cv::Point2f> cur_pts_2d; // result of trackFeatureBetweenFrames


    // To do: match features between the key frame and the current left image
    Estimator::trackFeatureBetweenFrames(key_frame, _img, key_pts_3d, cur_pts_2d);

    // To do: undistort the points of the left image and compute relative motion with the key frame. 
    Estimator::estimateTBetweenFrames(key_pts_3d, cur_pts_2d, c_R_k, c_t_k);

  }

  Eigen::Matrix3d R_kc = c_R_k.transpose();
  Eigen::Vector3d t_kc = -c_R_k.transpose() * c_t_k;
  rel_key_time = key_frame.frame_time;
  // To do: extract new features for the current frame.
  // step 1: feature detection
  Estimator::extractNewFeatures(_img, left_pts_2d);
  // step 2: feature tracking
  Estimator::trackFeatureLeftRight(_img, _img1, left_pts_2d, right_pts_2d);
  cur_frame.uv = left_pts_2d;

   // To do: undistort the 2d points of the current frame and generate the corresponding 3d points. 
  // step 3: 3D point generation
  ud_left_pts_2d = Estimator::undistortedPts(left_pts_2d, m_camera[0]);
  ud_right_pts_2d = Estimator::undistortedPts(right_pts_2d, m_camera[1]);
  vector<uchar> ransacMask = rejectWithF(ud_left_pts_2d, ud_right_pts_2d);
  reduceVector(ud_left_pts_2d, ransacMask);
  reduceVector(ud_right_pts_2d, ransacMask);
  reduceVector(cur_frame.uv, ransacMask);

  cur_frame.xyz.clear();
  Estimator::generate3dPoints(ud_left_pts_2d, ud_right_pts_2d, cur_frame.xyz , cur_frame.uv);


  cur_frame.w_R_c = key_frame.w_R_c*R_kc;
  cur_frame.w_t_c = key_frame.w_R_c*t_kc+key_frame.w_t_c;
  


  // Change key frame
  if(c_t_k.norm() > TRANSLATION_THRESHOLD || acos(Quaterniond(c_R_k).w()) * 2.0 > ROTATION_THRESHOLD || key_pts_3d.size() < FEATURE_THRESHOLD || !init_finish){
    key_frame = cur_frame;
    ROS_INFO("Change key frame to current frame.");
  }

  prev_frame = cur_frame;
  
  updateLatestStates(cur_frame);

  init_finish = true;

  return true;
}

bool Estimator::trackFeatureBetweenFrames(const Estimator::frame& keyframe, const cv::Mat& cur_img,
                                             vector<cv::Point3f>& key_pts_3d,
                                             vector<cv::Point2f>& cur_pts_2d) {

  // To do: track features between the key frame and the current frame to obtain corresponding 2D, 3D points.

  vector<uchar> status_list;
  vector<float> error_list;
  key_pts_3d = key_frame.xyz;
  vector<cv::Point2f> key_pts_2d = key_frame.uv;
  cv::calcOpticalFlowPyrLK(key_frame.img, cur_img, key_frame.uv, cur_pts_2d, status_list, error_list, cv::Size(21,21), 3);
  for (int i = 0; i < cur_pts_2d.size(); i++){
    if (status_list[i] && !inBorder(cur_pts_2d[i], ROW, COL)){
      status_list[i] = 0;
    }
  }
  reduceVector(key_pts_2d, status_list);
  reduceVector(key_pts_3d, status_list);
  reduceVector(cur_pts_2d, status_list);
  vector<cv::Point2f> ud_key_pts_2d, ud_cur_pts_2d;
  ud_key_pts_2d = Estimator::undistortedPts(key_pts_2d, m_camera[0]);
  ud_cur_pts_2d = Estimator::undistortedPts(cur_pts_2d, m_camera[0]);
  vector<uchar> ransacMask = rejectWithF(ud_key_pts_2d, ud_cur_pts_2d);
  reduceVector(key_pts_3d, ransacMask);
  reduceVector(cur_pts_2d, ransacMask);

  return true;
}


bool Estimator::estimateTBetweenFrames(vector<cv::Point3f>& key_pts_3d,
                                          vector<cv::Point2f>& cur_pts_2d, Matrix3d& R, Vector3d& t) {

  // To do: calculate relative pose between the key frame and the current frame using the matched 2d-3d points
  if (int(cur_pts_2d.size()) < 4) {
    printf("feature tracking not enough, please slowly move you device! \n");
    return false;
  }
  vector<cv::Point2f> ud_cur_pts_2d; 
  ud_cur_pts_2d = Estimator::undistortedPts(cur_pts_2d, m_camera[0]);
  cv::Mat r, rvec, t_mat, D;
  cv::Mat fake_K = cv::Mat::eye(3, 3, CV_64F);
  cv::solvePnPRansac(key_pts_3d, ud_cur_pts_2d, fake_K, D, rvec, t_mat, false,200, 6,0.98, cv::noArray());
  cv::Rodrigues(rvec, r);
  for (int i = 0; i < 3; i++){
      t(i)=t_mat.at<double>(i, 0);
  }
  for(int i=0;i<3;i++)
      for(int j=0;j<3;j++)
      {
          R(i,j) = r.at<double>(i, j);
      }
  cout << "camera to keyframe Rotation matrix:\n" << R << endl;
  cout << "camera to keyframe Translation vector:\n" << t << endl;

  return true;
}

void Estimator::extractNewFeatures(const cv::Mat& img, vector<cv::Point2f>& uv) {
  
  //To do: extract the new 2d features of img and store them in uv.
  int max_corners = 150;
	double quality_level = 0.03;
	double min_distance = 20.0;
	int block_size = 3;
	bool use_harris = false;
	double k = 0.04;
  
  cv::goodFeaturesToTrack(img, 
							uv, 
							max_corners, 
							quality_level, 
							min_distance, 
							cv::Mat(), 
							block_size, 
							use_harris, 
							k);

}


bool Estimator::trackFeatureLeftRight(const cv::Mat& _img, const cv::Mat& _img1,
                                         vector<cv::Point2f>& left_pts, vector<cv::Point2f>& right_pts) {

  // To do: track features left to right frame and obtain corresponding 2D points.
  // cv::calcOpticalFlowPyrLK(_img, _img1, left_pts, right_pts );
  vector<uchar> status_list;
  vector<float> error_list;
  cv::TermCriteria criteria = cv::TermCriteria((cv::TermCriteria::COUNT)+ (cv::TermCriteria::EPS), 10, 0.03);
  cv::calcOpticalFlowPyrLK(_img, _img1, left_pts, right_pts, status_list, error_list, cv::Size(40,40), 4, criteria);
  for (int i = 0; i < right_pts.size(); i++){
    if (status_list[i] && !inBorder(right_pts[i], ROW, COL)){
      status_list[i] = 0;
    }
  }
  reduceVector(left_pts, status_list);
  std::cout << "reduced points num: " << right_pts.size() - left_pts.size() << std::endl;
  reduceVector(right_pts, status_list);
  drawImage(_img1, right_pts, "LK_result");

  return true;
}

void Estimator::generate3dPoints(const vector<cv::Point2f>& left_pts,
                                 const vector<cv::Point2f>& right_pts, 
                                 vector<cv::Point3f>& cur_pts_3d,
                                 vector<cv::Point2f>& cur_pts_2d) {

  Eigen::Matrix<double, 3, 4> P1, P2;

  P1 << 1, 0, 0, 0,  0, 1, 0, 0,  0, 0, 1, 0;
  P2.block(0,0,3,3) = (Tlr.block(0,0,3,3).transpose());
  P2.block(0,3,3,1) = -P2.block(0,0,3,3) * Tlr.block(0,3,3,1);

  vector<uchar> status;

  for (unsigned int i = 0; i < left_pts.size(); ++i) {
    Vector2d pl(left_pts[i].x, left_pts[i].y);
    Vector2d pr(right_pts[i].x, right_pts[i].y);
    Vector3d pt3;
    triangulatePoint(P1, P2, pl, pr, pt3);

    if (pt3[2] > 0) {
      cur_pts_3d.push_back(cv::Point3f(pt3[0], pt3[1], pt3[2]));
      status.push_back(1);
    } else {
      status.push_back(0);
    }
  }

  reduceVector<cv::Point2f>(cur_pts_2d, status);
}


bool Estimator::inBorder(const cv::Point2f& pt, const int& row, const int& col) {
  const int BORDER_SIZE = 1;
  int img_x = cvRound(pt.x);
  int img_y = cvRound(pt.y);
  return BORDER_SIZE <= img_x && img_x < col - BORDER_SIZE && BORDER_SIZE <= img_y &&
      img_y < row - BORDER_SIZE;
}


double Estimator::distance(cv::Point2f pt1, cv::Point2f pt2) {
  double dx = pt1.x - pt2.x;
  double dy = pt1.y - pt2.y;
  return sqrt(dx * dx + dy * dy);
}


template <typename Derived>
void Estimator::reduceVector(vector<Derived>& v, vector<uchar> status) {
  int j = 0;
  for (int i = 0; i < int(v.size()); i++)
    if (status[i]) v[j++] = v[i];
  v.resize(j);
}


void Estimator::updateLatestStates(frame &latest_frame) {
  
  // To do: update the latest_time, latest_pointcloud, latest_P, latest_Q, latest_rel_P and latest_rel_Q.
  // latest_P and latest_Q should be the pose of the body (IMU) in the world frame.
  // latest_rel_P and latest_rel_Q should be the relative pose of the current body frame relative to the body frame of the key frame.
  // latest_pointcloud should be in the current camera frame.
  latest_time = latest_frame.frame_time;
  latest_pointcloud = latest_frame.xyz;
  // latest_P = latest_frame.w_t_c;
  // latest_Q = Eigen::Quaterniond(latest_frame.w_R_c);
  latest_P = latest_frame.w_R_c *(-ric[0].transpose()*tic[0]) + latest_frame.w_t_c;
  latest_Q = Eigen::Quaterniond(latest_frame.w_R_c * ric[0].transpose()).normalized();

  latest_rel_P = -ric[0]*c_R_k.transpose()*c_t_k;
  latest_rel_Q = Eigen::Quaterniond(ric[0]*c_R_k.transpose()*ric[0].transpose());

  // latest_rel_Q = -c_R_k.transpose()*ric[0].transpose();
  // latest_rel_P = -c_R_k.transpose()*ric[0].transpose()*tic[0]-c_R_k.transpose()*c_t_k;
  
}


void Estimator::triangulatePoint(Eigen::Matrix<double, 3, 4>& Pose0, Eigen::Matrix<double, 3, 4>& Pose1,
                                 Eigen::Vector2d& point0, Eigen::Vector2d& point1,
                                 Eigen::Vector3d& point_3d) {
  Eigen::Matrix4d design_matrix = Eigen::Matrix4d::Zero();
  design_matrix.row(0) = point0[0] * Pose0.row(2) - Pose0.row(0);
  design_matrix.row(1) = point0[1] * Pose0.row(2) - Pose0.row(1);
  design_matrix.row(2) = point1[0] * Pose1.row(2) - Pose1.row(0);
  design_matrix.row(3) = point1[1] * Pose1.row(2) - Pose1.row(1);
  Eigen::Vector4d triangulated_point;
  triangulated_point = design_matrix.jacobiSvd(Eigen::ComputeFullV).matrixV().rightCols<1>();
  point_3d(0) = triangulated_point(0) / triangulated_point(3);
  point_3d(1) = triangulated_point(1) / triangulated_point(3);
  point_3d(2) = triangulated_point(2) / triangulated_point(3);
}


double Estimator::reprojectionError(Matrix3d &R, Vector3d &t, cv::Point3f &key_pts_3d, cv::Point2f &cur_pts_2d){
    Vector3d pt1(key_pts_3d.x, key_pts_3d.y, key_pts_3d.z);
    Vector3d pt2 = R * pt1 + t;
    pt2 = pt2 / pt2[2];
    return sqrt(pow(pt2[0] - cur_pts_2d.x, 2) + pow(pt2[1] - cur_pts_2d.y, 2));
}


vector<cv::Point2f> Estimator::undistortedPts(vector<cv::Point2f>& pts, camodocal::CameraPtr cam) {
  vector<cv::Point2f> un_pts;
  for (unsigned int i = 0; i < pts.size(); i++) {
    Eigen::Vector2d a(pts[i].x, pts[i].y);
    Eigen::Vector3d b;
    cam->liftProjective(a, b);
    un_pts.push_back(cv::Point2f(b.x() / b.z(), b.y() / b.z()));
  }
  return un_pts;
}
