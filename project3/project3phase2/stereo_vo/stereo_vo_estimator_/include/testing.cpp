#include "estimator.h"


void drawImage(const cv::Mat& img, const vector<cv::Point2f>& pts, string name) {
  auto draw = img.clone();
  for (unsigned int i = 0; i < pts.size(); i++) {
    cv::circle(draw, pts[i], 2, cv::Scalar(0, 255, 0), -1, 8);
  }
  cv::imshow(name, draw);
  cv::waitKey(1);
}

vector<uchar> rejectWithF(vector<cv::Point2f> &ud1_pts, vector<cv::Point2f> &ud2_pts){
  if(ud2_pts.size()>8){
    vector<uchar> status;
    // If want to use pixel threshold, need to project to the image plane again, but that requires
    // intrinsic matrix which cannot be accessed via camera_model pack.
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
  Tlr = Tl.inverse() * Tr;        // FIXME: according to this, ric is $T_{bc}$, extrinsic is $T_{bc}$
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

  std::cout << "receive new image===========================" << std::endl;

  Estimator::frame cur_frame;
  cur_frame.frame_time = time_stamp;
  cur_frame.img = _img;

  // cv::imshow("img", _img);
  // cv::waitKey(1);

  vector<cv::Point2f> left_pts_2d, right_pts_2d;
  vector<cv::Point3f> prev_pts_3d;


  R_21.setIdentity();
  t_21.setZero();

  if (init_finish) {
    // To do: match features between previous and current left image
    vector<cv::Point2f> tracked_curr_2d;
    trackFeatureBetweenFrames(key_frame, _img, prev_pts_3d, tracked_curr_2d);

    // To do: undistort the points of the left image and compute relative motion with the key frame.
    Matrix3d R_cp;
    Vector3d t_cp;
    estimateTBetweenFrames(prev_pts_3d, tracked_curr_2d, R_cp, t_cp);

    printf("\n===== Init finished ====\n"
           "== relative to KF: %6.3f, %6.3f, %6.3f\n", t_cp(0), t_cp(1), t_cp(2));
    cout<<t_cp.norm()<<endl;


    if(static_cast<int>(tracked_curr_2d.size())<MIN_CNT){
      cout<< "\033[1;33mLow tracked feature num\033[0m "<<tracked_curr_2d.size()<<endl;
    }

    cout<<"== Key frame position: "<<key_frame.w_t_c.transpose()<<endl;

    R_21 = R_cp;
    t_21 = t_cp;

  }

  Matrix4d T_wc, T_wp, T_cp0;
  T_wp.setZero();
  T_cp0.setZero();

  T_cp0.block<3,3>(0,0) = R_21;
  T_cp0.block<3,1>(0,3) = t_21;
  T_cp0(3,3) = 1;

  T_wp.block<3,3>(0,0) = key_frame.w_R_c;
  T_wp.block<3,1>(0,3) = key_frame.w_t_c;
  T_wp(3,3) = 1;

  T_wc = T_wp * T_cp0.inverse();

  cur_frame.w_R_c = T_wc.block<3,3>(0,0);
  cur_frame.w_t_c = T_wc.block<3,1>(0,3);


  // To do: extract new features for the current frame.
  vector<cv::Point2f> ud_left_pts_2d, ud_right_pts_2d, tracked_in_right_2d, ud_tracked_2d;

  extractNewFeatures(_img, left_pts_2d);

  // To do: compute the 3d position for the new undistorted features in the world frame which is aligned with the body frame at the initialization.
  trackFeatureLeftRight(_img, _img1, left_pts_2d, tracked_in_right_2d);


  cur_frame.uv = left_pts_2d;

  // Undistort to reconstruct:
  ud_left_pts_2d = undistortedPts(left_pts_2d, m_camera[0]);
  ud_tracked_2d = undistortedPts(tracked_in_right_2d, m_camera[1]);

  int beforeSize=ud_left_pts_2d.size(), afterSize;
  vector<uchar> ransacMask = rejectWithF(ud_left_pts_2d, ud_tracked_2d);
  reduceVector(ud_left_pts_2d, ransacMask);
  reduceVector(ud_tracked_2d, ransacMask);
  reduceVector(cur_frame.uv, ransacMask);

  afterSize = cur_frame.uv.size();
  printf("== RANSAC, lr: %d %d\n", beforeSize, afterSize);

  // 3D points in left camera coordinate
  cur_frame.xyz.clear();
  generate3dPoints(ud_left_pts_2d, ud_tracked_2d, Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero(), cur_frame.xyz, cur_frame.uv);



  // Change key frame
  if(t_21.norm() > TRANSLATION_THRESHOLD || acos(Quaterniond(R_21).w()) * 2.0 > ROTATION_THRESHOLD || prev_pts_3d.size() < FEATURE_THRESHOLD || !init_finish){
    key_frame = cur_frame;    // 5cm of translation or 3 deg of rotation will change the kf.
    ROS_INFO("Change key frame to current frame.");
  }

  prev_frame = cur_frame;

  updateLatestStates(cur_frame);

  init_finish = true;

  return true;
}

bool Estimator::trackFeatureBetweenFrames(const Estimator::frame& prev_frame, const cv::Mat& cur_img,
                                             vector<cv::Point3f>& prev_pts_3d,
                                             vector<cv::Point2f>& cur_pts_2d) {

  cur_pts_2d.clear();
  std::cout << "prev feature num: " << prev_frame.uv.size() << std::endl;

  // To do: track features between previous and current frames and obtain corresponding 2D, 3D points.
  vector<uchar> pc_status;  // prev-curr matching status
  vector<float> pc_err;
  vector<cv::Point2f> prev_pts_2d = prev_frame.uv;
  prev_pts_3d = prev_frame.xyz;
  vector<cv::Point2f> ud_prev_pts_2d, ud_tracked_curr_2d;

  cv::calcOpticalFlowPyrLK(key_frame.img, cur_img, prev_pts_2d, cur_pts_2d, pc_status, pc_err, cv::Size(21, 21), 3);
  for (int i = 0; i < int(cur_pts_2d.size()); i++)
    if (pc_status[i] && !inBorder(cur_pts_2d[i], ROW, COL))
      pc_status[i] = 0;

  reduceVector(prev_pts_2d, pc_status);
  reduceVector(prev_pts_3d, pc_status);
  reduceVector(cur_pts_2d, pc_status);

  int beforeSize=prev_pts_2d.size(), afterSize;
  ud_prev_pts_2d = undistortedPts(prev_pts_2d, m_camera[0]);
  ud_tracked_curr_2d = undistortedPts(cur_pts_2d, m_camera[0]);

  // ransac
  auto ransacMask = rejectWithF(ud_prev_pts_2d, ud_tracked_curr_2d);
//  reduceVector(prev_pts_2d, ransacMask);
  reduceVector(prev_pts_3d, ransacMask);
  reduceVector(cur_pts_2d, ransacMask);
  afterSize = cur_pts_2d.size();
  printf("== RANSAC, pc: %d %d\n", beforeSize, afterSize);


  if(SHOW_FEATURE){
    drawImage(prev_frame.img, prev_pts_2d, "key frame");
    drawImage(cur_img, cur_pts_2d, "current frame");
  }

  std::cout << "tracked feature num: " << prev_frame.uv.size() << std::endl;
  return int(cur_pts_2d.size()) >= MIN_CNT;

}

bool Estimator::estimateTBetweenFrames(vector<cv::Point3f>& prev_pts_3d,
                                          vector<cv::Point2f>& cur_pts_2d, Matrix3d& R, Vector3d& t) {

  if (int(cur_pts_2d.size()) < 4) {
    printf("feature tracking not enough, please slowly move you device! \n");
    return false;
  }

  std::cout << "3d: " << prev_pts_3d.size() << ", 2d: " << cur_pts_2d.size() << std::endl;

  // To do: calculate relative pose between previous and current frame using the match 2d-3d points
  cv::Mat rvec, tvec, Rmat;
  m_camera[0]->estimateExtrinsics(prev_pts_3d, cur_pts_2d, rvec, tvec);
  cv::Rodrigues(rvec, Rmat);
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++) {
      R(i, j) = Rmat.at<double>(i, j);
    }
  t(0) = tvec.at<double>(0, 0);
  t(1) = tvec.at<double>(1, 0);
  t(2) = tvec.at<double>(2, 0);

  return true;
}

void Estimator::extractNewFeatures(const cv::Mat& img, vector<cv::Point2f>& uv) {
  //To do: extract the new 2d features of img and store them in uv.
  cv::goodFeaturesToTrack(img, uv, MAX_CNT, 0.03, MIN_DIST);

}


bool Estimator::trackFeatureLeftRight(const cv::Mat& _img, const cv::Mat& _img1,
                                         vector<cv::Point2f>& left_pts, vector<cv::Point2f>& right_pts) {

  // To do: track features left to right frame and obtain corresponding 2D points.
  vector<uchar> lr_status;
  vector<float> lr_err;
  cv::calcOpticalFlowPyrLK(_img, _img1, left_pts, right_pts, lr_status, lr_err, cv::Size(21, 21), 3);

  for (int i = 0; i < int(right_pts.size()); i++)
    if (lr_status[i] && !inBorder(right_pts[i], ROW, COL))
      lr_status[i] = 0;

  reduceVector(left_pts, lr_status);
  cout<<"== LK LR tracked and lost "<<left_pts.size()<<", "<<right_pts.size()<<endl;
  reduceVector(right_pts, lr_status);

  return int(left_pts.size()) >= MIN_CNT;

}

void Estimator::generate3dPoints(const vector<cv::Point2f>& left_pts,
                                 const vector<cv::Point2f>& right_pts, const Matrix3d& Ri,
                                 const Vector3d& ti, vector<cv::Point3f>& cur_pts_3d,
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

  // To do: update the latest_time, latest_pointcloud, latest_P and latest_Q.
  // latest_P and latest_Q should be in the world frame which is aligned with the body frame at the initialization.

  latest_time = latest_frame.frame_time;
  latest_pointcloud = latest_frame.xyz;

  // T_wi, inertial(body) to the world.
  latest_P = latest_frame.w_R_c * (-ric[0].transpose()*tic[0]) + latest_frame.w_t_c;
  latest_Q = Eigen::Quaterniond(latest_frame.w_R_c * ric[0].transpose()).normalized();
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


double Estimator::reprojectionError(Matrix3d &R, Vector3d &t, cv::Point3f &prev_pts_3d, cv::Point2f &cur_pts_2d){
    Vector3d pt1(prev_pts_3d.x, prev_pts_3d.y, prev_pts_3d.z);
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