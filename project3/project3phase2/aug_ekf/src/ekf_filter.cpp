#include <ekf_filter.h>
#include "backward.hpp"

namespace backward {

backward::SignalHandling sh;

} // namespace backward


namespace ekf_imu_vision {
EKFImuVision::EKFImuVision(/* args */) {}

EKFImuVision::~EKFImuVision() {}

double clamp_euler_angle(double theta)
{
    double result = atan2 (sin(theta),cos(theta)); 
    return result;
}


void EKFImuVision::init(ros::NodeHandle& nh) {
  node_ = nh;

  /* ---------- parameter ---------- */
  Qt_.setZero();
  Rt1_.setZero();
  Rt2_.setZero();

  // addition and removel of augmented state
  
  // TODO
  // set M_a_ and M_r_
  M_r_.setZero();
  M_r_.block<15,15>(0,0) = Eigen::MatrixXd::Identity(15,15);

  M_a_.setZero();
  M_a_.block<15,15>(0,0) = Eigen::MatrixXd::Identity(15, 15);
  M_a_.block<6,6>(15,0) = Eigen::MatrixXd::Identity(6, 6);
  for (int i = 0; i < 3; i++) {
    /* process noise */
    node_.param("aug_ekf/ng", Qt_(i, i), -1.0);
    node_.param("aug_ekf/na", Qt_(i + 3, i + 3), -1.0);
    node_.param("aug_ekf/nbg", Qt_(i + 6, i + 6), -1.0);
    node_.param("aug_ekf/nba", Qt_(i + 9, i + 9), -1.0);
    node_.param("aug_ekf/pnp_p", Rt1_(i, i), -1.0);
    node_.param("aug_ekf/pnp_q", Rt1_(i + 3, i + 3), -1.0);
    node_.param("aug_ekf/vo_pos", Rt2_(i, i), -1.0);
    node_.param("aug_ekf/vo_rot", Rt2_(i + 3, i + 3), -1.0);
  }
  cout << "Qt_:\n" << Qt_ << endl;
  cout << "Rt1_:\n" << Rt1_ << endl;
  cout << "Rt2_:\n" << Rt2_ << endl;
  cout << "deque 0: " << aug_state_hist_.begin()->mean << endl;

  init_        = false;
  big_bag      = false;

  for(int i = 0; i < 4; i++)
    latest_idx[i] = 0;

  /* ---------- subscribe and publish ---------- */
  imu_sub_ =
      node_.subscribe<sensor_msgs::Imu>("/dji_sdk_1/dji_sdk/imu", 100, &EKFImuVision::imuCallback, this);
  pnp_sub_     = node_.subscribe<nav_msgs::Odometry>("tag_odom", 10, &EKFImuVision::PnPCallback, this);
  // opti_tf_sub_ = node_.subscribe<geometry_msgs::PointStamped>("opti_tf_odom", 10,
                                                              // &EKFImuVision::opticalCallback, this);
  stereo_sub_  = node_.subscribe<stereo_vo::relative_pose>("/vo/Relative_pose", 10,
                                                          &EKFImuVision::stereoVOCallback, this);
  fuse_odom_pub_ = node_.advertise<nav_msgs::Odometry>("ekf_fused_odom", 10);
  path_pub_         = node_.advertise<nav_msgs::Path>("/aug_ekf/Path", 100);

  ros::Duration(0.5).sleep();

  ROS_INFO("Start ekf.");
}

void EKFImuVision::PnPCallback(const nav_msgs::OdometryConstPtr& msg) {

  // TODO
  // construct a new state using the absolute measurement from marker PnP and process the new state

  bool pnp_lost = fabs(msg->pose.pose.position.x) < 1e-4 && fabs(msg->pose.pose.position.y) < 1e-4 &&
      fabs(msg->pose.pose.position.z) < 1e-4;
  if (pnp_lost) return;
  Vec3 c_t_w, i_t_c, t_w_b;
  Mat3x3 c_R_w, i_R_c, R_w_b;
  if (big_bag == false){

    R_w_b = Eigen::Quaterniond(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
                                  msg->pose.pose.orientation.y, msg->pose.pose.orientation.z)
                    .toRotationMatrix();
    t_w_b;
    t_w_b[0] = msg->pose.pose.position.x;
    t_w_b[1] = msg->pose.pose.position.y;
    t_w_b[2] = msg->pose.pose.position.z;
  } else {
    c_R_w = Eigen::Quaterniond(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
                                  msg->pose.pose.orientation.y, msg->pose.pose.orientation.z)
                    .toRotationMatrix();
    c_t_w[0] = msg->pose.pose.position.x;
    c_t_w[1] = msg->pose.pose.position.y;
    c_t_w[2] = msg->pose.pose.position.z;

    i_t_c << 0.07, -0.02, 0.01;
    i_R_c << 1,  0,  0,
             0, -1,  0,
             0,  0, -1;
    i_t_c.setZero();
    i_R_c.setIdentity();
    R_w_b = c_R_w.transpose()*i_R_c.transpose();
    t_w_b = -c_R_w.transpose()*i_R_c.transpose()*i_t_c - c_R_w.transpose()*c_t_w;
  }

  AugState         new_state;

  new_state.time_stamp = msg->header.stamp;
  new_state.type = pnp;
  new_state.mean = Vec21::Zero(21);
  new_state.covariance = 0.8*Mat21x21::Identity(21,21);
  new_state.ut.head(3)  =     t_w_b;
  new_state.ut.segment(3, 3) = rotation2Euler(R_w_b);
  new_state.mean.segment(0,6) = new_state.ut;
  new_state.mean.segment(15,6) = new_state.ut;
  // cout << "====//////PnPCallback///////==========" << endl;
  // cout << new_state.mean.transpose()<<endl;
  deque<AugState>::iterator state_it;

  // step 0: initialization with first PnP measurement
  if (init_ == false){

    state_it = insertNewState(new_state);
    init_ = initUsingPnP(state_it);
    changeAugmentedState(*state_it);
    cout << "first state:\n" << state_it->mean.transpose() << endl;
    latest_idx[keyframe] = distance(aug_state_hist_.begin(),state_it);
    cout << "keyframe index: " << latest_idx[keyframe] << endl;
    return;
    
  }

  // step 1: insert new state
  state_it = insertNewState(new_state);

  
  updatePnP(*state_it,*(state_it-1));

  // step 3: repropagate
  repropagate(state_it, init_);

  removeOldState();

  
  if (!processNewState(new_state, false)) {
    return;
  }

}

void EKFImuVision::stereoVOCallback(const stereo_vo::relative_poseConstPtr& msg) {

  // TODO
  // label the previous keyframe
  // construct a new state using the relative measurement from VO and process the new state
  if(!init_){
    return;
  }
  // step 1: construct a new state
  ros::Time keyframe_time = msg->key_stamp, vo_time = msg->header.stamp;
  Mat3x3 i_R_k;
  Vec3 i_t_k;
  i_R_k = Eigen::Quaterniond(msg->relative_pose.orientation.w, msg->relative_pose.orientation.x,
                                  msg->relative_pose.orientation.y, msg->relative_pose.orientation.z)
                    .toRotationMatrix();
  i_t_k[0] = msg->relative_pose.position.x;
  i_t_k[1] = msg->relative_pose.position.y;
  i_t_k[2] = msg->relative_pose.position.z;

  AugState new_state;
  new_state.time_stamp = vo_time;
  new_state.type = vo;
  new_state.ut.segment(0,3) = i_t_k;
  new_state.ut.segment(3,3) = rotation2Euler(i_R_k);
  new_state.key_frame_time_stamp = keyframe_time;

  // step 2: preocess new state
  deque<AugState>::iterator state_it;
  state_it = insertNewState(new_state);
  deque<AugState>::iterator keyframe_it = aug_state_hist_.end();
  if(keyframe_time != aug_state_hist_.at(latest_idx[keyframe]).time_stamp){

    while (keyframe_it != aug_state_hist_.begin() && keyframe_it->time_stamp !=keyframe_time){
      keyframe_it--;
    }
    if (keyframe_it->time_stamp.toSec() ==0){
      updateVO(*state_it,*(state_it-1));
      repropagate(state_it, init_);
      removeOldState();
      return;
    }
    cout << "key_frame index:" << distance(aug_state_hist_.begin(),keyframe_it) << endl;
    changeAugmentedState(*keyframe_it);
    latest_idx[keyframe] = distance(aug_state_hist_.begin(),keyframe_it);
    updateVO(*state_it,*(state_it-1));
    repropagate(keyframe_it, init_);
  } else{
    updateVO(*state_it,*(state_it-1));
    repropagate(state_it, init_);

  }
  removeOldState();

}

void EKFImuVision::imuCallback(const sensor_msgs::ImuConstPtr& imu_msg) {

  // TODO
  // construct a new state using the IMU input and process the new state
  if(!init_){
    return;
  }
  // step 1: construct the new state
  Vec3 accel(imu_msg->linear_acceleration.x, imu_msg->linear_acceleration.y, imu_msg->linear_acceleration.z);
  Vec3 ang_vel(imu_msg->angular_velocity.x,imu_msg->angular_velocity.y,imu_msg->angular_velocity.z);
  AugState new_state;
  new_state.time_stamp = imu_msg->header.stamp;
  new_state.type = imu;
  new_state.ut.segment(0,3) = ang_vel;
  new_state.ut.segment(3,3) = accel;

  // step 2: process new state
  Vec6 tmp_ut = Vec6::Zero(6);
  aug_state_hist_.push_back(new_state);
  latest_idx[imu] = aug_state_hist_.size()-1;
  predictIMU(aug_state_hist_.at(latest_idx[imu]), aug_state_hist_.at(latest_idx[imu]-1),tmp_ut);
  Mat3x3 R_EKF;
  
  Vec21 x_est =aug_state_hist_.at(max(latest_idx[pnp],latest_idx[vo])).mean; 
  // Vec21 x_est =aug_state_hist_.at(latest_idx[imu]).mean; 
  R_EKF << cos(x_est(5))*cos(x_est(4))- sin(x_est(3))*sin(x_est(5))*sin(x_est(4)), -cos(x_est(3))*sin(x_est(5)), cos(x_est(5))*sin(x_est(4))+cos(x_est(4))*sin(x_est(3))*sin(x_est(5)),
        cos(x_est(4))*sin(x_est(5))+cos(x_est(5))*(sin(x_est(3)))*sin(x_est(4)), cos(x_est(3))*cos(x_est(5)), sin(x_est(5))*sin(x_est(4))-cos(x_est(5))*sin(x_est(3))*cos(x_est(4)),
        -cos(x_est(3))*sin(x_est(4)), sin(x_est(3)), cos(x_est(3))*cos(x_est(4));
  Eigen::Quaterniond Q_EKF(R_EKF);
  nav_msgs::Odometry odom_EKF;
  odom_EKF.header.stamp = aug_state_hist_.at(latest_idx[imu]).time_stamp;
  odom_EKF.header.frame_id = "world";
  odom_EKF.pose.pose.position.x = x_est(0);
  odom_EKF.pose.pose.position.y = x_est(1);
  odom_EKF.pose.pose.position.z = x_est(2);
  odom_EKF.twist.twist.linear.x = x_est(6);
  odom_EKF.twist.twist.linear.y = x_est(7);
  odom_EKF.twist.twist.linear.z = x_est(8);
  odom_EKF.pose.pose.orientation.w = Q_EKF.w();
  odom_EKF.pose.pose.orientation.x = Q_EKF.x();
  odom_EKF.pose.pose.orientation.y = Q_EKF.y();
  odom_EKF.pose.pose.orientation.z = Q_EKF.z();
  fuse_odom_pub_.publish(odom_EKF);

  geometry_msgs::PoseStamped path_pose;
  path_pose.header.frame_id = path_.header.frame_id = "world";
  path_pose.pose.position.x                         = x_est(0);
  path_pose.pose.position.y                         = x_est(1);
  path_pose.pose.position.z                         = x_est(2);
  path_.poses.push_back(path_pose);
  path_pub_.publish(path_);

  
  // cout << "size of deque: " << aug_state_hist_.size() << endl;
}

void EKFImuVision::predictIMU(AugState& cur_state, AugState& prev_state, Vec6 ut) {
//  cout << "==Begin IMU prediction===" << endl;
  double dt = (cur_state.time_stamp - prev_state.time_stamp).toSec();
  Vec15 f = Vec15::Zero(15);
  Vec12 n_t = Vec12::Zero(12);
  Mat15x15 At, Ft;
  Mat15x12 Ut, Vt;
  f = modelF(prev_state.mean.segment(0,15), cur_state.ut,n_t);
  cur_state.mean.segment(15,6) = prev_state.mean.segment(15,6);
  cur_state.mean.segment(0,15) = prev_state.mean.segment(0,15) + dt*f;

  Mat15x15 A_t = jacobiFx(prev_state.mean.segment(0,15), cur_state.ut, n_t), F_t;
  F_t = Mat15x15::Identity(15,15)+dt*A_t;

  Mat15x12 U_t = jacobiFn(prev_state.mean.segment(0,15), cur_state.ut, n_t), V_t;
  V_t = dt * U_t;

  cur_state.covariance.block<15,15>(0,0) = F_t * prev_state.covariance.block<15,15>(0,0) * F_t.transpose() + V_t * Qt_ * V_t.transpose();
  cur_state.covariance.block<6,15>(15,0) = prev_state.covariance.block<6,15>(15,0) * F_t.transpose();
  cur_state.covariance.block<15,6>(0,15) = F_t * prev_state.covariance.block<15,6>(0,15);
  cur_state.covariance.block<6,6>(15,15) = prev_state.covariance.block<6,6>(15,15);
}



void EKFImuVision::updatePnP(AugState& cur_state, AugState& prev_state) {

  // TODO
  // update by marker PnP measurements
  // cout << "==Begin PnP update===" << endl;
  
  Vec6 x_t, inno, v_t = Vec6::Zero(6);
  x_t = modelG1(prev_state.mean.segment(0,15), v_t);
  Mat6x21 C_t;
  C_t.setZero();
  C_t.block<6,15>(0,0)=jacobiG1x(prev_state.mean.segment(0,15), v_t);
  inno = cur_state.ut - C_t * prev_state.mean;
  inno(3) = ekf_imu_vision::clamp_euler_angle(inno(3));
  inno(4) = ekf_imu_vision::clamp_euler_angle(inno(4));
  inno(5) = ekf_imu_vision::clamp_euler_angle(inno(5));

  Mat21x6 K_t;
  K_t = prev_state.covariance*C_t.transpose()*(C_t*prev_state.covariance*C_t.transpose()+Rt1_).inverse();
  // K_xx = (C_t*prev_state.covariance*C_t.transpose()+Rt1_).lu().solve(C_t*prev_state.covariance).transpose();
  cur_state.mean = prev_state.mean + K_t*inno;
  cur_state.covariance = prev_state.covariance - K_t * C_t * prev_state.covariance;
  cur_state.mean(3) = ekf_imu_vision::clamp_euler_angle(cur_state.mean(3));
  cur_state.mean(4) = ekf_imu_vision::clamp_euler_angle(cur_state.mean(4));
  cur_state.mean(5) = ekf_imu_vision::clamp_euler_angle(cur_state.mean(5));
  cout << "PNP update result: " << cur_state.mean.transpose() << endl;
  
}

void EKFImuVision::updateVO(AugState& cur_state, AugState& prev_state) {
  // cout << "==Begin VO update===" << endl;
  Vec6 x_t, inno, v_t = Vec6::Zero(6); 
  x_t = modelG2(prev_state.mean, v_t);
  Mat6x21 C_t;
  C_t = jacobiG2x(prev_state.mean, v_t);
  inno = cur_state.ut - C_t * prev_state.mean;
  inno(3) = ekf_imu_vision::clamp_euler_angle(inno(3));
  inno(4) = ekf_imu_vision::clamp_euler_angle(inno(4));
  inno(5) = ekf_imu_vision::clamp_euler_angle(inno(5));
  Mat21x6 K_t;
  Mat6x6 W_t;
  W_t = jacobiG2v(prev_state.mean, v_t);
  K_t = prev_state.covariance*C_t.transpose()*(C_t*prev_state.covariance*C_t.transpose()+W_t* Rt2_*W_t.transpose()).inverse();
  cur_state.mean = prev_state.mean + K_t*inno;
  cur_state.covariance = prev_state.covariance - K_t*C_t*prev_state.covariance;
  cur_state.mean(3) = ekf_imu_vision::clamp_euler_angle(cur_state.mean(3));
  cur_state.mean(4) = ekf_imu_vision::clamp_euler_angle(cur_state.mean(4));
  cur_state.mean(5) = ekf_imu_vision::clamp_euler_angle(cur_state.mean(5));

}

void EKFImuVision::changeAugmentedState(AugState& state) {
  ROS_ERROR("----------------change keyframe------------------------");

  // TODO
  // change augmented state
  state.covariance = M_a_ * M_r_ * state.covariance * M_r_.transpose() * M_a_.transpose();
  state.mean = M_a_ * M_r_ * state.mean;
  state.key_frame_time_stamp = state.time_stamp;
}

bool EKFImuVision::processNewState(AugState& new_state, bool change_keyframe) {

  // TODO
  // process the new state
  // step 1: insert the new state into the queue and get the iterator to start to propagate (be careful about the change of key frame).
  // step 2: try to initialize the filter if it is not initialized.
  // step 3: repropagate from the iterator you extracted.
  // step 4: remove the old states.
  // step 5: publish the latest fused odom

  // if (change_keyframe == false){
  //   insertNewState(new_state);
  //   // type 1: pnp measurement 
  //   if (new_state.type == pnp){
  //     if (init_ == false){
  //       init_ = initFilter();
  //     } else {

  //     }

  //     }
  //   }

  // }
  return true;

}

deque<AugState>::iterator EKFImuVision::insertNewState(AugState& new_state){

  ros::Time time = new_state.time_stamp;
  if (aug_state_hist_.size()==0){
    aug_state_hist_.insert(aug_state_hist_.begin(),new_state);
    return aug_state_hist_.begin();
  }
  deque<AugState>::iterator state_it = aug_state_hist_.end();

  while (state_it != aug_state_hist_.begin() && time<=(*(state_it-1)).time_stamp){
    state_it--;
  }
  state_it = aug_state_hist_.insert(state_it, new_state);
  // update the lastest idx
  latest_idx[new_state.type] = distance(aug_state_hist_.begin(),state_it);

  for (int i =0; i <4; i++){
    if (i != new_state.type){
      if(latest_idx[i]>=latest_idx[new_state.type]){
        latest_idx[i]++;
      }
    }
  }
  // TODO
  // insert the new state to the queue
  // update the latest_idx of the type of the new state
  // return the iterator point to the new state in the queue 

  return state_it;

}

void EKFImuVision::repropagate(deque<AugState>::iterator& new_input_it, bool& init) {
  // cout << "==============repropagate================" << endl;
  cout <<"time" << new_input_it->time_stamp.toSec() << endl;;
  cout << new_input_it->type << endl;
  ROS_ERROR("%d", aug_state_hist_.size());
  for (deque<AugState>::iterator it = new_input_it+1; it != aug_state_hist_.end(); it++){
    if (it->type == imu){
      Vec6 tmp_ut = Vec6::Zero();
      predictIMU(*it, *(it-1), tmp_ut);
    } else if (it->type ==pnp)
    {
      updatePnP(*it, *(it-1));
    } else if (it->type == vo || it->type == keyframe) {
      updateVO(*it, *(it-1));
      
    }
    
  }
//  cout << "=============end repropagate================" << endl;

  // TODO
  // repropagate along the queue from the new input according to the type of the inputs / measurements
  // remember to consider the initialization case  

}

void EKFImuVision::removeOldState() {

  // TODO
  // remove the unnecessary old states to prevent the queue from becoming too long

  unsigned int remove_idx = min(min(latest_idx[imu], latest_idx[pnp]), latest_idx[keyframe]);

  aug_state_hist_.erase(aug_state_hist_.begin(), aug_state_hist_.begin() + remove_idx);

  for(int i = 0; i < 4; i++){
    latest_idx[i] -= remove_idx;
  }

  
}


bool EKFImuVision::initFilter() {

  // TODO
  // Initial the filter when a keyframe after marker PnP measurements is available
  // bool inited = initUsingPnP();
  return false;
}


bool EKFImuVision::initUsingPnP(deque<AugState>::iterator start_it) {

  // TODO
  // Initialize the absolute pose of the state in the queue using marker PnP measurement.
  // This is only step 1 of the initialization.
  if (start_it->type!=pnp){
    ROS_ERROR("it is not PnP state for initialization");
    return false;
  }
  //we treat the
  AugState start_state;
  start_state.mean = Vec21::Zero(21);
  start_state.covariance = 0.8*Mat21x21::Identity(21,21);
  updatePnP(*start_it, start_state);
  // std::cout << "init PnP state: " << start_it->mean.transpose() << std::endl;
  return true;
}


Vec3 EKFImuVision::rotation2Euler(const Mat3x3& R) {
  double phi   = asin(R(2, 1));
  double theta = atan2(-R(2, 0), R(2, 2));
  double psi   = atan2(-R(0, 1), R(1, 1));
  return Vec3(phi, theta, psi);
}


}  // namespace ekf_imu_visionchangeAugmentedState