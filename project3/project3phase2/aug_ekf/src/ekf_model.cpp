#include <ekf_model.h>

namespace ekf_imu_vision {
  inline double clamp_euler_angle(double theta)
{
    double result = atan2 (sin(theta),cos(theta)); 
    return result;
}

Vec15 modelF(const Vec15& x, const Vec6& u, const Vec12& n) {

  // TODO
  // return the model xdot = f(x,u,n)

  Vec3 ang_vel(u(0), u(1), u(2));
  Vec3 accel(u(3), u(4), u(5));
  Mat3x3 G, G_inv, R, R_dot, At_x2_x2;
  double phi =x(3);
  double theta=x(4);
  double psi=x(5);
  G << cos(theta), 0, -cos(phi)*sin(theta),
         0, 1, sin(phi),
         sin(theta), 0, cos(theta)*cos(phi);

  R << cos(psi)*cos(theta)- sin(phi)*sin(psi)*sin(theta), -cos(phi)*sin(psi), cos(psi)*sin(theta)+cos(theta)*sin(phi)*sin(psi),
      cos(theta)*sin(psi)+cos(psi)*sin(phi)*sin(theta), cos(phi)*cos(psi), sin(psi)*sin(theta)-cos(psi)*sin(phi)*cos(theta),
          -cos(phi)*sin(theta), sin(phi), cos(phi)*cos(theta);
  G_inv = G.inverse();
  Vec15 xdot;
  xdot.block<3,1>(0,0) = x.block<3,1>(6,0);
  xdot.block<3,1>(3,0) = G_inv * (ang_vel-x.segment(9,3)-n.segment(0,3));
  xdot.block<3,1>(6,0) = Vec3(0,0,-9.81) + R*(accel - x.segment(12,3)- n.segment(3,3));
  xdot.block<3,1>(9,0) = n.segment(6,3);
  xdot.block<3,1>(12,0) = n.segment(9,3);
  xdot(3) = clamp_euler_angle(xdot(3));
  xdot(4) = clamp_euler_angle(xdot(4));
  xdot(5) = clamp_euler_angle(xdot(5));

  //  1: x0:2 ~ x, y, z """
  //  2: x3:5 ~ phi theta psi """
  //  3: x6:8 ~ vx vy vz """
  //  4: x9:11 ~ bgx bgy bgz """
  //  5: x12:14 ~  bax bay baz """

  // u0:2 wmx, wmy, wmz
  // u3:5 amx, amy, amz

  // n0:2 ngx, ngy, ngz
  // n3:5 nax, nay, naz
  // n6:8 nbgx, nbgy, nbgz
  // n9:11 nbax, nbay, nbaz

  return xdot;
}

Mat15x15 jacobiFx(const Vec15& x, const Vec6& u, const Vec12& n) {

  // TODO
  // return the derivative wrt original state df/dx
  Vec3 ang_vel(u(0), u(1), u(2));
  Vec3 accel(u(3), u(4), u(5));
  Mat3x3 G, G_inv, R, R_dot, At_x2_x2;
  double phi =x(3);
  double theta=x(4);
  double psi=x(5);
  G << cos(theta), 0, -cos(phi)*sin(theta),
         0, 1, sin(phi),
         sin(theta), 0, cos(theta)*cos(phi);

  R << cos(psi)*cos(theta)- sin(phi)*sin(psi)*sin(theta), -cos(phi)*sin(psi), cos(psi)*sin(theta)+cos(theta)*sin(phi)*sin(psi),
      cos(theta)*sin(psi)+cos(psi)*sin(phi)*sin(theta), cos(phi)*cos(psi), sin(psi)*sin(theta)-cos(psi)*sin(phi)*cos(theta),
          -cos(phi)*sin(theta), sin(phi), cos(phi)*cos(theta);
  G_inv = G.inverse();
  double x1, x2, x3,x4,x5,x6,x7,x8,x9,x10,x11,x12,x13,x14,x15;
  double n1,n2,n3,n4,n5,n6,w1,w2,w3;
  x4 = x(3); x5 = x(4), x6 = x(5), x7 = x(6), x8 = x(7), x9 = x(8), x10 = x(9), x11 = x(10), x12 = x(11), x13 = x(12), x14 = x(13), x15 = x(14);
  n1 = 0, n2 = 0, n3 = 0, n4= 0, n5 = 0, n6 = 0;
  w1 = ang_vel(0), w2 = ang_vel(1), w3 = ang_vel(2);
  At_x2_x2 << 0, (sin(x5)*(n1 - w1 + x10))/(cos(x5)*cos(x5) + sin(x5)*sin(x5)) - (cos(x5)*(n3 - w3 + x12))/(cos(x5)*cos(x5) + sin(x5)*sin(x5)), 0, 
                (cos(x4)*cos(x5)*(n3 - w3 + x12))/(cos(x4)*cos(x5)*cos(x5) + cos(x4)*sin(x5)*sin(x5)) - (cos(x4)*sin(x5)*(n1 - w1 + x10))/(cos(x4)*cos(x5)*cos(x5) + cos(x4)*sin(x5)*sin(5)) + (cos(x5)*sin(x4)*(sin(x4)*cos(x5)*cos(x5) + sin(x4)*sin(x5)*sin(x5))*(n3 - w3 + x12))/(cos(x4)*cos(x5)*cos(x5) + cos(x4)*sin(x5)*sin(x5))*(cos(x4)*cos(x5)*cos(x5) + cos(x4)*sin(x5)*sin(x5)) - (sin(x4)*sin(x5)*(sin(x4)*cos(x5)*cos(x5) + sin(x4)*sin(x5)*sin(x5))*(n1 - w1 + x10))/(cos(x4)*cos(x5)*cos(x5) + cos(x4)*sin(x5)*sin(x5))*(cos(x4)*cos(x5)*cos(x5) + cos(x4)*sin(x5)*sin(x5)), 
                - (sin(x4)*sin(x5)*(n3 - w3 + x12))/(cos(x4)*cos(x5)*cos(x5) + cos(x4)*sin(x5)*sin(x5)) - (cos(x5)*sin(x4)*(n1 - w1 + x10))/(cos(x4)*cos(x5)*cos(x5) + cos(x4)*sin(x5)*sin(x5)), 0,
              (sin(x5)*(sin(x4)*cos(x5)*cos(x5) + sin(x4)*sin(x5)*sin(x5))*(n1 - w1 + x10))/(cos(x4)*cos(x5)*cos(x5) + cos(x4)*sin(x5)*sin(x5))*(cos(x4)*cos(x5)*cos(x5) + cos(x4)*sin(x5)*sin(x5)) - (cos(x5)*(sin(x4)*cos(x5)*cos(x5) + sin(x4)*sin(x5)*sin(x5))*(n3 - w3 + x12))/(cos(x4)*cos(x5)*cos(x5) + cos(x4)*sin(x5)*sin(x5))*(cos(x4)*cos(x5)*cos(x5) + cos(x4)*sin(x5)*sin(x5)), (cos(x5)*(n1 - w1 + x10))/(cos(x4)*cos(x5)*cos(x5) + cos(x4)*sin(x5)*sin(x5)) + (sin(x5)*(n3 - w3 + x12))/(cos(x4)*cos(x5)*cos(x5) + cos(x4)*sin(x5)*sin(x5)), 0;

  R_dot << accel(1)*sin(phi)*sin(psi) + accel(2)*cos(phi)*cos(theta)*sin(psi) - accel(0)*cos(phi)*sin(theta)*sin(psi), accel(2)*(cos(theta)*cos(psi) - sin(phi)*sin(theta)*sin(psi)) - accel(0)*(cos(psi)*sin(theta) + cos(theta)*sin(phi)*sin(psi)), - accel(0)*(cos(theta)*sin(psi) + cos(psi)*sin(phi)*sin(theta)) - accel(2)*(sin(theta)*sin(psi) - cos(theta)*cos(psi)*sin(phi)) - accel(1)*cos(phi)*cos(psi),
          accel(0)*cos(phi)*cos(psi)*sin(theta) - accel(2)*cos(phi)*cos(theta)*cos(psi) - accel(1)*cos(psi)*sin(phi), accel(2)*(cos(theta)*sin(psi) + cos(psi)*sin(phi)*sin(theta)) - accel(0)*(sin(theta)*sin(psi) - cos(theta)*cos(psi)*sin(phi)),   accel(0)*(cos(theta)*cos(psi) - sin(phi)*sin(theta)*sin(psi)) + accel(2)*(cos(psi)*sin(theta) + cos(theta)*sin(phi)*sin(psi)) - accel(1)*cos(phi)*sin(psi),
          accel(1)*cos(phi) - accel(2)*cos(theta)*sin(phi) + accel(0)*sin(phi)*sin(theta), - accel(0)*cos(phi)*cos(theta) - accel(2)*cos(phi)*sin(theta), 0;

  Mat15x15 At = Mat15x15::Zero(15,15);
  At.block<3,3>(0,6) = Mat3x3::Identity(3,3);
  At.block<3,3>(3,3) << At_x2_x2;
  At.block<3,3>(6,3) << R_dot;
  At.block<3,3>(3,9) << -G_inv;
  At.block<3,3>(6,12) << -R;
  return At;

}

Mat15x12 jacobiFn(const Vec15& x, const Vec6& u, const Vec12& n) {

  // TODO
  // return the derivative wrt noise df/dn
  Vec3 ang_vel(u(0), u(1), u(2));
  Vec3 accel(u(3), u(4), u(5));
  Mat3x3 G, G_inv, R, R_dot, At_x2_x2;
  double phi =x(3);
  double theta=x(4);
  double psi=x(5);
  G << cos(theta), 0, -cos(phi)*sin(theta),
         0, 1, sin(phi),
         sin(theta), 0, cos(theta)*cos(phi);

  R << cos(psi)*cos(theta)- sin(phi)*sin(psi)*sin(theta), -cos(phi)*sin(psi), cos(psi)*sin(theta)+cos(theta)*sin(phi)*sin(psi),
      cos(theta)*sin(psi)+cos(psi)*sin(phi)*sin(theta), cos(phi)*cos(psi), sin(psi)*sin(theta)-cos(psi)*sin(phi)*cos(theta),
          -cos(phi)*sin(theta), sin(phi), cos(phi)*cos(theta);
  G_inv = G.inverse();
  Mat15x12 Ut =Mat15x12::Zero(15,12);
  Ut.block<3,3>(3,0) << -G_inv;
  Ut.block<3,3>(6,3) << -R;
  Ut.block<6,6>(9,6) << Mat6x6::Identity(6,6);
  return Ut;
}

/* ============================== model of PnP ============================== */

Vec6 modelG1(const Vec15& x, const Vec6& v) {

  // TODO: the down looking camera
  // FIXME: assume measurement of body in world frame. x is the measurement from PnP
  // return the model g(x,v), where x = x_origin
  Vec6 z_t;
  z_t=x.segment(0,6);
  return z_t;
}

Mat6x15 jacobiG1x(const Vec15& x, const Vec6& v) {

  // TODO
  // return the derivative wrt original state dz/dx, where x = x_origin

  Mat6x15 Ct;
  Ct.setZero();
  Ct.block<6, 6>(0, 0) = Eigen::MatrixXd::Identity(6, 6);

  return Ct;
}

Mat6x6 jacobiG1v(const Vec15& x, const Vec6& v) {

  // TODO;
  // return the derivative wrt noise dz/dv

  Mat6x6 I6;

  I6.setIdentity();

  return I6;
}

/* ============================== model of stereo VO relative pose ============================== */
// relative pose: T_{kc}

// Vec6 modelG2(const Vec21& x, const Vec6& v) {

//   // TODO
//   // return the model g(x,v), where x = (x_origin, x_augmented)

//   Vec6 zt;

//   Mat3x3 R_qk=rpy2Rotmat(x(18),x(19),x(20));

//   zt.segment(0,3) = R_qk.transpose() * (x.segment(0, 3)- x.segment(15, 3));
//   zt.segment(3,3) = rot2Euler(R_qk.transpose() * rpy2Rotmat(x(3),x(4),x(5)));

//   clampEuler(zt(3));
//   clampEuler(zt(4));
//   clampEuler(zt(5));

//   return zt;
// }
Vec6 modelG2(const Vec21 &state, const Vec6 &v)
  {

    // TODO
    // return the model g(x,v), where x = (x_origin, x_augmented)

    Vec6 zt;
    Eigen::Quaterniond q_wk, q_wb;
    q_wb = Eigen::AngleAxisd(state(5), Eigen::Vector3d::UnitZ()) *
           Eigen::AngleAxisd(state(3), Eigen::Vector3d::UnitX()) *
           Eigen::AngleAxisd(state(4), Eigen::Vector3d::UnitY());

    q_wk = Eigen::AngleAxisd(state(20), Eigen::Vector3d::UnitZ()) *
           Eigen::AngleAxisd(state(18), Eigen::Vector3d::UnitX()) *
           Eigen::AngleAxisd(state(19), Eigen::Vector3d::UnitY());

    Eigen::Matrix3d R_wk = q_wk.matrix();

    Eigen::Matrix3d R_kb = R_wk.transpose() * q_wb.matrix();

    zt.segment<3>(0) = R_wk.transpose() * (state.head(3) - state.segment<3>(15));

    zt(3) = asin(R_kb(2, 1));
    zt(4) = atan2(-R_kb(2, 0), R_kb(2, 2));
    zt(5) = atan2(-R_kb(0, 1), R_kb(1, 1));

    return zt;
  }
Mat6x21 jacobiG2x(const Vec21 &state, const Vec6 &noise)
  {

    // TODO
    // return the derivative wrt original state dz/dx, where x = (x_origin, x_augmented)
    double x_b = state(0);
    double y_b = state(1);
    double z_b = state(2);
    double phi_b = state(3);
    double theta_b = state(4);
    double psi_b = state(5);

    double x_k = state(15);
    double y_k = state(16);
    double z_k = state(17);
    double phi_k = state(18);
    double theta_k = state(19);
    double psi_k = state(20);
    Mat6x21 Ct = Mat6x21::Zero();

    Ct(0, 0) = cos(psi_k) * cos(theta_k) - sin(phi_k) * sin(psi_k) * sin(theta_k);
    Ct(0, 1) = cos(theta_k) * sin(psi_k) + cos(psi_k) * sin(phi_k) * sin(theta_k);
    Ct(0, 2) = -cos(phi_k) * sin(theta_k);
    Ct(0, 15) = -cos(psi_k) * cos(theta_k) + sin(phi_k) * sin(psi_k) * sin(theta_k);
    Ct(0, 16) = -cos(theta_k) * sin(psi_k) - cos(psi_k) * sin(phi_k) * sin(theta_k);
    Ct(0, 17) = cos(phi_k) * sin(theta_k);
    Ct(0, 18) = sin(phi_k) * sin(theta_k) * (z_b - z_k) + cos(phi_k) * cos(psi_k) * sin(theta_k) * (y_b - y_k) - cos(phi_k) * sin(psi_k) * sin(theta_k) * (x_b - x_k);
    Ct(0, 19) = -(cos(psi_k) * sin(theta_k) + cos(theta_k) * sin(phi_k) * sin(psi_k)) * (x_b - x_k) - (sin(psi_k) * sin(theta_k) - cos(psi_k) * cos(theta_k) * sin(phi_k)) * (y_b - y_k) - cos(phi_k) * cos(theta_k) * (z_b - z_k);
    Ct(0, 20) = -(cos(theta_k) * sin(psi_k) + cos(psi_k) * sin(phi_k) * sin(theta_k)) * (x_b - x_k) + (cos(psi_k) * cos(theta_k) - sin(phi_k) * sin(psi_k) * sin(theta_k)) * (y_b - y_k);
    Ct(1, 0) = -cos(phi_k) * sin(psi_k);
    Ct(1, 1) = cos(phi_k) * cos(psi_k);
    Ct(1, 2) = sin(phi_k);
    Ct(1, 15) = cos(phi_k) * sin(psi_k);
    Ct(1, 16) = -cos(phi_k) * cos(psi_k);
    Ct(1, 17) = -sin(phi_k);
    Ct(1, 18) = cos(phi_k) * (z_b - z_k) - cos(psi_k) * sin(phi_k) * (y_b - y_k) + sin(phi_k) * sin(psi_k) * (x_b - x_k);
    Ct(1, 20) = -cos(phi_k) * cos(psi_k) * (x_b - x_k) - cos(phi_k) * sin(psi_k) * (y_b - y_k);
    Ct(2, 0) = cos(psi_k) * sin(theta_k) + cos(theta_k) * sin(phi_k) * sin(psi_k);
    Ct(2, 1) = sin(psi_k) * sin(theta_k) - cos(psi_k) * cos(theta_k) * sin(phi_k);
    Ct(2, 2) = cos(phi_k) * cos(theta_k);
    Ct(2, 15) = -cos(psi_k) * sin(theta_k) - cos(theta_k) * sin(phi_k) * sin(psi_k);
    Ct(2, 16) = -sin(psi_k) * sin(theta_k) + cos(psi_k) * cos(theta_k) * sin(phi_k);
    Ct(2, 17) = -cos(phi_k) * cos(theta_k);
    Ct(2, 18) = -cos(theta_k) * sin(phi_k) * (z_b - z_k) - cos(phi_k) * cos(psi_k) * cos(theta_k) * (y_b - y_k) + cos(phi_k) * cos(theta_k) * sin(psi_k) * (x_b - x_k);
    Ct(2, 19) = (cos(psi_k) * cos(theta_k) - sin(phi_k) * sin(psi_k) * sin(theta_k)) * (x_b - x_k) + (cos(theta_k) * sin(psi_k) + cos(psi_k) * sin(phi_k) * sin(theta_k)) * (y_b - y_k) - cos(phi_k) * sin(theta_k) * (z_b - z_k);
    Ct(2, 20) = -(sin(psi_k) * sin(theta_k) - cos(psi_k) * cos(theta_k) * sin(phi_k)) * (x_b - x_k) + (cos(psi_k) * sin(theta_k) + cos(theta_k) * sin(phi_k) * sin(psi_k)) * (y_b - y_k);
    Ct(3, 3) = 1.0 / sqrt(-pow(cos(phi_b) * cos(psi_b) * (sin(psi_k) * sin(theta_k) - cos(psi_k) * cos(theta_k) * sin(phi_k)) - cos(phi_b) * sin(psi_b) * (cos(psi_k) * sin(theta_k) + cos(theta_k) * sin(phi_k) * sin(psi_k)) + cos(phi_k) * cos(theta_k) * sin(phi_b), 2.0) + 1.0) * (-cos(psi_b) * sin(phi_b) * (sin(psi_k) * sin(theta_k) - cos(psi_k) * cos(theta_k) * sin(phi_k)) + sin(phi_b) * sin(psi_b) * (cos(psi_k) * sin(theta_k) + cos(theta_k) * sin(phi_k) * sin(psi_k)) + cos(phi_b) * cos(phi_k) * cos(theta_k));
    Ct(3, 5) = -1.0 / sqrt(-pow(cos(phi_b) * cos(psi_b) * (sin(psi_k) * sin(theta_k) - cos(psi_k) * cos(theta_k) * sin(phi_k)) - cos(phi_b) * sin(psi_b) * (cos(psi_k) * sin(theta_k) + cos(theta_k) * sin(phi_k) * sin(psi_k)) + cos(phi_k) * cos(theta_k) * sin(phi_b), 2.0) + 1.0) * (cos(phi_b) * cos(psi_b) * (cos(psi_k) * sin(theta_k) + cos(theta_k) * sin(phi_k) * sin(psi_k)) + cos(phi_b) * sin(psi_b) * (sin(psi_k) * sin(theta_k) - cos(psi_k) * cos(theta_k) * sin(phi_k)));
    Ct(3, 18) = -cos(theta_k) * 1.0 / sqrt(-pow(cos(phi_b) * cos(psi_b) * (sin(psi_k) * sin(theta_k) - cos(psi_k) * cos(theta_k) * sin(phi_k)) - cos(phi_b) * sin(psi_b) * (cos(psi_k) * sin(theta_k) + cos(theta_k) * sin(phi_k) * sin(psi_k)) + cos(phi_k) * cos(theta_k) * sin(phi_b), 2.0) + 1.0) * (sin(phi_b) * sin(phi_k) + cos(phi_b) * cos(phi_k) * cos(psi_b) * cos(psi_k) + cos(phi_b) * cos(phi_k) * sin(psi_b) * sin(psi_k));
    Ct(3, 19) = -1.0 / sqrt(-pow(cos(phi_b) * cos(psi_b) * (sin(psi_k) * sin(theta_k) - cos(psi_k) * cos(theta_k) * sin(phi_k)) - cos(phi_b) * sin(psi_b) * (cos(psi_k) * sin(theta_k) + cos(theta_k) * sin(phi_k) * sin(psi_k)) + cos(phi_k) * cos(theta_k) * sin(phi_b), 2.0) + 1.0) * (-cos(phi_b) * cos(psi_b) * (cos(theta_k) * sin(psi_k) + cos(psi_k) * sin(phi_k) * sin(theta_k)) + cos(phi_b) * sin(psi_b) * (cos(psi_k) * cos(theta_k) - sin(phi_k) * sin(psi_k) * sin(theta_k)) + cos(phi_k) * sin(phi_b) * sin(theta_k));
    Ct(3, 20) = 1.0 / sqrt(-pow(cos(phi_b) * cos(psi_b) * (sin(psi_k) * sin(theta_k) - cos(psi_k) * cos(theta_k) * sin(phi_k)) - cos(phi_b) * sin(psi_b) * (cos(psi_k) * sin(theta_k) + cos(theta_k) * sin(phi_k) * sin(psi_k)) + cos(phi_k) * cos(theta_k) * sin(phi_b), 2.0) + 1.0) * (cos(phi_b) * cos(psi_b) * (cos(psi_k) * sin(theta_k) + cos(theta_k) * sin(phi_k) * sin(psi_k)) + cos(phi_b) * sin(psi_b) * (sin(psi_k) * sin(theta_k) - cos(psi_k) * cos(theta_k) * sin(phi_k)));
    Ct(4, 3) = (pow((cos(psi_b) * sin(theta_b) + cos(theta_b) * sin(phi_b) * sin(psi_b)) * (cos(psi_k) * sin(theta_k) + cos(theta_k) * sin(phi_k) * sin(psi_k)) + (sin(psi_b) * sin(theta_b) - cos(psi_b) * cos(theta_b) * sin(phi_b)) * (sin(psi_k) * sin(theta_k) - cos(psi_k) * cos(theta_k) * sin(phi_k)) + cos(phi_b) * cos(phi_k) * cos(theta_b) * cos(theta_k), 2.0) * (sin(psi_b) * sin(psi_k) * sin(theta_k) + cos(psi_b) * cos(psi_k) * sin(theta_k) + cos(psi_b) * cos(theta_k) * sin(phi_k) * sin(psi_k) - cos(psi_k) * cos(theta_k) * sin(phi_k) * sin(psi_b)) * (-cos(phi_k) * cos(theta_k) * sin(phi_b) - cos(phi_b) * cos(psi_b) * sin(psi_k) * sin(theta_k) + cos(phi_b) * cos(psi_k) * sin(psi_b) * sin(theta_k) + cos(phi_b) * cos(psi_b) * cos(psi_k) * cos(theta_k) * sin(phi_k) + cos(phi_b) * cos(theta_k) * sin(phi_k) * sin(psi_b) * sin(psi_k)) * 1.0 / pow(cos(phi_b) * cos(phi_k) * cos(theta_b) * cos(theta_k) + cos(psi_b) * cos(psi_k) * sin(theta_b) * sin(theta_k) + sin(psi_b) * sin(psi_k) * sin(theta_b) * sin(theta_k) - cos(psi_b) * cos(theta_b) * sin(phi_b) * sin(psi_k) * sin(theta_k) + cos(psi_k) * cos(theta_b) * sin(phi_b) * sin(psi_b) * sin(theta_k) + cos(psi_b) * cos(theta_k) * sin(phi_k) * sin(psi_k) * sin(theta_b) - cos(psi_k) * cos(theta_k) * sin(phi_k) * sin(psi_b) * sin(theta_b) + cos(psi_b) * cos(psi_k) * cos(theta_b) * cos(theta_k) * sin(phi_b) * sin(phi_k) + cos(theta_b) * cos(theta_k) * sin(phi_b) * sin(phi_k) * sin(psi_b) * sin(psi_k), 2.0)) / (pow((cos(psi_b) * sin(theta_b) + cos(theta_b) * sin(phi_b) * sin(psi_b)) * (cos(psi_k) * sin(theta_k) + cos(theta_k) * sin(phi_k) * sin(psi_k)) + (sin(psi_b) * sin(theta_b) - cos(psi_b) * cos(theta_b) * sin(phi_b)) * (sin(psi_k) * sin(theta_k) - cos(psi_k) * cos(theta_k) * sin(phi_k)) + cos(phi_b) * cos(phi_k) * cos(theta_b) * cos(theta_k), 2.0) + pow((cos(theta_b) * sin(psi_b) + cos(psi_b) * sin(phi_b) * sin(theta_b)) * (sin(psi_k) * sin(theta_k) - cos(psi_k) * cos(theta_k) * sin(phi_k)) + (cos(psi_b) * cos(theta_b) - sin(phi_b) * sin(psi_b) * sin(theta_b)) * (cos(psi_k) * sin(theta_k) + cos(theta_k) * sin(phi_k) * sin(psi_k)) - cos(phi_b) * cos(phi_k) * cos(theta_k) * sin(theta_b), 2.0));
    Ct(4, 4) = ((1.0 / pow((cos(psi_b) * sin(theta_b) + cos(theta_b) * sin(phi_b) * sin(psi_b)) * (cos(psi_k) * sin(theta_k) + cos(theta_k) * sin(phi_k) * sin(psi_k)) + (sin(psi_b) * sin(theta_b) - cos(psi_b) * cos(theta_b) * sin(phi_b)) * (sin(psi_k) * sin(theta_k) - cos(psi_k) * cos(theta_k) * sin(phi_k)) + cos(phi_b) * cos(phi_k) * cos(theta_b) * cos(theta_k), 2.0) * pow((cos(theta_b) * sin(psi_b) + cos(psi_b) * sin(phi_b) * sin(theta_b)) * (sin(psi_k) * sin(theta_k) - cos(psi_k) * cos(theta_k) * sin(phi_k)) + (cos(psi_b) * cos(theta_b) - sin(phi_b) * sin(psi_b) * sin(theta_b)) * (cos(psi_k) * sin(theta_k) + cos(theta_k) * sin(phi_k) * sin(psi_k)) - cos(phi_b) * cos(phi_k) * cos(theta_k) * sin(theta_b), 2.0) + 1.0) * pow((cos(psi_b) * sin(theta_b) + cos(theta_b) * sin(phi_b) * sin(psi_b)) * (cos(psi_k) * sin(theta_k) + cos(theta_k) * sin(phi_k) * sin(psi_k)) + (sin(psi_b) * sin(theta_b) - cos(psi_b) * cos(theta_b) * sin(phi_b)) * (sin(psi_k) * sin(theta_k) - cos(psi_k) * cos(theta_k) * sin(phi_k)) + cos(phi_b) * cos(phi_k) * cos(theta_b) * cos(theta_k), 2.0)) / (pow((cos(psi_b) * sin(theta_b) + cos(theta_b) * sin(phi_b) * sin(psi_b)) * (cos(psi_k) * sin(theta_k) + cos(theta_k) * sin(phi_k) * sin(psi_k)) + (sin(psi_b) * sin(theta_b) - cos(psi_b) * cos(theta_b) * sin(phi_b)) * (sin(psi_k) * sin(theta_k) - cos(psi_k) * cos(theta_k) * sin(phi_k)) + cos(phi_b) * cos(phi_k) * cos(theta_b) * cos(theta_k), 2.0) + pow((cos(theta_b) * sin(psi_b) + cos(psi_b) * sin(phi_b) * sin(theta_b)) * (sin(psi_k) * sin(theta_k) - cos(psi_k) * cos(theta_k) * sin(phi_k)) + (cos(psi_b) * cos(theta_b) - sin(phi_b) * sin(psi_b) * sin(theta_b)) * (cos(psi_k) * sin(theta_k) + cos(theta_k) * sin(phi_k) * sin(psi_k)) - cos(phi_b) * cos(phi_k) * cos(theta_k) * sin(theta_b), 2.0));
    Ct(4, 5) = ((((cos(theta_b) * sin(psi_b) + cos(psi_b) * sin(phi_b) * sin(theta_b)) * (cos(psi_k) * sin(theta_k) + cos(theta_k) * sin(phi_k) * sin(psi_k)) - (cos(psi_b) * cos(theta_b) - sin(phi_b) * sin(psi_b) * sin(theta_b)) * (sin(psi_k) * sin(theta_k) - cos(psi_k) * cos(theta_k) * sin(phi_k))) / ((cos(psi_b) * sin(theta_b) + cos(theta_b) * sin(phi_b) * sin(psi_b)) * (cos(psi_k) * sin(theta_k) + cos(theta_k) * sin(phi_k) * sin(psi_k)) + (sin(psi_b) * sin(theta_b) - cos(psi_b) * cos(theta_b) * sin(phi_b)) * (sin(psi_k) * sin(theta_k) - cos(psi_k) * cos(theta_k) * sin(phi_k)) + cos(phi_b) * cos(phi_k) * cos(theta_b) * cos(theta_k)) + ((cos(psi_b) * sin(theta_b) + cos(theta_b) * sin(phi_b) * sin(psi_b)) * (sin(psi_k) * sin(theta_k) - cos(psi_k) * cos(theta_k) * sin(phi_k)) - (sin(psi_b) * sin(theta_b) - cos(psi_b) * cos(theta_b) * sin(phi_b)) * (cos(psi_k) * sin(theta_k) + cos(theta_k) * sin(phi_k) * sin(psi_k))) * 1.0 / pow((cos(psi_b) * sin(theta_b) + cos(theta_b) * sin(phi_b) * sin(psi_b)) * (cos(psi_k) * sin(theta_k) + cos(theta_k) * sin(phi_k) * sin(psi_k)) + (sin(psi_b) * sin(theta_b) - cos(psi_b) * cos(theta_b) * sin(phi_b)) * (sin(psi_k) * sin(theta_k) - cos(psi_k) * cos(theta_k) * sin(phi_k)) + cos(phi_b) * cos(phi_k) * cos(theta_b) * cos(theta_k), 2.0) * ((cos(theta_b) * sin(psi_b) + cos(psi_b) * sin(phi_b) * sin(theta_b)) * (sin(psi_k) * sin(theta_k) - cos(psi_k) * cos(theta_k) * sin(phi_k)) + (cos(psi_b) * cos(theta_b) - sin(phi_b) * sin(psi_b) * sin(theta_b)) * (cos(psi_k) * sin(theta_k) + cos(theta_k) * sin(phi_k) * sin(psi_k)) - cos(phi_b) * cos(phi_k) * cos(theta_k) * sin(theta_b))) * pow((cos(psi_b) * sin(theta_b) + cos(theta_b) * sin(phi_b) * sin(psi_b)) * (cos(psi_k) * sin(theta_k) + cos(theta_k) * sin(phi_k) * sin(psi_k)) + (sin(psi_b) * sin(theta_b) - cos(psi_b) * cos(theta_b) * sin(phi_b)) * (sin(psi_k) * sin(theta_k) - cos(psi_k) * cos(theta_k) * sin(phi_k)) + cos(phi_b) * cos(phi_k) * cos(theta_b) * cos(theta_k), 2.0)) / (pow((cos(psi_b) * sin(theta_b) + cos(theta_b) * sin(phi_b) * sin(psi_b)) * (cos(psi_k) * sin(theta_k) + cos(theta_k) * sin(phi_k) * sin(psi_k)) + (sin(psi_b) * sin(theta_b) - cos(psi_b) * cos(theta_b) * sin(phi_b)) * (sin(psi_k) * sin(theta_k) - cos(psi_k) * cos(theta_k) * sin(phi_k)) + cos(phi_b) * cos(phi_k) * cos(theta_b) * cos(theta_k), 2.0) + pow((cos(theta_b) * sin(psi_b) + cos(psi_b) * sin(phi_b) * sin(theta_b)) * (sin(psi_k) * sin(theta_k) - cos(psi_k) * cos(theta_k) * sin(phi_k)) + (cos(psi_b) * cos(theta_b) - sin(phi_b) * sin(psi_b) * sin(theta_b)) * (cos(psi_k) * sin(theta_k) + cos(theta_k) * sin(phi_k) * sin(psi_k)) - cos(phi_b) * cos(phi_k) * cos(theta_k) * sin(theta_b), 2.0));
    Ct(4, 18) = -(((-cos(phi_k) * cos(psi_k) * cos(theta_k) * (cos(theta_b) * sin(psi_b) + cos(psi_b) * sin(phi_b) * sin(theta_b)) + cos(phi_k) * cos(theta_k) * sin(psi_k) * (cos(psi_b) * cos(theta_b) - sin(phi_b) * sin(psi_b) * sin(theta_b)) + cos(phi_b) * cos(theta_k) * sin(phi_k) * sin(theta_b)) / ((cos(psi_b) * sin(theta_b) + cos(theta_b) * sin(phi_b) * sin(psi_b)) * (cos(psi_k) * sin(theta_k) + cos(theta_k) * sin(phi_k) * sin(psi_k)) + (sin(psi_b) * sin(theta_b) - cos(psi_b) * cos(theta_b) * sin(phi_b)) * (sin(psi_k) * sin(theta_k) - cos(psi_k) * cos(theta_k) * sin(phi_k)) + cos(phi_b) * cos(phi_k) * cos(theta_b) * cos(theta_k)) + 1.0 / pow((cos(psi_b) * sin(theta_b) + cos(theta_b) * sin(phi_b) * sin(psi_b)) * (cos(psi_k) * sin(theta_k) + cos(theta_k) * sin(phi_k) * sin(psi_k)) + (sin(psi_b) * sin(theta_b) - cos(psi_b) * cos(theta_b) * sin(phi_b)) * (sin(psi_k) * sin(theta_k) - cos(psi_k) * cos(theta_k) * sin(phi_k)) + cos(phi_b) * cos(phi_k) * cos(theta_b) * cos(theta_k), 2.0) * ((cos(theta_b) * sin(psi_b) + cos(psi_b) * sin(phi_b) * sin(theta_b)) * (sin(psi_k) * sin(theta_k) - cos(psi_k) * cos(theta_k) * sin(phi_k)) + (cos(psi_b) * cos(theta_b) - sin(phi_b) * sin(psi_b) * sin(theta_b)) * (cos(psi_k) * sin(theta_k) + cos(theta_k) * sin(phi_k) * sin(psi_k)) - cos(phi_b) * cos(phi_k) * cos(theta_k) * sin(theta_b)) * (cos(phi_k) * cos(psi_k) * cos(theta_k) * (sin(psi_b) * sin(theta_b) - cos(psi_b) * cos(theta_b) * sin(phi_b)) - cos(phi_k) * cos(theta_k) * sin(psi_k) * (cos(psi_b) * sin(theta_b) + cos(theta_b) * sin(phi_b) * sin(psi_b)) + cos(phi_b) * cos(theta_b) * cos(theta_k) * sin(phi_k))) * pow((cos(psi_b) * sin(theta_b) + cos(theta_b) * sin(phi_b) * sin(psi_b)) * (cos(psi_k) * sin(theta_k) + cos(theta_k) * sin(phi_k) * sin(psi_k)) + (sin(psi_b) * sin(theta_b) - cos(psi_b) * cos(theta_b) * sin(phi_b)) * (sin(psi_k) * sin(theta_k) - cos(psi_k) * cos(theta_k) * sin(phi_k)) + cos(phi_b) * cos(phi_k) * cos(theta_b) * cos(theta_k), 2.0)) / (pow((cos(psi_b) * sin(theta_b) + cos(theta_b) * sin(phi_b) * sin(psi_b)) * (cos(psi_k) * sin(theta_k) + cos(theta_k) * sin(phi_k) * sin(psi_k)) + (sin(psi_b) * sin(theta_b) - cos(psi_b) * cos(theta_b) * sin(phi_b)) * (sin(psi_k) * sin(theta_k) - cos(psi_k) * cos(theta_k) * sin(phi_k)) + cos(phi_b) * cos(phi_k) * cos(theta_b) * cos(theta_k), 2.0) + pow((cos(theta_b) * sin(psi_b) + cos(psi_b) * sin(phi_b) * sin(theta_b)) * (sin(psi_k) * sin(theta_k) - cos(psi_k) * cos(theta_k) * sin(phi_k)) + (cos(psi_b) * cos(theta_b) - sin(phi_b) * sin(psi_b) * sin(theta_b)) * (cos(psi_k) * sin(theta_k) + cos(theta_k) * sin(phi_k) * sin(psi_k)) - cos(phi_b) * cos(phi_k) * cos(theta_k) * sin(theta_b), 2.0));
    Ct(4, 19) = -((sin(phi_b) * sin(phi_k) + cos(phi_b) * cos(phi_k) * cos(psi_b) * cos(psi_k) + cos(phi_b) * cos(phi_k) * sin(psi_b) * sin(psi_k)) * pow((cos(psi_b) * sin(theta_b) + cos(theta_b) * sin(phi_b) * sin(psi_b)) * (cos(psi_k) * sin(theta_k) + cos(theta_k) * sin(phi_k) * sin(psi_k)) + (sin(psi_b) * sin(theta_b) - cos(psi_b) * cos(theta_b) * sin(phi_b)) * (sin(psi_k) * sin(theta_k) - cos(psi_k) * cos(theta_k) * sin(phi_k)) + cos(phi_b) * cos(phi_k) * cos(theta_b) * cos(theta_k), 2.0) * 1.0 / pow(cos(phi_b) * cos(phi_k) * cos(theta_b) * cos(theta_k) + cos(psi_b) * cos(psi_k) * sin(theta_b) * sin(theta_k) + sin(psi_b) * sin(psi_k) * sin(theta_b) * sin(theta_k) - cos(psi_b) * cos(theta_b) * sin(phi_b) * sin(psi_k) * sin(theta_k) + cos(psi_k) * cos(theta_b) * sin(phi_b) * sin(psi_b) * sin(theta_k) + cos(psi_b) * cos(theta_k) * sin(phi_k) * sin(psi_k) * sin(theta_b) - cos(psi_k) * cos(theta_k) * sin(phi_k) * sin(psi_b) * sin(theta_b) + cos(psi_b) * cos(psi_k) * cos(theta_b) * cos(theta_k) * sin(phi_b) * sin(phi_k) + cos(theta_b) * cos(theta_k) * sin(phi_b) * sin(phi_k) * sin(psi_b) * sin(psi_k), 2.0)) / (pow((cos(psi_b) * sin(theta_b) + cos(theta_b) * sin(phi_b) * sin(psi_b)) * (cos(psi_k) * sin(theta_k) + cos(theta_k) * sin(phi_k) * sin(psi_k)) + (sin(psi_b) * sin(theta_b) - cos(psi_b) * cos(theta_b) * sin(phi_b)) * (sin(psi_k) * sin(theta_k) - cos(psi_k) * cos(theta_k) * sin(phi_k)) + cos(phi_b) * cos(phi_k) * cos(theta_b) * cos(theta_k), 2.0) + pow((cos(theta_b) * sin(psi_b) + cos(psi_b) * sin(phi_b) * sin(theta_b)) * (sin(psi_k) * sin(theta_k) - cos(psi_k) * cos(theta_k) * sin(phi_k)) + (cos(psi_b) * cos(theta_b) - sin(phi_b) * sin(psi_b) * sin(theta_b)) * (cos(psi_k) * sin(theta_k) + cos(theta_k) * sin(phi_k) * sin(psi_k)) - cos(phi_b) * cos(phi_k) * cos(theta_k) * sin(theta_b), 2.0));
    Ct(4, 20) = -((((cos(theta_b) * sin(psi_b) + cos(psi_b) * sin(phi_b) * sin(theta_b)) * (cos(psi_k) * sin(theta_k) + cos(theta_k) * sin(phi_k) * sin(psi_k)) - (cos(psi_b) * cos(theta_b) - sin(phi_b) * sin(psi_b) * sin(theta_b)) * (sin(psi_k) * sin(theta_k) - cos(psi_k) * cos(theta_k) * sin(phi_k))) / ((cos(psi_b) * sin(theta_b) + cos(theta_b) * sin(phi_b) * sin(psi_b)) * (cos(psi_k) * sin(theta_k) + cos(theta_k) * sin(phi_k) * sin(psi_k)) + (sin(psi_b) * sin(theta_b) - cos(psi_b) * cos(theta_b) * sin(phi_b)) * (sin(psi_k) * sin(theta_k) - cos(psi_k) * cos(theta_k) * sin(phi_k)) + cos(phi_b) * cos(phi_k) * cos(theta_b) * cos(theta_k)) + ((cos(psi_b) * sin(theta_b) + cos(theta_b) * sin(phi_b) * sin(psi_b)) * (sin(psi_k) * sin(theta_k) - cos(psi_k) * cos(theta_k) * sin(phi_k)) - (sin(psi_b) * sin(theta_b) - cos(psi_b) * cos(theta_b) * sin(phi_b)) * (cos(psi_k) * sin(theta_k) + cos(theta_k) * sin(phi_k) * sin(psi_k))) * 1.0 / pow((cos(psi_b) * sin(theta_b) + cos(theta_b) * sin(phi_b) * sin(psi_b)) * (cos(psi_k) * sin(theta_k) + cos(theta_k) * sin(phi_k) * sin(psi_k)) + (sin(psi_b) * sin(theta_b) - cos(psi_b) * cos(theta_b) * sin(phi_b)) * (sin(psi_k) * sin(theta_k) - cos(psi_k) * cos(theta_k) * sin(phi_k)) + cos(phi_b) * cos(phi_k) * cos(theta_b) * cos(theta_k), 2.0) * ((cos(theta_b) * sin(psi_b) + cos(psi_b) * sin(phi_b) * sin(theta_b)) * (sin(psi_k) * sin(theta_k) - cos(psi_k) * cos(theta_k) * sin(phi_k)) + (cos(psi_b) * cos(theta_b) - sin(phi_b) * sin(psi_b) * sin(theta_b)) * (cos(psi_k) * sin(theta_k) + cos(theta_k) * sin(phi_k) * sin(psi_k)) - cos(phi_b) * cos(phi_k) * cos(theta_k) * sin(theta_b))) * pow((cos(psi_b) * sin(theta_b) + cos(theta_b) * sin(phi_b) * sin(psi_b)) * (cos(psi_k) * sin(theta_k) + cos(theta_k) * sin(phi_k) * sin(psi_k)) + (sin(psi_b) * sin(theta_b) - cos(psi_b) * cos(theta_b) * sin(phi_b)) * (sin(psi_k) * sin(theta_k) - cos(psi_k) * cos(theta_k) * sin(phi_k)) + cos(phi_b) * cos(phi_k) * cos(theta_b) * cos(theta_k), 2.0)) / (pow((cos(psi_b) * sin(theta_b) + cos(theta_b) * sin(phi_b) * sin(psi_b)) * (cos(psi_k) * sin(theta_k) + cos(theta_k) * sin(phi_k) * sin(psi_k)) + (sin(psi_b) * sin(theta_b) - cos(psi_b) * cos(theta_b) * sin(phi_b)) * (sin(psi_k) * sin(theta_k) - cos(psi_k) * cos(theta_k) * sin(phi_k)) + cos(phi_b) * cos(phi_k) * cos(theta_b) * cos(theta_k), 2.0) + pow((cos(theta_b) * sin(psi_b) + cos(psi_b) * sin(phi_b) * sin(theta_b)) * (sin(psi_k) * sin(theta_k) - cos(psi_k) * cos(theta_k) * sin(phi_k)) + (cos(psi_b) * cos(theta_b) - sin(phi_b) * sin(psi_b) * sin(theta_b)) * (cos(psi_k) * sin(theta_k) + cos(theta_k) * sin(phi_k) * sin(psi_k)) - cos(phi_b) * cos(phi_k) * cos(theta_k) * sin(theta_b), 2.0));
    Ct(5, 3) = (sin(psi_b) * sin(psi_k) * sin(theta_k) + cos(psi_b) * cos(psi_k) * sin(theta_k) + cos(psi_b) * cos(theta_k) * sin(phi_k) * sin(psi_k) - cos(psi_k) * cos(theta_k) * sin(phi_k) * sin(psi_b)) / (pow(sin(phi_b) * sin(phi_k) + cos(phi_b) * cos(phi_k) * cos(psi_b) * cos(psi_k) + cos(phi_b) * cos(phi_k) * sin(psi_b) * sin(psi_k), 2.0) + pow(-cos(phi_b) * cos(psi_b) * (cos(theta_k) * sin(psi_k) + cos(psi_k) * sin(phi_k) * sin(theta_k)) + cos(phi_b) * sin(psi_b) * (cos(psi_k) * cos(theta_k) - sin(phi_k) * sin(psi_k) * sin(theta_k)) + cos(phi_k) * sin(phi_b) * sin(theta_k), 2.0));
    Ct(5, 5) = (((cos(phi_b) * cos(psi_b) * (cos(psi_k) * cos(theta_k) - sin(phi_k) * sin(psi_k) * sin(theta_k)) + cos(phi_b) * sin(psi_b) * (cos(theta_k) * sin(psi_k) + cos(psi_k) * sin(phi_k) * sin(theta_k))) / (sin(phi_b) * sin(phi_k) + cos(phi_b) * cos(phi_k) * cos(psi_b) * cos(psi_k) + cos(phi_b) * cos(phi_k) * sin(psi_b) * sin(psi_k)) + sin(psi_b - psi_k) * cos(phi_b) * cos(phi_k) * 1.0 / pow(sin(phi_b) * sin(phi_k) + cos(phi_b) * cos(phi_k) * cos(psi_b) * cos(psi_k) + cos(phi_b) * cos(phi_k) * sin(psi_b) * sin(psi_k), 2.0) * (-cos(phi_b) * cos(psi_b) * (cos(theta_k) * sin(psi_k) + cos(psi_k) * sin(phi_k) * sin(theta_k)) + cos(phi_b) * sin(psi_b) * (cos(psi_k) * cos(theta_k) - sin(phi_k) * sin(psi_k) * sin(theta_k)) + cos(phi_k) * sin(phi_b) * sin(theta_k))) * pow(sin(phi_b) * sin(phi_k) + cos(phi_b) * cos(phi_k) * cos(psi_b) * cos(psi_k) + cos(phi_b) * cos(phi_k) * sin(psi_b) * sin(psi_k), 2.0)) / (pow(sin(phi_b) * sin(phi_k) + cos(phi_b) * cos(phi_k) * cos(psi_b) * cos(psi_k) + cos(phi_b) * cos(phi_k) * sin(psi_b) * sin(psi_k), 2.0) + pow(-cos(phi_b) * cos(psi_b) * (cos(theta_k) * sin(psi_k) + cos(psi_k) * sin(phi_k) * sin(theta_k)) + cos(phi_b) * sin(psi_b) * (cos(psi_k) * cos(theta_k) - sin(phi_k) * sin(psi_k) * sin(theta_k)) + cos(phi_k) * sin(phi_b) * sin(theta_k), 2.0));
    Ct(5, 18) = -((sin(theta_k) - 1.0 / pow(sin(phi_b) * sin(phi_k) + cos(phi_b) * cos(phi_k) * cos(psi_b) * cos(psi_k) + cos(phi_b) * cos(phi_k) * sin(psi_b) * sin(psi_k), 2.0) * (-cos(phi_k) * sin(phi_b) + cos(phi_b) * cos(psi_b) * cos(psi_k) * sin(phi_k) + cos(phi_b) * sin(phi_k) * sin(psi_b) * sin(psi_k)) * (-cos(phi_b) * cos(psi_b) * (cos(theta_k) * sin(psi_k) + cos(psi_k) * sin(phi_k) * sin(theta_k)) + cos(phi_b) * sin(psi_b) * (cos(psi_k) * cos(theta_k) - sin(phi_k) * sin(psi_k) * sin(theta_k)) + cos(phi_k) * sin(phi_b) * sin(theta_k))) * pow(sin(phi_b) * sin(phi_k) + cos(phi_b) * cos(phi_k) * cos(psi_b) * cos(psi_k) + cos(phi_b) * cos(phi_k) * sin(psi_b) * sin(psi_k), 2.0)) / (pow(sin(phi_b) * sin(phi_k) + cos(phi_b) * cos(phi_k) * cos(psi_b) * cos(psi_k) + cos(phi_b) * cos(phi_k) * sin(psi_b) * sin(psi_k), 2.0) + pow(-cos(phi_b) * cos(psi_b) * (cos(theta_k) * sin(psi_k) + cos(psi_k) * sin(phi_k) * sin(theta_k)) + cos(phi_b) * sin(psi_b) * (cos(psi_k) * cos(theta_k) - sin(phi_k) * sin(psi_k) * sin(theta_k)) + cos(phi_k) * sin(phi_b) * sin(theta_k), 2.0));
    Ct(5, 19) = ((sin(phi_b) * sin(phi_k) + cos(phi_b) * cos(phi_k) * cos(psi_b) * cos(psi_k) + cos(phi_b) * cos(phi_k) * sin(psi_b) * sin(psi_k)) * (cos(phi_b) * cos(psi_b) * (sin(psi_k) * sin(theta_k) - cos(psi_k) * cos(theta_k) * sin(phi_k)) - cos(phi_b) * sin(psi_b) * (cos(psi_k) * sin(theta_k) + cos(theta_k) * sin(phi_k) * sin(psi_k)) + cos(phi_k) * cos(theta_k) * sin(phi_b))) / (pow(sin(phi_b) * sin(phi_k) + cos(phi_b) * cos(phi_k) * cos(psi_b) * cos(psi_k) + cos(phi_b) * cos(phi_k) * sin(psi_b) * sin(psi_k), 2.0) + pow(-cos(phi_b) * cos(psi_b) * (cos(theta_k) * sin(psi_k) + cos(psi_k) * sin(phi_k) * sin(theta_k)) + cos(phi_b) * sin(psi_b) * (cos(psi_k) * cos(theta_k) - sin(phi_k) * sin(psi_k) * sin(theta_k)) + cos(phi_k) * sin(phi_b) * sin(theta_k), 2.0));
    Ct(5, 20) = -(((cos(phi_b) * cos(psi_b) * (cos(psi_k) * cos(theta_k) - sin(phi_k) * sin(psi_k) * sin(theta_k)) + cos(phi_b) * sin(psi_b) * (cos(theta_k) * sin(psi_k) + cos(psi_k) * sin(phi_k) * sin(theta_k))) / (sin(phi_b) * sin(phi_k) + cos(phi_b) * cos(phi_k) * cos(psi_b) * cos(psi_k) + cos(phi_b) * cos(phi_k) * sin(psi_b) * sin(psi_k)) + sin(psi_b - psi_k) * cos(phi_b) * cos(phi_k) * 1.0 / pow(sin(phi_b) * sin(phi_k) + cos(phi_b) * cos(phi_k) * cos(psi_b) * cos(psi_k) + cos(phi_b) * cos(phi_k) * sin(psi_b) * sin(psi_k), 2.0) * (-cos(phi_b) * cos(psi_b) * (cos(theta_k) * sin(psi_k) + cos(psi_k) * sin(phi_k) * sin(theta_k)) + cos(phi_b) * sin(psi_b) * (cos(psi_k) * cos(theta_k) - sin(phi_k) * sin(psi_k) * sin(theta_k)) + cos(phi_k) * sin(phi_b) * sin(theta_k))) * pow(sin(phi_b) * sin(phi_k) + cos(phi_b) * cos(phi_k) * cos(psi_b) * cos(psi_k) + cos(phi_b) * cos(phi_k) * sin(psi_b) * sin(psi_k), 2.0)) / (pow(sin(phi_b) * sin(phi_k) + cos(phi_b) * cos(phi_k) * cos(psi_b) * cos(psi_k) + cos(phi_b) * cos(phi_k) * sin(psi_b) * sin(psi_k), 2.0) + pow(-cos(phi_b) * cos(psi_b) * (cos(theta_k) * sin(psi_k) + cos(psi_k) * sin(phi_k) * sin(theta_k)) + cos(phi_b) * sin(psi_b) * (cos(psi_k) * cos(theta_k) - sin(phi_k) * sin(psi_k) * sin(theta_k)) + cos(phi_k) * sin(phi_b) * sin(theta_k), 2.0));

    return Ct;
  }


Mat6x6 jacobiG2v(const Vec21& x, const Vec6& v) {

  // TODO
  // return the derivative wrt noise dz/dv

  Mat6x6 I6;

  I6.setIdentity();

  return I6;
}

}  // namespace ekf_imu_vision