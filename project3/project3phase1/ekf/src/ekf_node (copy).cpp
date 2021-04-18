#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Range.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Eigen>
#include<cmath>

using namespace std;
using namespace Eigen;
ros::Publisher odom_pub;
MatrixXd Q = MatrixXd::Identity(12, 12);
MatrixXd Rt = MatrixXd::Identity(6,6);

// Global variables
double t_last = 0; // use to store last time step
double t_last_odom = 0;
bool updated = false; // use to indicate received odom or not
VectorXd X_post = VectorXd::Zero(15); //estimated state
MatrixXd Sig_post = 0.001*MatrixXd::Identity(15,15); // covariance matrix
Vector3d gravity(0, 0, 9.81);
// Sigma.setIdentity(15,15);
// Sigma = 0.001*Sigma;
// vectorXd x_bar = vectorXd::Zero(15); // prior state



double clamp_euler_angle(double theta)
{
    // double result = atan2 (sin(theta),cos(theta)); 
    // return result;
    while (theta > M_PI){
        theta -= 2*M_PI;
    }
    while (theta < -M_PI){
        theta += 2*M_PI;
    }
    return theta;
}

inline void clampEuler(double &x) {
  while (x < -M_PI)
    x += 2 * M_PI;
  while (x > M_PI)
    x -= 2 * M_PI;
}
void imu_callback(const sensor_msgs::Imu::ConstPtr &msg)
{
    // //your code for propagation
    // // IMU topic
    // // frame id: FLU
    // // orientation: quaterniod
    // // angular_velocity: x, y, z
    // // linear_acceleration: x, y, z

    // // cout << "received IMU" << endl;
    // double dt = msg->header.stamp.toSec() - t_last;
    // t_last = msg->header.stamp.toSec();
    // if (updated == false || dt > 1)
    // {
    //     cout << "Haven't received odom result." <<  endl;
    //     return;
    // }
    // // cout << "dt imu: " << dt << endl;
    // Vector3d angular_vel;
    // angular_vel << msg->angular_velocity.x,msg->angular_velocity.y, msg->angular_velocity.z;
    // Vector3d linear_acc;
    // linear_acc << msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z;
    // Vector3d omega_m, accel_m;
    // omega_m = angular_vel;
    // accel_m = linear_acc;
    // VectorXd f = VectorXd::Zero(15);
    // Matrix3d G_t, G_inv, R_wi;

    // f.segment(0, 3) = X_post.segment(6, 3);

    // G_t << cos(X_post(4)), 0, -cos(X_post(3) * sin(X_post(4))),
    //         0, 1, sin(X_post(3)),
    //         sin(X_post(4)), 0, cos(X_post(3)) * cos(X_post(4));
    // G_inv = G_t.inverse();
    // f.segment(3, 3) = G_inv * (omega_m - X_post.segment(9, 3));

    // R_wi << cos(X_post(5)) * cos(X_post(4)) - sin(X_post(3)) * sin(X_post(5)) * sin(X_post(4)), -cos(X_post(3)) *
    //                                                                                             sin(X_post(5)),
    //         cos(X_post(5)) * sin(X_post(4)) + cos(X_post(4)) * sin(X_post(3)) * sin(X_post(5)),
    //         cos(X_post(4)) * sin(X_post(5)) + cos(X_post(5)) * sin(X_post(3)) * sin(X_post(4)), cos(X_post(3)) *
    //                                                                                             cos(X_post(5)),
    //         sin(X_post(5)) * sin(X_post(4)) - cos(X_post(5)) * cos(X_post(4)) * sin(X_post(3)),
    //         -cos(X_post(3)) * sin(X_post(4)), sin(X_post(3)), cos(X_post(3)) * cos(X_post(4));
    // f.segment(6, 3) = R_wi * (accel_m - X_post.segment(12, 3)) + gravity;

    // // predict cov
    // Matrix<double, 15, 15> A_t, F_t;
    // Matrix<double, 15, 12> U_t, V_t;
    // A_t.setZero();
    // U_t.setZero();

    // Matrix3d G_inv_dot, R_dot;

    // G_inv_dot << 0, omega_m(2) * cos(X_post(4)) - omega_m(0) * sin(X_post(4)), 0,
    //         omega_m(0) * sin(X_post(4)) - omega_m(2) * cos(X_post(4)) -
    //         (omega_m(2) * cos(X_post(4)) * sin(X_post(3)) * sin(X_post(3))) / (cos(X_post(3)) * cos(X_post(3))) +
    //         (omega_m(0) * sin(X_post(3)) * sin(X_post(3)) * sin(X_post(4))) / (cos(X_post(3)) * cos(X_post(3))),
    //         (omega_m(0) * cos(X_post(4)) * sin(X_post(3))) / cos(X_post(3)) +
    //         (omega_m(2) * sin(X_post(3)) * sin(X_post(4))) / cos(X_post(3)), 0,
    //         (omega_m(2) * cos(X_post(4)) * sin(X_post(3))) / (cos(X_post(3)) * cos(X_post(3))) -
    //         (omega_m(0) * sin(X_post(3)) * sin(X_post(4))) / (cos(X_post(3)) * cos(X_post(3))),
    //         -(omega_m(0) * cos(X_post(4))) / cos(X_post(3)) - (omega_m(2) * sin(X_post(4))) / cos(X_post(3)), 0;
    // R_dot << accel_m(1) * sin(X_post(3)) * sin(X_post(5)) +
    //          accel_m(2) * cos(X_post(3)) * cos(X_post(4)) * sin(X_post(5)) -
    //          accel_m(0) * cos(X_post(3)) * sin(X_post(4)) * sin(X_post(5)),
    //         accel_m(2) * (cos(X_post(4)) * cos(X_post(5)) - sin(X_post(3)) * sin(X_post(4)) * sin(X_post(5))) -
    //         accel_m(0) * (cos(X_post(5)) * sin(X_post(4)) + cos(X_post(4)) * sin(X_post(3)) * sin(X_post(5))),
    //         -accel_m(0) * (cos(X_post(4)) * sin(X_post(5)) + cos(X_post(5)) * sin(X_post(3)) * sin(X_post(4))) -
    //         accel_m(2) * (sin(X_post(4)) * sin(X_post(5)) - cos(X_post(4)) * cos(X_post(5)) * sin(X_post(3))) -
    //         accel_m(1) * cos(X_post(3)) * cos(X_post(5)),
    //         accel_m(0) * cos(X_post(3)) * cos(X_post(5)) * sin(X_post(4)) -
    //         accel_m(2) * cos(X_post(3)) * cos(X_post(4)) * cos(X_post(5)) -
    //         accel_m(1) * cos(X_post(5)) * sin(X_post(3)),
    //         accel_m(2) * (cos(X_post(4)) * sin(X_post(5)) + cos(X_post(5)) * sin(X_post(3)) * sin(X_post(4))) -
    //         accel_m(0) * (sin(X_post(4)) * sin(X_post(5)) - cos(X_post(4)) * cos(X_post(5)) * sin(X_post(3))),
    //         accel_m(0) * (cos(X_post(4)) * cos(X_post(5)) - sin(X_post(3)) * sin(X_post(4)) * sin(X_post(5))) +
    //         accel_m(2) * (cos(X_post(5)) * sin(X_post(4)) + cos(X_post(4)) * sin(X_post(3)) * sin(X_post(5))) -
    //         accel_m(1) * cos(X_post(3)) * sin(X_post(5)),
    //         accel_m(1) * cos(X_post(3)) - accel_m(2) * cos(X_post(4)) * sin(X_post(3)) +
    //         accel_m(0) * sin(X_post(3)) * sin(X_post(4)), -accel_m(0) * cos(X_post(3)) * cos(X_post(4)) -
    //                                                       accel_m(2) * cos(X_post(3)) * sin(X_post(4)), 0;

    // A_t.block<3, 3>(0, 6) = MatrixXd::Identity(3, 3);
    // A_t.block<3, 3>(3, 3) = G_inv_dot;
    // A_t.block<3, 3>(6, 3) = R_dot;
    // A_t.block<3, 3>(3, 9) = -G_inv;
    // A_t.block<3, 3>(6, 12) = -R_wi;
    // F_t = MatrixXd::Identity(15, 15) + dt * A_t;

    // U_t.block<3, 3>(3, 0) = -G_inv;
    // U_t.block<3, 3>(6, 3) = -R_wi;
    // U_t.block<6, 6>(9, 6) = MatrixXd::Identity(6, 6);
    // V_t = dt * U_t;

    // X_post += dt * f;
    // clampEuler(X_post(3));
    // clampEuler(X_post(4));
    // clampEuler(X_post(5));
    // Sig_post = F_t * Sig_post * F_t.transpose() + V_t * Q * V_t.transpose();

    // // use to record data for matlab
    // VectorXd u = VectorXd::Zero(6);
    // u.segment(0,3) = angular_vel;
    // u.segment(3,3) = linear_acc;
    // cout << u.transpose() << endl;
}
void pub_ekf_odom(const VectorXd &x, const std_msgs::Header &hd) {
  Matrix3d R_wi;
  R_wi << cos(x(5)) * cos(x(4)) - sin(x(3)) * sin(x(5)) * sin(x(4)), -cos(x(3)) * sin(x(5)), cos(x(5)) * sin(x(4)) +
                                                                                             cos(x(4)) * sin(x(3)) *
                                                                                             sin(x(5)),
          cos(x(4)) * sin(x(5)) + cos(x(5)) * (sin(x(3))) * sin(x(4)), cos(x(3)) * cos(x(5)), sin(x(5)) * sin(x(4)) -
                                                                                              cos(x(5)) * sin(x(3)) *
                                                                                              cos(x(4)),
          -cos(x(3)) * sin(x(4)), sin(x(3)), cos(x(3)) * cos(x(4));

  Quaterniond q(R_wi);
  nav_msgs::Odometry ekf_odom;
  ekf_odom.header.frame_id = "world";
  ekf_odom.header.stamp = hd.stamp;
  ekf_odom.pose.pose.position.x = x(0);
  ekf_odom.pose.pose.position.y = x(1);
  ekf_odom.pose.pose.position.z = x(2);
  ekf_odom.twist.twist.linear.x = x(6);
  ekf_odom.twist.twist.linear.y = x(7);
  ekf_odom.twist.twist.linear.z = x(8);
  ekf_odom.pose.pose.orientation.x = q.x();
  ekf_odom.pose.pose.orientation.y = q.y();
  ekf_odom.pose.pose.orientation.z = q.z();
  ekf_odom.pose.pose.orientation.w = q.w();
  odom_pub.publish(ekf_odom);
}
//Rotation from the camera frame to the IMU frame
Eigen::Matrix3d Rcam;
void odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    // odom topic
    // frame id: world
    // position: x,y,z
    // orientation: quaterniod 

    //your code for update
    // camera position in the IMU frame = (0.05, 0.05, 0)
    // camera orientaion in the IMU frame = Quaternion(0, 1, 0, 0); w x y z, respectively
    //					   RotationMatrix << 1, 0, 0,
    //							             0, -1, 0,
    //                                       0, 0, -1;
    // cout << "received odom" << endl;

    // // step 0: test dt 
    // double dt = msg->header.stamp.toSec() - t_last_odom;
    // t_last_odom = msg->header.stamp.toSec();
    // cout << dt << endl;
    // test original camera measurement data
    // VectorXd z_cam = VectorXd::Zero(6);
    // double phi_cam = asin(c_R_w(2, 1));
    // double theta_cam = atan2(-c_R_w(2, 0) / cos(phi_cam), c_R_w(2, 2) / cos(phi_cam));
    // double psi_cam = atan2(-c_R_w(0, 1) / cos(phi_cam), c_R_w(1, 1) / cos(phi_cam));
    // z_cam << c_t_w(0), c_t_w(1), c_t_w(2), phi_cam, theta_cam, psi_cam;

    // step1: transfer imu to world frame
    Vector3d c_t_w, i_t_c, w_t_i;
    Matrix3d c_R_w, i_R_c, w_R_i;
    Quaterniond q(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
    c_t_w << msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z; 
    c_R_w = q.toRotationMatrix();
    i_t_c << 0.05, 0.05, 0;
    i_R_c << 1,  0,  0,
             0, -1,  0,
             0,  0, -1;
    w_R_i = c_R_w.transpose()*i_R_c.transpose();
    w_t_i = -c_R_w.transpose()*i_R_c.transpose()*i_t_c - c_R_w.transpose()*c_t_w;


    //rotation matrix to euler angle
    double phi = asin(w_R_i(2, 1));
    double theta = atan2(-w_R_i(2, 0) / cos(phi), w_R_i(2, 2) / cos(phi));
    double psi = atan2(-w_R_i(0, 1) / cos(phi), w_R_i(1, 1) / cos(phi));


    // step2: measurement update of EKF
    VectorXd z_t = VectorXd::Zero(6), inno = VectorXd::Zero(6); // measurement
    MatrixXd C_t = MatrixXd::Zero(6,15), K_t, K_xx;
    z_t << w_t_i(0), w_t_i(1), w_t_i(2), phi, theta, psi;
    C_t.block(0,0,6,6).setIdentity(6,6);
    inno = z_t - C_t*X_post;
    clampEuler(inno(3));
    clampEuler(inno(4));
    clampEuler(inno(5));
    K_xx = Sig_post*C_t.transpose()*(C_t*Sig_post*C_t.transpose()+Rt).inverse(); // normal way to get K
    K_t = (C_t*Sig_post*C_t.transpose()+Rt).lu().solve(C_t*Sig_post).transpose(); // Ax=b to get K
    X_post += K_t*inno;
    Sig_post -= K_t*C_t*Sig_post;
    cout << X_post.segment(0,3).transpose() << endl;

   
    pub_ekf_odom(X_post, msg->header);
    // // step3: publish a odom topic
    // Quaterniond Q_EKF(w_R_i);
    // nav_msgs::Odometry odom_EKF;
    // odom_EKF.header.stamp = msg->header.stamp;
    // odom_EKF.header.frame_id = "world";
    // odom_EKF.pose.pose.position.x = w_t_i(0);
    // odom_EKF.pose.pose.position.y = w_t_i(1);
    // odom_EKF.pose.pose.position.z = w_t_i(2);
    // odom_EKF.pose.pose.orientation.w = Q_EKF.w();
    // odom_EKF.pose.pose.orientation.x = Q_EKF.x();
    // odom_EKF.pose.pose.orientation.y = Q_EKF.y();
    // odom_EKF.pose.pose.orientation.z = Q_EKF.z();
    // odom_pub.publish(odom_EKF);
    updated = true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ekf");
    ros::NodeHandle n("~");
    ros::Subscriber s1 = n.subscribe("imu", 1000, imu_callback);
    ros::Subscriber s2 = n.subscribe("tag_odom", 1000, odom_callback);
    odom_pub = n.advertise<nav_msgs::Odometry>("ekf_odom", 100);
    Rcam = Quaterniond(0, 1, 0, 0).toRotationMatrix();
    cout << "R_cam" << endl << Rcam << endl;
    // Q imu covariance matrix; Rt visual odomtry covariance matrix
    // You should also tune these parameters
    Q.topLeftCorner(6, 6) = 0.01 * Q.topLeftCorner(6, 6);
    Q.bottomRightCorner(6, 6) = 0.01 * Q.bottomRightCorner(6, 6);
    Rt.topLeftCorner(3, 3) = 0.1 * Rt.topLeftCorner(3, 3);
    Rt.bottomRightCorner(3, 3) = 0.1 * Rt.bottomRightCorner(3, 3);
    Rt.bottomRightCorner(1, 1) = 0.1 * Rt.bottomRightCorner(1, 1);
    cout << "Rt" << endl << Rt << endl;
    cout << "Q" << endl << Q << endl;
    ros::spin();
}

    // Quaterniond q_1(msg->pose.pose.orientation.w, msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
    //             msg->pose.pose.orientation.z);
    // Matrix3d R_cw = q_1.toRotationMatrix();
    // Vector3d t_cw = Vector3d(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    // Matrix3d Rcam = Quaterniond(0, 1, 0, 0).toRotationMatrix();
    // Matrix3d R_wi = R_cw.transpose() * Rcam.transpose();
    // Vector3d t_ic = Vector3d(0.05, 0.05, 0);
    // Vector3d t_wi = -R_cw.transpose() * (Rcam.transpose() * t_ic + t_cw);
    // cout << "my_R_result:\n" << w_R_i << endl;
    // cout << "BQ_R_result:\n" << R_wi << endl;
    // cout << "my_t_result:\n" << w_t_i << endl;
    // cout << "BQ_t_result:\n" << t_wi << endl;
    // cout << "======================" << endl;