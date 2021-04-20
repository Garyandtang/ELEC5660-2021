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
VectorXd x_est = VectorXd::Zero(15); //estimated state
MatrixXd Sig_post = 0.5*MatrixXd::Identity(15,15); // covariance matrix
Vector3d gravity(0, 0, 9.81);

// vectorXd x_bar = vectorXd::Zero(15); // prior state

double clamp_euler_angle(double theta)
{
    double result = atan2 (sin(theta),cos(theta)); 
    return result;
}

void imu_callback(const sensor_msgs::Imu::ConstPtr &msg)
{
    double dt = msg->header.stamp.toSec() - t_last;
    if(!updated || dt>1){
        t_last = msg->header.stamp.toSec();
        //update=true;
        return;
    }
    // cout << dt << endl;
    //accerlation
    Vector3d accel(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z);
    //angular velocity
    Vector3d ang_vel(msg->angular_velocity.x,msg->angular_velocity.y,msg->angular_velocity.z);
    Matrix3d G, R, G_inv_dot, R_dot;
    double phi =x_est(3);
    double theta=x_est(4);
    double psi=x_est(5);
    G << cos(theta), 0, -cos(phi)*sin(theta),
         0, 1, sin(phi),
         sin(theta), 0, cos(theta)*cos(phi);

    R << cos(psi)*cos(theta)- sin(phi)*sin(psi)*sin(theta), -cos(phi)*sin(psi), cos(psi)*sin(theta)+cos(theta)*sin(phi)*sin(psi),
        cos(theta)*sin(psi)+cos(psi)*sin(phi)*sin(theta), cos(phi)*cos(psi), sin(psi)*sin(theta)-cos(psi)*sin(phi)*cos(theta),
            -cos(phi)*sin(theta), sin(phi), cos(phi)*cos(theta);

    Matrix3d G_inv = G.inverse();

    G_inv_dot << 0, ang_vel(2)*cos(theta) - ang_vel(0)*sin(theta), 0,
            ang_vel(0)*sin(theta) - ang_vel(2)*cos(theta) - (ang_vel(2)*cos(theta)*sin(phi)*sin(phi))/(cos(phi)*cos(phi)) + (ang_vel(0)*sin(phi)*sin(phi)*sin(theta))/(cos(phi)*cos(phi)), (ang_vel(0)*cos(theta)*sin(phi))/cos(phi) + (ang_vel(2)*sin(phi)*sin(theta))/cos(phi), 0,
            (ang_vel(2)*cos(theta)*sin(phi))/(cos(phi)*cos(phi)) - (ang_vel(0)*sin(phi)*sin(theta))/(cos(phi)*cos(phi)), - (ang_vel(0)*cos(theta))/cos(phi) - (ang_vel(2)*sin(theta))/cos(phi), 0;
    R_dot << accel(1)*sin(phi)*sin(psi) + accel(2)*cos(phi)*cos(theta)*sin(psi) - accel(0)*cos(phi)*sin(theta)*sin(psi), accel(2)*(cos(theta)*cos(psi) - sin(phi)*sin(theta)*sin(psi)) - accel(0)*(cos(psi)*sin(theta) + cos(theta)*sin(phi)*sin(psi)), - accel(0)*(cos(theta)*sin(psi) + cos(psi)*sin(phi)*sin(theta)) - accel(2)*(sin(theta)*sin(psi) - cos(theta)*cos(psi)*sin(phi)) - accel(1)*cos(phi)*cos(psi),
           accel(0)*cos(phi)*cos(psi)*sin(theta) - accel(2)*cos(phi)*cos(theta)*cos(psi) - accel(1)*cos(psi)*sin(phi), accel(2)*(cos(theta)*sin(psi) + cos(psi)*sin(phi)*sin(theta)) - accel(0)*(sin(theta)*sin(psi) - cos(theta)*cos(psi)*sin(phi)),   accel(0)*(cos(theta)*cos(psi) - sin(phi)*sin(theta)*sin(psi)) + accel(2)*(cos(psi)*sin(theta) + cos(theta)*sin(phi)*sin(psi)) - accel(1)*cos(phi)*sin(psi),
           accel(1)*cos(phi) - accel(2)*cos(theta)*sin(phi) + accel(0)*sin(phi)*sin(theta), - accel(0)*cos(phi)*cos(theta) - accel(2)*cos(phi)*sin(theta), 0;

    MatrixXd At = MatrixXd::Zero(15,15);
    At.block<3,3>(0,6) = MatrixXd::Identity(3,3);
    At.block<3,3>(3,3) << G_inv_dot;
    At.block<3,3>(6,3) << R_dot;
    At.block<3,3>(3,9) << -G_inv;
    At.block<3,3>(6,12) << -R;

    MatrixXd Ut = MatrixXd::Zero(15,12);
    Ut.block<3,3>(3,0) << -G_inv;
    Ut.block<3,3>(6,3) << -R;
    Ut.block<6,6>(9,6) << MatrixXd::Identity(6,6);

    MatrixXd Ft = MatrixXd::Identity(15,15) + dt*At;
    MatrixXd Vt = dt*Ut;
    Sig_post = Ft * Sig_post * Ft.transpose() + Vt * Q * Vt.transpose();
    VectorXd f = VectorXd::Zero(15);
    f.block<3,1>(0,0) = x_est.block<3,1>(6,0);
    f.block<3,1>(3,0) = G_inv * (ang_vel-Vector3d(x_est(9),x_est(10),x_est(11)));
    f.block<3,1>(6,0) = gravity + R*(accel-Vector3d(x_est(12),x_est(13),x_est(14)));
    x_est += dt*f;
    x_est(3) = clamp_euler_angle(x_est(3));
    x_est(4) = clamp_euler_angle(x_est(4));
    x_est(5) = clamp_euler_angle(x_est(5));
    t_last = msg->header.stamp.toSec();


    // // use to record data for matlab
    // VectorXd u = VectorXd::Zero(6);
    // u.segment(0,3) = angular_vel;
    // u.segment(3,3) = linear_acc;
    // cout << u.transpose() << endl;
}

//Rotation from the camera frame to the IMU frame
Eigen::Matrix3d Rcam;
void odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    // odom topic
    // frame id: world
    // position: x,y,z
    // orientation: quaterniod 

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
    cout << z_t.transpose() << endl;
    C_t.block(0,0,6,6).setIdentity(6,6);
    inno = z_t - C_t*x_est;
    inno(3) = clamp_euler_angle(inno(3));
    inno(4) = clamp_euler_angle(inno(4));
    inno(5) = clamp_euler_angle(inno(5));

    K_xx = Sig_post*C_t.transpose()*(C_t*Sig_post*C_t.transpose()+Rt).inverse(); // normal way to get K
    K_t = (C_t*Sig_post*C_t.transpose()+Rt).lu().solve(C_t*Sig_post).transpose(); // Ax=b to get K
    x_est += K_t*inno;
    Sig_post -= K_t*C_t*Sig_post;
    // cout << x_est.segment(0,3).transpose() << endl;

   
  
    // step3: publish a odom topic
    Matrix3d R_EKF;
    R_EKF << cos(x_est(5))*cos(x_est(4))- sin(x_est(3))*sin(x_est(5))*sin(x_est(4)), -cos(x_est(3))*sin(x_est(5)), cos(x_est(5))*sin(x_est(4))+cos(x_est(4))*sin(x_est(3))*sin(x_est(5)),
          cos(x_est(4))*sin(x_est(5))+cos(x_est(5))*(sin(x_est(3)))*sin(x_est(4)), cos(x_est(3))*cos(x_est(5)), sin(x_est(5))*sin(x_est(4))-cos(x_est(5))*sin(x_est(3))*cos(x_est(4)),
          -cos(x_est(3))*sin(x_est(4)), sin(x_est(3)), cos(x_est(3))*cos(x_est(4));
    Quaterniond Q_EKF(R_EKF);
    nav_msgs::Odometry odom_EKF;
    odom_EKF.header.stamp = msg->header.stamp;
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
    odom_pub.publish(odom_EKF);
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
    Q.topLeftCorner(6, 6) = 0.1 * Q.topLeftCorner(6, 6);
    Q.bottomRightCorner(6, 6) = 0.04 * Q.bottomRightCorner(6, 6);
    Rt = 0.01*MatrixXd::Identity(6,6);
    // Rt.topLeftCorner(3, 3) = 0.005 * Rt.topLeftCorner(3, 3);
    // Rt.bottomRightCorner(3, 3) = 0.005 * Rt.bottomRightCorner(3, 3);
    // Rt.bottomRightCorner(1, 1) = 0.005 * Rt.bottomRightCorner(1, 1);
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

// -0.00210754    0.026198  -0.0117304
// -0.00417698   0.0522297   -0.017793
// -0.00575996   0.0837092   0.0497537
// -0.00788572    0.114321    0.094218
// -0.0119424   0.161457   0.299477
// -0.0153449   0.204642   0.425302
// -0.0230392   0.274252   0.731807
// -0.0275868   0.331348   0.871413
// -0.0456481   0.445843    1.31945
// -0.0621024   0.550947    1.59729
// -0.0984047   0.716456    2.07492
// -0.130517  0.861833   2.35517
// -0.210789    1.1229   2.98381
// -0.265645    1.3113   3.23813
// -0.406457   1.67693   3.96021
// -0.488762   1.91844   4.22062
// -0.688761   2.37999   4.98543
// -0.785539   2.65343   5.20364
// -1.05286   3.2253  6.01398
// -1.18379  3.57543  6.26958


// -0.00210754    0.026198  -0.0117304
// -0.00413267   0.0523999  -0.0152521
// -0.00548132   0.0839441   0.0543726
// -0.00727783    0.117298    0.131215
// -0.00965389    0.162265    0.318147
// -0.0108827   0.200646   0.403702
// -0.0131655   0.273697   0.761055
// -0.0131682   0.330501   0.903532
// -0.0167261   0.440736     1.3412
// -0.0201789   0.545009    1.62286
// -0.0335746   0.720465    2.15365
// -0.0441298   0.851052    2.36813
// -0.0855367    1.12969    3.06103
// -0.12378  1.35046  3.40886
// -0.201392   1.70256    4.0665
// -0.255488   1.95092   4.34007
// -0.385237   2.42973   5.13633
// ^C-0.457128   2.72376   5.40568

// -0.210754    2.6198  -1.17304
// -0.216444   2.63183  -1.17842
// -0.22427  2.63654 -1.18005
// -0.233448    2.6385  -1.18042
// -0.248428   2.63745  -1.17905
// -0.260468   2.63392  -1.17744
// -0.27792  2.62818 -1.17479
// -0.289727   2.62198  -1.17332
// -0.306684   2.61144  -1.17144
// -0.316353   2.60372  -1.17037
// -0.331428    2.5936  -1.16895
// -0.342243   2.58816  -1.16724
// -0.361374   2.57733  -1.16467
// -0.392542   2.59052  -1.15261
// -0.427978   2.59433  -1.14181
// -0.445825   2.58718  -1.13754
// -0.455358   2.56257  -1.13846
// -0.458233   2.53495   -1.1412
// -0.46488  2.51731 -1.14024
// -0.462221   2.50443  -1.14091
// -0.45182  2.48744 -1.14262
// -0.440265   2.47773  -1.14355
// -0.406915   2.45998  -1.14696
// ^C-0.363031   2.42787  -1.15394
