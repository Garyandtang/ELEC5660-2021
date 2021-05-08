## ELEC5660 Project 3 Phase 2: Augmented EKF for Quadrotor

C++ implementation of Augmented Extended Kalman filter for quadrotor state estimation with visual odometry, tag detection and imu. 

## Installation

#### Platform

Ubuntu 16.04 with Ros Kinetic

#### Dependencies

* OpenCV: 3.3.1
* catkin library
* Eigen: 3.3
* Aruco: 1.2.4

#### Install

Follow the tutorials in Eigen and OpenCV official website to install them.

Follow the tutorials in `../../project2/project2pahse1/assignment.pdf` to install Aruco and build the project.

#### Data

Offline dataset can be download [here]()

It should be placed under `aug_ekf/bag`.

## Result

#### Figures

**Figure 1: Augmented EKF with simple bag**

* Red line: path
* Arrow: Augmented EKF odomentry result

![](/home/eeuser/Dropbox/01_hkust/01_course/ELEC5660/ELEC5660-2021/project3/project3phase2/img/AEKF_simple.png)

**Figure 2: Augmented EKF with original bag**

![](/home/eeuser/Dropbox/01_hkust/01_course/ELEC5660/ELEC5660-2021/project3/project3phase2/img/AEKF_odom.png)

**Figure 3: Comparison of VO and Augmented EKF with original bag**

![](/home/eeuser/Dropbox/01_hkust/01_course/ELEC5660/ELEC5660-2021/project3/project3phase2/img/AEKF_vo.png)