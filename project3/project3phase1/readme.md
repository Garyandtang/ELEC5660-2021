# ELEC5660 Project 3 Phase 1: EKF for Quadrotor

C++ implementation of Extended Kalman filter for quadrotor state estimation with camera measurement and imu. 

## Installation

#### Platform

Ubuntu 16.04 with Ros Kinetic

#### Dependencies

* OpenCV: 3.3.1
* catkin library
* Eigen: 3.3

#### Install

Follow the tutorials in Eigen, OpenCV and ceres official website to install them.

#### Data

Offline dataset can be download [here]()

It should be placed under `ekf/bag`.

## Usage

1. Clone `ekf`  to your `catkin_ws/src` 

2. Cone [`tag_detection`](https://github.com/Garyandtang/ELEC5660-2021/tree/main/project2/project2phase1/tag_detector) to your `catkin_ws/src` 

3. catkin_make in your catkin workspace

4. source development setup

5. launch the file

   ```shell
   roslaunch ekf A3.launch
   ```

## Result

Show in `project3phase1_jiawei.pdf`

**Figure 1: RVIZ result**: 

* Axes: tag detection result by SolvePnP ($T_{cw}$)
* Arrow: EKF result  of body frame under camera frame ($T_{wi}$)

![EKF_RVIZ](/home/eeuser/Dropbox/01_hkust/01_course/ELEC5660/ELEC5660-2021/project3/project3phase1/img/EKF_result_RVIZ.png)

**Figure 2: rqt result of position**: 

![EKF_position](/home/eeuser/Dropbox/01_hkust/01_course/ELEC5660/ELEC5660-2021/project3/project3phase1/img/EKF_position_result.png)

**Figure 3: rqt result of orientation**: 

![EKF_orientation](/home/eeuser/Dropbox/01_hkust/01_course/ELEC5660/ELEC5660-2021/project3/project3phase1/img/EKF_orientation_result.png)

**Figure 4: rqt result of linear velocity**: 

![EKF_twist](/home/eeuser/Dropbox/01_hkust/01_course/ELEC5660/ELEC5660-2021/project3/project3phase1/img/EKF_twist_result.png)