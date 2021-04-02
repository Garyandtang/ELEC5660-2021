# ELEC5660 Project 2 Phase 2: Visual Odometry Stereo Camera

C++ implementation of visual odometry with stereo camera. Offline dataset on images obtained by stereo camera as well as camera intrinsic parameters are provided. The project are mainly implemented with OpenCV function, including:

* `cv::goodFeaturesToTrack()` 
* `cv::calcOpticalFlowPyrLK()`
* `cv::solvePnPRansac()`
* `cv::findFundamentalMat()`

## Installation

#### Platform

Ubuntu 16.04 with Ros Kinetic

#### Dependencies

* OpenCV: 3.3.1
* catkin library
* Eigen: 3.3
* ceres: 1.14

#### Install

Follow the tutorials in Eigen, OpenCV and ceres official website to install them.

#### Data

Offline dataset can be download [here]()

It should be placed under `stereo_vo_estmator/bag`.



## Usage

Follow tutorials in `assigment.pdf` to build the project and launch the project

```
roslaunch stereo_vo_estmator stereo_vo_bag.launch
```

## Result

Show in `project2phase2_jiawei.pdf`