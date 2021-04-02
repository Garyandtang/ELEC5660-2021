# ELEC5660 Project 2 Phase 1: Pose Estimation with DLT

C++ implementation of direct linear transformation for 2D-3D pose estimation. Offline dataset on 2D points and 3D points  as well as camera intrinsic parameters are provided. The OpenCV functions are not allowed to used in this implementation. 

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

Follow the tutorials in assignment.pdf` to install Aruco and build the project.

#### Data

Offline dataset can be download [here]()

It should be placed under `tag_detector/bag`.



## Usage

Follow tutorials in `assigment.pdf` to build the project and launch the project

```
roslaunch tag_detector bag.launch
```

## Result

Show in `project2phase1_jiawei.pdf`