[toc]
# Source code for ELEC5660-2021

Source code of ELEC5660 Introduction to Aerial Robotics by [Prof. Shaojie Shen](https://facultyprofiles.ust.hk/profiles.php?profile=shaojie-shen-eeshaojie/) at HKUST.

C++ for Project 2-4, Matlab for Project 1.

This year we totally have 4 projects in this course, including 

- [x] Project 1 (Control and Planning)
  - [x] PID Quadrotor trajectory tracking control
  - [x] Minimum snap optimization-based trajectory generation
  - [x] Optimal path planning (A* + Dijkstra)
- [x] Project 2 (Visual estimator)
  - [x] 3D-2D Pose Estimation with Direct  Linear Transform (DLT)
  - [x] Visual Odometry with Stereo Camera
- [x] Project 3 (EKF sensor fusion)
  - [x] EKF for quadrotor state estimation
  - [x] Augmented EKF with imu, visual odometry, and tag detection
- [ ] Project 4 (System integration on-board the drone)

## Dataset
* Dataset of project 2 and project 3: https://hkustconnect-my.sharepoint.com/:f:/g/personal/jtangas_connect_ust_hk/EpemMUamNbFBtL2c18R_DfQBDykZWsBn9RR3IKZTftks6w?e=bQaWDb

## Project 1 (Control and Planning)

The project 1 is done in **MATLAB simulator**, where I implemented 1) Quadrotor trajectory tracking control; 2) Optimization-based trajectory generation;. 3) path planning + trajectory generation + control.

#### Phase 1 Result (PID)

![phase_1](project1/proj1phase1/code/img/diamond.jpg)

#### Phase 2 Result (PID+Trajectory Generation)

![phase_2](project1/proj1phase2/code/img/path4.jpg)

#### Phase 3 Result (PID+A*+Trajectory Generation)

![phase_1](project1/proj1phase3/code/img/path2.jpg)

## Project 2 (Visual estimator)

The project 1 is done in **ROS ** with offline dataset. The code is done with C++. Two sub tasks are:

1. 3D-2D Pose Estimation with Direct  Linear Transform (DLT)
2. Visual Odometry with Stereo Camera

#### Phase 1 Result (Pose Estimation)

![phase_1](project2/project2phase1/tag_detector/document/proj2phase1_result.png)

#### Phase 2 Result (Visual Odometry)

![phase_1](project2/project2phase2/img/project2phase2_result.png)



## Project 3 (Sensor Fusion)

The project 3 is done in **ROS ** with offline dataset. The code is done with C++. Two sub tasks are:

1. EKF for quadrotor state estimation.
2.   Augmented EKF for quadrotor state estimation with tag detection, visual odometry and IMU.

#### Phase 1 Result (EKF for Quadrotor State Estimation)

![phase_1](project3/project3phase1/img/EKF_result_RVIZ.png)



#### Phase 2 Result (Augmented EKF for Quadrotor State Estimation)

![phase_2](project3/project3phase2/img/AEKF_odom.png)

![phase_2](project3/project3phase2/img/AEKF_simple.png)

![phase_2](project3/project3phase2/img/AEKF_vo.png)

## Folder Structure

```
.
├── project1
│   ├── proj1phase1
│   │   ├── assignment.pdf
│   │   ├── code
│   │   ├── proj1phase1.md
│   │   ├── Project 1 Phase 1_Report_Jiawei_Tang.pdf
│   │   └── README.txt
│   ├── proj1phase2
│   │   ├── assignment.pdf
│   │   ├── code
│   │   ├── Project 1 Phase 2_Jiawei_Tang.pdf
│   │   └── README.txt
│   └── proj1phase3
│       ├── assignment.pdf
│       ├── code
│       ├── project1pahse3_Jiawei.md
│       ├── project1pahse3_Jiawei.pdf
│       └── README.txt
├── project2
│   ├── project2phase1
│   │   ├── aruco-1.2.4
│   │   ├── assignment.pdf
│   │   ├── project2pahse1_jiawei.pdf
│   │   ├── readme.md
│   │   └── tag_detector
│   └── project2phase2
│       ├── assignment.pdf
│       ├── camera_models
│       ├── img
│       ├── project2pahse2_Jiawei.md
│       ├── project2pahse2_Jiawei.pdf
│       ├── readme.md
│       └── stereo_vo_estimator
├── project3
│   ├── project3phase1
│   │   ├── assignment.pdf
│   │   ├── ekf
│   │   ├── img
│   │   ├── project3pahse1_Jiawei.md
│   │   ├── project3pahse1_Jiawei.pdf
│   │   ├── readme.md
│   │   └── script
│   └── project3phase2
│       ├── aug_ekf
│       ├── img
│       ├── project3pahse2_Jiawei.md
│       ├── project3pahse2_Jiawei.pdf
│       ├── readme.md
│       ├── stereo_vo
│       └── tag_detector
├── readme.md
└── testing
    └── angular_velocity.m
```

## Academic Integrity Policy

Students at Hong Kong University of Science and Technology are expected to produce their own  original academic work. Please think carefully when you are using the  codes and do not violate academic integrity policy.
