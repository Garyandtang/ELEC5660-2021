[toc]

# Source code for ELEC5660-2021

Source code of ELEC5660 Introduction to Aerial Robotics by [Prof. Shaojie Shen](https://facultyprofiles.ust.hk/profiles.php?profile=shaojie-shen-eeshaojie/) at HKUST.

This year we totally have 4 projects in this course, including 

- [x] Project 1 (Control and Planning)
  - [x] PID Quadrotor trajectory tracking control
  - [x] Minimum snap optimization-based trajectory generation
  - [x] Optimal path planning (A* + Dijkstra)
- [x] Project 2 (Visual estimator)
  - [x] 3D-2D Pose Estimation with Direct  Linear Transform (DLT)
  - [x] Visual Odometry with Stereo Camera
- [ ] Project 3 (EKF sensor fusion)
- [ ] Project 4 (System integration on-board the drone)



## Project 1 (Control and Planning)

The project 1 is done in **MATLAB simulator**, where I implemented 1) Quadrotor trajectory tracking control; 2) Optimization-based trajectory generation;. 3) path planning + trajectory generation + control.

#### Phase 1 Result

![phase_1](project1/proj1phase1/code/img/diamond.jpg)

#### Phase 2 Result

![phase_2](project1/proj1phase2/code/img/path4.jpg)

#### Phase 3 Result

![phase_1](project1/proj1phase3/code/img/path2.jpg)

## Project 2 (Visual estimator)

The project 1 is done in **ROS ** with offline dataset. The code is done with C++. Two sub tasks are:

1. 3D-2D Pose Estimation with Direct  Linear Transform (DLT)
2. Visual Odometry with Stereo Camera

#### Phase 1 Result

![phase_1](project2/project2phase1/tag_detector/document/proj2phase1_result.png)

#### Phase 2 Result

![phase_1](project2/project2phase2/img/project2phase2_result.png)



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
│   │   ├── project2pahse1_jiawei.pdf
│   │   └── tag_detector
│   └── project2phase2
│       ├── assignment.pdf
│       ├── camera_models
│       ├── img
│       ├── project2pahse2_Jiawei.md
│       ├── project2pahse2_Jiawei.pdf
│       └── stereo_vo_estimator
├── readme.md
└── testing
    └── angular_velocity.m


```

