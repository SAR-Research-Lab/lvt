# LVT
Lightweight Visual Tracking (LVT) is a real-time feature-based visual odometry system, supporting both Stereo and RGB-D sensors. The system is described in the following paper:
> Mohamed Aladem and Samir A. Rawashdeh, "Lightweight Visual Odometry for Autonomous Mobile Robots". Sensors 2018, 18(9).

A video demonstrating the system:

[![LVT demo](http://img.youtube.com/vi/t2gr6y90aWI/0.jpg)](http://www.youtube.com/watch?v=t2gr6y90aWI)

# Building
LVT uses CMake build system. It was built successfully on Linux Ubuntu 16.04 and Windows 10 using MSVC 2017. The build dependencies are:
* [Eigen3](http://eigen.tuxfamily.org/index.php?title=Main_Page)
* [OpenCV3](https://github.com/opencv/opencv) and [OpenCV3 contrib](https://github.com/opencv/opencv_contrib) which is needed for the BRIEF descriptor extractor.
* [G2O](https://github.com/RainerKuemmerle/g2o)
* [Pangolin](https://github.com/stevenlovegrove/Pangolin) - OPTIONAL, only needed if LVT is built with visualization enabled.

Building then proceeds like any typical CMake-based project using the provided top-level CMakeLists file, which will generate lvt as a static library, lvt_c which is a C-interface for lvt as a shared library, and the executable example projects in their source directories.
# Using ROS
LVT optionally supports the Robot Operating System (ROS) and can build a node compatible with it. It was tested and working successfully with ROS Kinetic Kame. The subdirectory _lvt_ is actually a compatible ROS package which can be copied into your workspace and built directly using catkin. 
