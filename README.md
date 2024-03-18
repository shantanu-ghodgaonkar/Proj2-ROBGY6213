# Vision Based 3-D Pose Estimator
This repository contains the code and resources for ROB-6213 Project 2, which focuses on implementing a vision-based 3D pose estimator for a Nano+ quadrotor using AprilTags.

## Project Overview

This project aims to estimate the position and orientation (pose) of a Nano+ quadrotor in real-time using a monocular camera and AprilTags. AprilTags are fiducial markers that can be easily detected and identified in images, allowing for accurate pose estimation.

## Data Description

The data provided includes:

AprilTag Mat: A physical layout with AprilTags arranged in a 12x9 grid (Figure 1).
Calibration Files: Intrinsic camera calibration matrix and camera-to-robot transformation stored in parameters.txt.
Image Data: Mat files containing image data for each trial. Each file includes:
Timestamp (seconds)
AprilTag IDs observed in the image
Center and corner locations of each observed AprilTag in image coordinates
Rectified image (not required for this phase)
Vicon Data: Ground truth measurements stored in two variables:
time: timestamps
vicon: data containing position, orientation, and velocities in the world frame
## Project Tasks

### Pose Estimation (estimatePose.m)

Implement an algorithm to estimate the pose of the quadrotor for each image frame based on detected AprilTags and their world frame locations.
Utilize the camera calibration data and corner locations.
### Corner Extraction and Tracking (getCorners.m)

Choose and implement a method to extract corners from each image, including AprilTag corners and random scribbles.
Track these corners across consecutive frames using the KLT tracker or a similar approach.
### Velocity Estimation (velocityRANSAC.m)

Based on the tracked corners and optical flow, estimate the linear and angular velocities of the quadrotor using Eq. (1) provided.
Implement RANSAC to reject outliers in the optical flow data.

## References

AprilTags: https://april.eecs.umich.edu/media/apriltag/?C=S;O=A
MATLAB Image Processing Toolbox: https://www.mathworks.com/products/image.html
MATLAB Computer Vision System Toolbox: https://www.mathworks.com/products/computer-vision.html
VLFeat: https://www.vlfeat.org/
B. D. Lucas and T. Kanade, "An iterative image registration technique with an application to stereo vision," in Proc. of the Intl. Joint Conf. on Artificial Intelligence, 1981.
Disclaimer

*The code provided is for educational purposes only. You may need to modify or adapt it for specific applications.