# Vision-Based 3D Attitude Estimation Using AprilTags

## Project Overview

This project presents a vision-based system for **3D attitude estimation** (position, orientation, and velocity) of a Nano+ quadrotor using **AprilTags**. The system leverages camera calibration data and AprilTag information to estimate the quadrotor's pose and compute its velocity using **optical flow** and **RANSAC** to remove outliers.

The project is divided into two main parts:
1. **Part 1**: Estimating the **position and orientation** of the quadrotor using AprilTags and homography.
2. **Part 2**: Estimating **velocity and angular velocity** using optical flow and RANSAC to improve robustness.

## Key Features

- **AprilTag Detection**: Detects AprilTags in the camera frame to estimate the quadrotor's pose.
- **Pose Estimation**: Uses homography and camera calibration data to estimate the quadrotor’s position and orientation.
- **Velocity Estimation**: Computes velocity using optical flow techniques, with **RANSAC** to reduce outliers.
- **Robust Vision-Based System**: Leverages camera images and corner feature tracking to estimate quadrotor attitude in real-time.

## Methodology

### Part 1: Pose Estimation

In this part, we detect AprilTags and estimate the pose of the quadrotor using homography. The steps include:
- **AprilTag Corner Detection**: Extracts the corner points of each detected AprilTag in the image.
- **Homography Calculation**: Establishes a relationship between world coordinates and camera coordinates.
- **Pose Extraction**: Extracts the rotation and translation from the homography matrix to estimate the 3D pose (position and orientation) of the quadrotor.

### Part 2: Velocity Estimation with RANSAC

For velocity estimation, optical flow is computed to track the motion of corner features between consecutive frames. The steps include:
- **Optical Flow Calculation**: Tracks the motion of corner features between frames to compute the quadrotor’s linear and angular velocity.
- **RANSAC Algorithm**: Filters outliers in the velocity estimates using RANSAC to improve robustness, particularly in noisy data scenarios.

## Results

### Part 1: Pose Estimation

The system effectively estimates the pose for the given sensor data. Below are the results for **position** and **orientation** for **Part 1**.

#### Dataset 1

<p align="center">
  <img src="https://github.com/shantanu-ghodgaonkar/Proj2-ROBGY6213/blob/d4e30e00d512ee5a1befeb83f8927bd7f5a5a846/img/plots/part1/dataset1/Position%20X.png" alt="Part 1: Position X Estimate" width="300"/>
  <img src="https://github.com/shantanu-ghodgaonkar/Proj2-ROBGY6213/blob/d4e30e00d512ee5a1befeb83f8927bd7f5a5a846/img/plots/part1/dataset1/Position%20Y.png" alt="Part 1: Position Y Estimate" width="300"/>
  <img src="https://github.com/shantanu-ghodgaonkar/Proj2-ROBGY6213/blob/d4e30e00d512ee5a1befeb83f8927bd7f5a5a846/img/plots/part1/dataset1/Position%20Z.png" alt="Part 1: Position Z Estimate" width="300"/>
</p>

<p align="center">
  <b>Fig 1:</b> Part 1 - Estimated Position X, Y, Z
</p>

<p align="center">
  <img src="https://github.com/shantanu-ghodgaonkar/Proj2-ROBGY6213/blob/d4e30e00d512ee5a1befeb83f8927bd7f5a5a846/img/plots/part1/dataset1/Orientation%20X.png" alt="Part 1: Orientation X Estimate" width="300"/>
  <img src="https://github.com/shantanu-ghodgaonkar/Proj2-ROBGY6213/blob/d4e30e00d512ee5a1befeb83f8927bd7f5a5a846/img/plots/part1/dataset1/Orientation%20Y.png" alt="Part 1: Orientation Y Estimate" width="300"/>
  <img src="https://github.com/shantanu-ghodgaonkar/Proj2-ROBGY6213/blob/d4e30e00d512ee5a1befeb83f8927bd7f5a5a846/img/plots/part1/dataset1/Orientation%20Z.png" alt="Part 1: Orientation Z Estimate" width="300"/>
</p>

<p align="center">
  <b>Fig 2:</b> Part 1 - Estimated Orientation X, Y, Z
</p>

### Part 2: Velocity and Angular Velocity Estimation with RANSAC

Below are the results for **velocity** and **angular velocity** for **Part 2** (with RANSAC).

#### Dataset 1 with RANSAC

<p align="center">
  <img src="https://github.com/shantanu-ghodgaonkar/Proj2-ROBGY6213/blob/d4e30e00d512ee5a1befeb83f8927bd7f5a5a846/img/plots/part2/wRANSAC/dataset1/Velocity%20X.png" alt="Part 2: Velocity X Estimate" width="300"/>
  <img src="https://github.com/shantanu-ghodgaonkar/Proj2-ROBGY6213/blob/d4e30e00d512ee5a1befeb83f8927bd7f5a5a846/img/plots/part2/wRANSAC/dataset1/Velocity%20Y.png" alt="Part 2: Velocity Y Estimate" width="300"/>
  <img src="https://github.com/shantanu-ghodgaonkar/Proj2-ROBGY6213/blob/d4e30e00d512ee5a1befeb83f8927bd7f5a5a846/img/plots/part2/wRANSAC/dataset1/Velocity%20Z.png" alt="Part 2: Velocity Z Estimate" width="300"/>
</p>

<p align="center">
  <b>Fig 3:</b> Part 2 - Estimated Velocity X, Y, Z (with RANSAC)
</p>

<p align="center">
  <img src="https://github.com/shantanu-ghodgaonkar/Proj2-ROBGY6213/blob/d4e30e00d512ee5a1befeb83f8927bd7f5a5a846/img/plots/part2/wRANSAC/dataset1/Angular%20Velocity%20X.png" alt="Part 2: Angular Velocity X Estimate" width="300"/>
  <img src="https://github.com/shantanu-ghodgaonkar/Proj2-ROBGY6213/blob/d4e30e00d512ee5a1befeb83f8927bd7f5a5a846/img/plots/part2/wRANSAC/dataset1/Angular%20Velocity%20Y.png" alt="Part 2: Angular Velocity Y Estimate" width="300"/>
  <img src="https://github.com/shantanu-ghodgaonkar/Proj2-ROBGY6213/blob/d4e30e00d512ee5a1befeb83f8927bd7f5a5a846/img/plots/part2/wRANSAC/dataset1/Angular%20Velocity%20Z.png" alt="Part 2: Angular Velocity Z Estimate" width="300"/>
</p>

<p align="center">
  <b>Fig 4:</b> Part 2 - Estimated Angular Velocity X, Y, Z (with RANSAC)
</p>

## Conclusion

This project successfully implements a vision-based 3D attitude estimation system for a Nano+ quadrotor using AprilTags. The system effectively estimates the position, orientation, velocity, and angular velocity of the quadrotor. RANSAC is employed to increase the robustness of velocity estimates in the presence of outliers.

## Future Enhancements

1. **Improved Feature Tracking**: Enhance the feature tracking algorithm to further reduce noise and improve accuracy.
2. **Extended Testing**: Perform additional tests with different datasets and environmental conditions to validate the robustness of the system.
3. **Sensor Fusion**: Combine vision-based estimation with data from other sensors, such as IMU, to improve the accuracy of pose and velocity estimation.

## References

1. APRIL Robotics Laboratory, University of Michigan. AprilTags Visual Fiducial System. 2010. [Link](https://april.eecs.umich.edu/software/apriltag).
2. Bruce D. Lucas and Takeo Kanade. "An Iterative Image Registration Technique with an Application to Stereo Vision". In: *International Joint Conference on Artificial Intelligence*. 1981. [Link](https://api.semanticscholar.org/CorpusID:2121536).
3. Yi Ma et al. *An Invitation to 3-D Vision: From Images to Geometric Models*. SpringerVerlag, 2003. ISBN: 0387008934.
4. MathWorks. Detect corners using Harris–Stephens algorithm. 2013. [Link](https://www.mathworks.com/help/vision/ref/detectharrisfeatures.html).
5. MathWorks. MATLAB Computer Vision Toolbox. 2024. [Link](https://www.mathworks.com/products/computer-vision.html).
6. MathWorks. MATLAB Image Processing Toolbox. 2024. [Link](https://www.mathworks.com/products/image-processing.html).
7. MathWorks. Moore-Penrose pseudoinverse. 2021. [Link](https://www.mathworks.com/help/matlab/ref/pinv.html).
8. MathWorks. Savitzky-Golay filtering. 2006. [Link](https://www.mathworks.com/help/signal/ref/sgolayfilt.html).
9. MathWorks. Track points in video using Kanade-Lucas-Tomasi (KLT) algorithm. 2012. [Link](https://www.mathworks.com/help/vision/ref/vision.pointtracker-system-object.html).
10. Bruno Siciliano et al. *Robotics: Modelling, Planning and Control*. Springer Publishing Company, 2010. ISBN: 1849966346.
11. Richard Szeliski. *Computer Vision: Algorithms and Applications*. 1st ed. Springer-Verlag, 2010. ISBN: 1848829345.
