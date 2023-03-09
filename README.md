# A Motion Estimation Filter for Inertial Measurement Unit With On-Board Ferromagnetic Materials
## Introduction

Magnetic distortions due to existing appliances and on-board objects with ferromagnetic materials cause serious bias and deviations in motion estimation by inertial measurement sensors with magnetometers. This problem requires a proper sensor fusion to do motion tracking with a minimal angular error. This letter presents a design of a complementary filter that compensates the strong magnetic effects of on-board ferromagnetic materials. Not only the attached permanent magnets may have serious biases on the magnetometer axes but also there is a magnetic distortion due to soft ferromagnetic materials, i.e., steel. After defining the signals of the inertial/magnetic sensors, the process and measurement models are described and a Kalman filter is constructed. The designed filter can be used for motion tracking in environments with magnetic distortions, and in robot actuators with magnetic parts. The performance of the proposed filter is verified under experiment and compared with conventional filters. Finally, we raise a question about whether the attachment of permanent magnets to inertial measurement sensors can serve as a magnetic shield improving the motion estimation

## Concept and Video Presentation
![](/image.png)
![](/image.gif)



## Usage

To use the proposed Kalman-based filter code, please refer to the main code `Offline_IMU_DCM_COMP.m`. The code explains the details of how to run the filter. 

## Dependencies

There is no dependencies for the offline code. However, in order to run the real-time version, the following packages are required:

- Requires an IMU with magnetometer sensor for real experiment. 

## Website 
(https://www.amirtafrishi.com/)


## Contributors

- **Seyed Amir Tafrishi**, Engineering School, Cardiff University, UK,
- **Mikhail Svinin**,  Information Science and Engineering Department, Ritsumeikan University, Japan
- **Motoji Yamamoto**, Mechanical Engineering Department, Kyushu Univeristy, Japan.

##  Publications

- S. A. Tafrishi, M. Svinin and M. Yamamoto, 2021. [A Motion Estimation Filter for Inertial Measurement Unit With On-Board Ferromagnetic Materials](10.1109/LRA.2021.3067301). IEEE Robotics and Automation Letters, vol. 6, no. 3, pp. 4939-4946.

- S. A. Tafrishi, M. Svinin and M. Yamamoto, 2021. [A Motion Estimation Filter for Inertial Measurement Unit With On-Board Ferromagnetic Materials](https://ras.papercept.net/conferences/conferences/ICRA21/program/ICRA21_ContentListWeb_3.html). Presented in 2021 International Conference on Robotics and Automation (ICRA2022). 


 

 
