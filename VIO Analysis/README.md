## Overview

**Author: Cagri Kilic<br />
Affiliation: [WVU NAVLAB](https://navigationlab.wvu.edu/)<br />
Maintainer: Cagri Kilic, cakilic@mix.wvu.edu**

Supplementary Visual-Inertial Odometry (VIO) Analyses in low-feature environment for the Slip-Based Autonomous ZUPT through Gaussian Process to Improve Planetary Rover Proprioceptive Localization paper.

The VIO solution is generated using ROS Wrapper for Intel® RealSense™ Devices https://github.com/IntelRealSense/realsense-ros

Each figure provides a DGPS solution, filter estimation (corenav-GP solution), 2D Dead Reckoning (only WO and IMU heading). The Google Map representation of the ground truth (DGPS) solution is provided to visualize the low-feature environment (Point Marion, PA, Ashpiles Mars Analog Environment). Solution accuracy values are given for the corenav-GP implementation.
 
### Scenario 1, Execution 1
<p align="center">
<img alt="architecture" src="docs/scenario1.PNG" width="700">
</p>        

*VIO failed after traversing 124m

### Scenario 1, Execution 2
<p align="center">
<img alt="architecture" src="docs/scenario2.PNG" width="700">
</p>        


### Scenario 1, Execution 3
<p align="center">
<img alt="architecture" src="docs/scenario3.PNG" width="700">
</p>        


### Additional Scenario 1
<p align="center">
<img alt="architecture" src="docs/scenario4.PNG" width="700">
</p>        


### Additional Scenario 2
<p align="center">
<img alt="architecture" src="docs/scenario5.PNG" width="700">
</p>        


