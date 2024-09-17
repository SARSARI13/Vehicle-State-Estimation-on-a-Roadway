# Trajectory Estimation and Visualization


## Overview
This project uses sensor data to estimate and visualize the trajectory of a moving object. The following sensors are used:
- IMU (Inertial Measurement Unit)
- GNSS (Global Navigation Satellite System)
- LIDAR (Light Detection and Ranging)




## Data

The data used in this project is stored in a pickle file named `data/pt1_data.pkl`. The dataset contains:
- `gt`: Ground truth trajectory data.
- `imu_f`: IMU accelerometer data.
- `imu_w`: IMU gyroscope data.
- `gnss`: GNSS position data.
- `lidar`: LIDAR point cloud data.

## Dependencies

This project requires the following Python packages:

- `numpy`
- `matplotlib`
- `pickle` (part of the Python standard library)
- `mpl_toolkits` (part of `matplotlib`)
- `rotations` (a custom module for quaternion and rotation operations)

## Program Results 

The results are visualized in a 3D plot showing the estimated trajectory and the ground truth trajectory. The plot uses Matplotlib's 3D plotting capabilities.

![Ground Truth vs Estimated Trajectory](data/Results.PNG)

## Contributing

Feel free to fork this project, submit issues and pull requests. Contributions are welcome!
