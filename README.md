# simpl_ekf
ROS package.
Implements an Extended Kalman Filter for attitude estimation. 

Only uses the accelerometer measurements with a simplified model to compute pitch and roll of the device.

### How to run the package
rosrun simpl_ekf simpl_ekf

## Assumptions: 

-No movement (linear & angular) of the device
  - The model is therefore overly simplified.
