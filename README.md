# Odometry

A basic implementation for robot odometry using custom encoder configuration.
The configuration uses three encoders paired with omni-wheels that are mounted at equal distance from the chassis center.
The traditional XY encoder configuration fails if the robot is being rotated about its Z axis (yaw movement), hence the addition of the third encoder set mounted parallel to either of the X or Y axis set helps distinguish between diagonal and yaw movements of the chassis.

The class is tailored to be useful for cometitive robotics applications with arduino controllers.
