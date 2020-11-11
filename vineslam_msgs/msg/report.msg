# VineSLAM report msg

std_msgs/Header header

## Particles before resampling
particle[] b_particles

## Particles after resampling
particle[] a_particles

## VineSLAM logs
std_msgs/String log

## VineSLAM features in use
std_msgs/Bool use_high_level
std_msgs/Bool use_corners
std_msgs/Bool use_planars
std_msgs/Bool use_planes
std_msgs/Bool use_ground
std_msgs/Bool use_icp
std_msgs/Bool use_gps