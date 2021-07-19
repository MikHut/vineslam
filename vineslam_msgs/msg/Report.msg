# VineSLAM report msg

std_msgs/Header header

## Particles 
Particle[] particles

## VineSLAM logs
std_msgs/String log

## VineSLAM features in use
std_msgs/Bool use_semantic_features
std_msgs/Bool use_lidar_features
std_msgs/Bool use_image_features
std_msgs/Bool use_gps

# GNSS heading
float32 gps_heading