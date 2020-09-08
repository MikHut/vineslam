# Semantic feature ROS message type

# ID of the 2D feature
uint16 id

# Class of the 2D feature
uint8 semantic_class

# Cartesian position of the 2D feature
geometry_msgs/Point posXYZ

# Geographic position of the 2D landmark (GPS)
sensor_msgs/NavSatFix posGPS

# Position of the 2D feature for visualization
visualization_msgs/Marker pos
