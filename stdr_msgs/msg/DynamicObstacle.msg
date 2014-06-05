# Message for dynamic obstacles.
# The perception module publishes all known obstacles, which includes tracks
# that we are not directly observing at the moment


# track ID
uint64 id

# this is equal to 1 for obstacles that we are currently observing, and it
# decreases for tracks that we haven't observed for a while. When it reaches
# 0 the obstacle is removed.
float32 track_confidence



# classification result (track-wise)
# the value is drawn from an enum in obstacle_types.h
uint8 type

# the associated confidence in the type
float32 type_confidence



# position, orientation, velocity and variances
nav_msgs/Odometry odom

# dimensions of the bounding box in the obstacle's local frame (as defined
# by its pose)
geometry_msgs/Vector3 dimension



# Possible turn_signal values
uint8 TURN_SIG_UNKNOWN = 0
uint8 TURN_SIG_NONE    = 1
uint8 TURN_SIG_LEFT    = 2
uint8 TURN_SIG_RIGHT   = 3
uint8 TURN_SIG_BOTH    = 4

uint8 turn_signal
