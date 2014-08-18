# Message for annotated vehicles; intended to be published from the planner
# to indicate how it is tracking and behaving with respect to the other
# vehicles on the road

# Design notes:
#  this should include some tags for vehicles, to indicate things like if
#  a vehicle is being waited for at an intersection, or if we're following
#  or yeilding to another vehicle
#   - I'm not quite sure how to draw these yet. perhaps a pile of words, or
#     perhaps as small icons above each vehicle
#
#  this should also include the planner's prediction(s) for a vehicle's future
#  position. It should be rich enough to represent multiple, probabilistically
#  weighted outcomes
#
#  this should also include specific information about which point on which
#  lane a vehicle is matched to - this will help debug the matching code.
#   this is probably best drawn by the visualizer as a line or arrow connecting
#   the center of the vehicle's current position with the matched lane position
#
# This message replaces the current system of publishing markers on many
#  topics to represent various internal states of the planner
#
# The planner should publish an array message with a header and an array of
#  AnnotatedVehicles, which consists of its internal state; any vehicle not
#  in the current array is implicitly removed from the visualization (instead 
#  of requriring an explicit delete, as the current markers do).
#
# I expect this message deifnition to change and grow as I find additional
#  things that need to be added to it

# original track ID
uint64 id

# original track type
uint8 type

# annotation tags
string[] tags

# the vehicle's current position and size
geometry_msgs/Pose pose
geometry_msgs/Vector3 size

# the vehicle's position on the RNDF
geometry_msgs/Point rndf_point

# predictions about the vehicle's future movement
ObstaclePrediction[] predictions
