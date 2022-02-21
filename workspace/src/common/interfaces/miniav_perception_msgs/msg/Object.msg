#	This message defines an "Object"
#
# An "Object" is defined as a detected, and possibly further classified, entity of interest
# that can be passed between nodes. A detection is simply an identification of an entity
# in an image or pointcloud. A classification is a further "binning" of a detection, such as
# a traffic light being detected and further being classified as being green.
#
# An Object is also defined as in 3D space relative to the vehicle.

# Unique id
unique_identifier_msgs/UUID uuid

# A list of classifications for this Object
miniav_perception_msgs/ObjectClassification classification

# The position and orientation of the detected Object
geometry_msgs/Pose pose

# The twist (derivative of pose) of the detected Object
geometry_msgs/Twist twist

