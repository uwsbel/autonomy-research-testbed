# This message defines a Region of Interest (Roi)
#
# An Roi can represent an area for which a detection algorithm should take place
# 	An example of this would be a segmentation algorithm finding the road and then
# 	a detector is deployed to find road signals (bike lane, turn paint symbol, etc.)
#
# Further, an Roi can define an unrefined bounding box for which further classification is performed
# 	An example of this would be a traffic light detector that finds all possible
# 	traffic lights in some image. These bounding boxes the detector finds would be Rois
# 	and then a classifier would use these Rois to determine the traffic light state (green, yellow, left turn signal, etc.)

# A list of possible classifications for this Roi
art_perception_msgs/ObjectClassification classification

# The outline of the Roi in image coordinates assuming aabb (axis aligned bounding box)
geometry_msgs/Point bottom_left
geometry_msgs/Point top_right 
