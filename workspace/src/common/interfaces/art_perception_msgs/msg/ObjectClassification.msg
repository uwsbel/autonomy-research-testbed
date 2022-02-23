# This message defines an object classification

# Value between 0 and 1 defining the percent confidence of the detection
float32 confidence

# The classification types
# See ART_OBJECT_CLASSIFICATION below
uint8 classification 

# Further custom classification types to be used to further define an object
# 	Example use case is a traffic light, which can be defined as green, red, etc.
# 	This variable should then be set to a different enum, as desired
uint8 custom_classification 

# THe possible classification types
uint8 ART_OBJECT_CLASSIFICATION_UNKNOWN=0
uint8 ART_OBJECT_CLASSIFICATION_RED_CONE=1
uint8 ART_OBJECT_CLASSIFICATION_GREEN_CONE=2
