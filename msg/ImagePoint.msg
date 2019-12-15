# This message stores details about a point of interest
# The details are obtained from a 2D image
string name

# Meant for properties like left or right corner in case of gate/parking
string description

# Center's location {x is distance from extreme left, y is distance from extreme top, z is distance from camera}
geometry_msgs/Vector3 position
# Size of the shape { size does not use z. x and y are in image frame}
geometry_msgs/Vector3 shape

# Orientation {Range: 0 to 180; 90 vertical}
float32 orientation

# Now new msg with orientation having y p r
# for backward compatability, the original orientation is also left
# But start using the orientation3
auv_msgs/Orientation orientation3

#confidence {Range 0 to 1}
#Can be said as the confidence of detection(equivalent to probability)
float32 confidence

#cameraPos(front or bottom), the front camera or bottom camera frame
#in which the image is found
string cameraPos
