Header header

geometry_msgs/Quaternion quaternion
geometry_msgs/Vector3 acceleration
geometry_msgs/Vector3 angular_velocity
geometry_msgs/Vector3 position
geometry_msgs/Vector3 velocity
auv_msgs/Orientation orientation

#geometry_msgs/Vector3 relative_position_Bframe
#geometry_msgs/Vector3 velocity_Bframe

float64 height
float64 depth

uint8 LOCAL = 0
uint8 GLOBAL = 1
uint8 frame

# to be used only in tf relevant conditions
string origin  # name of frame, unless unspecified, then it means "world"
