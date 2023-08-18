uint8 GLOBAL_STATIC=0
uint8 GLOBAL_DYNAMIC_NOT_MOVABLE=1
uint8 GLOBAL_DYNAMINC_MOVABLE=2
uint8 GLOBAL_UNIQUE_NOT_MOVABLE=3 
uint8 GLOBAL_UNIQUE_MOVABLE=4
uint8 LOCAL_DYNAMIC_NOT_MOVABLE=5
uint8 LOCAL_DYNAMINC_MOVABLE=6
uint8 LOCAL_UNIQUE_NOT_MOVABLE=7 
uint8 LOCAL_UNIQUE_MOVABLE=8

std_msgs/Header header
string global_frame
uint64 id
uint8 type
geometry_msgs/Point[] footprint
geometry_msgs/Pose pose;
geometry_msgs/Twist velocity;