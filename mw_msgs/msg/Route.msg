# Unique ID for each different route
uint32 id

# Header
std_msgs/Header header

# Array of waypoints
geometry_msgs/PoseStamped[] waypoints

# Route status
uint8 status

# Enumeration for different route states
uint8 NOT_STARTED=0
uint8 WIP=1
uint8 COMPLETED=2