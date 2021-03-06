# Unique ID for each task
uint32 id

# Route field when type is ROUTE
mw_msgs/Route route

# Duration (in seconds) to run this task (when type is not ROUTE)
float32 duration

# End behaviour, ie, time to wait after this task ends and before starting the next task
float32 end_behaviour

# Type of task
uint8 type

# Enumeration for different types
uint8 ROUTE=0
uint8 WAIT=1

# Task status
uint8 status

# Enumeration for different task states
uint8 NOT_STARTED=0
uint8 WIP=1
uint8 COMPLETED=2