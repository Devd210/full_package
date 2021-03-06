# robot status for UI

# chief executive mode
uint8 chief_executive_mode

# 0 - 100
int32 battery_status

# 0 means no duty set / any positive integer represents a duty
uint32 current_duty_id

# if no duty set this is gibberish otherwise it represents the task id currently running
uint32 current_task_id

# if no duty set this is gibberish otherwise it represents type of current task 0 == route 1 == attachement task
uint8 task_type

# 0 == all is well any positive int means there is an issue for now just display the number
int32 error_code
