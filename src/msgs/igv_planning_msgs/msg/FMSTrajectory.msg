# FMS Trajectory Message
# Represents a complete trajectory with timing from the Fleet Management System

std_msgs/Header header

# Unique identifier for this trajectory
string trajectory_id

# Task/mission identifier
string task_id

# Sequence of waypoints with timing
igv_planning_msgs/FMSWaypoint[] waypoints

# Trajectory metadata
float64 total_length          # Total trajectory length in meters
float64 total_duration        # Total duration in seconds
float64 max_velocity          # Maximum velocity in m/s
float64 max_acceleration      # Maximum acceleration in m/s^2
float64 max_deceleration      # Maximum deceleration in m/s^2 (positive value)

# Trajectory validity time window
builtin_interfaces/Time valid_from
builtin_interfaces/Time valid_until

# Trajectory status
uint8 STATUS_ACTIVE = 0
uint8 STATUS_PAUSED = 1
uint8 STATUS_CANCELLED = 2
uint8 STATUS_COMPLETED = 3
uint8 STATUS_INVALID = 4
uint8 status

# Priority level (higher = more important)
uint8 priority
