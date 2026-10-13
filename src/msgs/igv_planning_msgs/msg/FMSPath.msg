# FMS Path Message
# Represents a path (sequence of waypoints) from the Fleet Management System

std_msgs/Header header

# Unique identifier for this path
string path_id

# Task/mission identifier
string task_id

# Sequence of waypoints
igv_planning_msgs/FMSWaypoint[] waypoints

# Path metadata
float64 total_length          # Total path length in meters
float64 estimated_duration    # Estimated duration in seconds

# Left and right bounds (optional, for drivable area)
geometry_msgs/Point[] left_bound
geometry_msgs/Point[] right_bound

# Path status
uint8 STATUS_ACTIVE = 0
uint8 STATUS_PAUSED = 1
uint8 STATUS_CANCELLED = 2
uint8 STATUS_COMPLETED = 3
uint8 status
