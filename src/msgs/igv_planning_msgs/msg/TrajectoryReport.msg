# Trajectory Report — Planned trajectory report (ICD §1.2, 1Hz)

std_msgs/Header header
string vehicle_id

# Ego state
float64 pos_x
float64 pos_y
float64 heading_angle
float64 speed

# Planned trajectory (100m ahead, from current output path)
igv_planning_msgs/TrajectoryPoint[] points

# Stop flag (ICD §1.2 STOP_FLAG)
bool stop_flag                 # true = vehicle planning long-term stop
string stop_reason             # reason for long-term stop
