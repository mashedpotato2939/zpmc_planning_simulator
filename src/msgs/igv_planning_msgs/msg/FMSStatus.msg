# FMS Status Message
# Status information published by the FMS bridge

std_msgs/Header header

# Connection status
bool fms_connected
float64 last_message_time      # Time since last FMS message (seconds)

# Current trajectory info
string current_trajectory_id
string current_task_id
uint32 waypoints_remaining
float64 distance_to_goal       # meters

# Bridge mode
uint8 MODE_DISABLED = 0
uint8 MODE_FMS_ONLY = 1
uint8 MODE_AUTOWARE_ONLY = 2
uint8 MODE_FMS_WITH_SAFETY = 3   # FMS path + Autoware safety (Option B)
uint8 current_mode

# Health status
bool trajectory_valid
bool trajectory_stale          # True if trajectory not updated recently
string last_error
