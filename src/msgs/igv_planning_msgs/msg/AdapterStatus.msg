# Adapter Status — Internal adapter diagnostics

std_msgs/Header header
bool map_loaded
bool route_active
bool using_normal_mode
string mode_reason
string current_route_id
string current_job_id
uint8 state
float64 route_progress
bool stop_active
string last_error

uint8 STATE_IDLE=0
uint8 STATE_ACTIVE=1
uint8 STATE_SUSPENDED=2
uint8 STATE_ERROR=3
