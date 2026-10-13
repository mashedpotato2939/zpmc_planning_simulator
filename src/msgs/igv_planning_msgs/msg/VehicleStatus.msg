# Vehicle Status Report — Uplink to FMS (ICD §1.1, 1Hz)

std_msgs/Header header
string vehicle_id              # AHT_NO

# Position and motion
float64 pos_x                  # UTM X (vehicle center for IGV)
float64 pos_y                  # UTM Y
float64 altitude               # meters
float64 heading_angle          # logical heading (radians, UTM 0°=east, CCW+)
float64 physical_angle         # physical head orientation (radians)
float64 speed                  # m/s

# Vehicle state
uint8 GEAR_PARK=0
uint8 GEAR_FORWARD=1
uint8 GEAR_REVERSE=2
uint8 GEAR_NEUTRAL=3
uint8 gear_mode

uint8 CONTROL_AUTO=0
uint8 CONTROL_MANUAL=1
uint8 CONTROL_REMOTE=2
uint8 CONTROL_IDLE=3
uint8 CONTROL_STANDSTILL=4
uint8 control_mode

# Job tracking
string job_id
string job_task_id
string route_id

# Route execution status
uint8 ROUTE_UNKNOWN=0
uint8 ROUTE_EXECUTING=2
uint8 ROUTE_FAILED=3
uint8 route_task_state

# Container (optional, default 0)
uint8 container_status
