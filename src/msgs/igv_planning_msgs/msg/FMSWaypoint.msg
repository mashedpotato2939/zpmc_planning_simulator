# FMS Waypoint Message
# Represents a single waypoint from the Fleet Management System

# Position in map frame
float64 x
float64 y
float64 z

# Orientation (yaw angle in radians, 0 = East, counter-clockwise positive)
float64 yaw

# Velocity at this waypoint (m/s), 0 or negative means use default
float64 velocity

# Acceleration at this waypoint (m/s^2), optional
float64 acceleration

# Curvature at this waypoint (1/m), optional
float64 curvature

# Time from trajectory start to reach this point (seconds), optional
float64 time_from_start

# Waypoint type
uint8 TYPE_NORMAL = 0
uint8 TYPE_STOP = 1
uint8 TYPE_SLOW = 2
uint8 TYPE_GOAL = 3
uint8 waypoint_type

# Lane ID if available (for map matching), 0 means not specified
int64 lane_id

# Driving direction (true = reverse/backward driving)
bool is_reverse

# Lane change type - specifies the type of lane change maneuver
uint8 LANE_CHANGE_NONE = 0      # Not a lane change waypoint
uint8 LANE_CHANGE_OBLIQUE = 1   # Oblique (crab) - constant heading, diagonal movement with 4WS
uint8 LANE_CHANGE_S_CURVE = 2   # S-curve - traditional lane change with heading change
uint8 lane_change_type

# Lane change segment markers
bool is_lane_change_start       # True for the first waypoint of lane change segment
bool is_lane_change_end         # True for the last waypoint of lane change segment
