# Segment Waypoint — Individual path point within a segment
# Aligned with FMS-NS ICD Section 2.2 SEGMENT_POINTS
# Named SegmentWaypoint to avoid conflict with existing FMSWaypoint.msg

# Core fields (from ICD §2.2 SEGMENT_POINTS)
float64 x                      # POS_X: UTM X coordinate (meters)
float64 y                      # POS_Y: UTM Y coordinate (meters)
float64 z                      # Altitude (meters, default 0 if not provided)
float64 heading                # Logical heading: direction of travel (radians, UTM 0=east, CCW+)
                               # For reverse segments, this is the travel direction, NOT vehicle front direction
float64 speed_limit            # SPEED_UPPER_LIMIT: speed limit at this point (m/s)

# Point type — matches ICD POINT_TYPE values
uint8 POINT_NORMAL=0           # Navigation reference point in lane
uint8 POINT_DESTINATION=1      # Destination point
uint8 POINT_LC_START=2         # Lane-change departure point
uint8 POINT_LC_END=3           # Lane-change end point (pairs with 2)
uint8 POINT_POI=4              # Point of interest
uint8 POINT_LOCAL_PLAN_START=5  # Local path planning start point
uint8 POINT_LOCAL_PLAN_END=6   # Local path planning end point
uint8 POINT_DRAWING=7          # Drawing point
uint8 point_type
