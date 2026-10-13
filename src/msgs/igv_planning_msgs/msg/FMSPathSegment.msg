# FMS Path Segment — Path segment with orientation
# Matches ICD ROUTE_PATH segment structure.
# Each segment has a consistent physical heading direction.

# Segment orientation — matches ICD ORIENTATION field
uint8 ORIENT_FORWARD=0         # Logical heading = physical heading
uint8 ORIENT_REVERSE=1         # Logical heading opposite to physical heading
uint8 orientation

# Points in this segment
igv_planning_msgs/SegmentWaypoint[] points
