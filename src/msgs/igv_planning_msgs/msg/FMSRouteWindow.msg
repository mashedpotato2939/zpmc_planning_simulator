# FMS Route Window — Sliding window route data
# Aligned with FMS-NS ICD Section 2.2 Route Data.
# Designed to be generic: fields not available from a given FMS are left empty/default.

std_msgs/Header header

# ICD-aligned identification fields
string vehicle_id              # AHT_NO: vehicle name in TOS (e.g. IGV2001)
string job_id                  # JOB_ID
string job_task_id             # JOB_TASK_ID
string route_id                # ROUTE_ID: unique ID for global route planning
string request_id              # AHT_REQUEST_ID: NS detour request ID (empty if not requested)

# Path data — 200m sliding window cut at 1Hz (per ICD)
# Segments are divided by physical heading direction (forward/reverse)
igv_planning_msgs/FMSPathSegment[] segments

# Route status
uint8 STATUS_ACTIVE=0
uint8 STATUS_PAUSED=1
uint8 STATUS_COMPLETED=2
uint8 STATUS_CANCELLED=3
uint8 status
