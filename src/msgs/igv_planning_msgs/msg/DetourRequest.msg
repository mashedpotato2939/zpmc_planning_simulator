# Detour Request — Vehicle detour request (ICD §1.6, triggered)

std_msgs/Header header
string vehicle_id
string request_id              # unique request ID

# Blocked area list (road closures projected to lanes)
igv_planning_msgs/DetourArea[] blocked_areas

string reason                  # description of detour reason
