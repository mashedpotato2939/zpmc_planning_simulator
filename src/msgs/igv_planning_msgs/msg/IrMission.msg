# IR-aligned mission. Published by vehicle_agent's RosPublisher to drive
# the planning bridge (fms_path_converter_node).

uint8 KIND_ROUTE=0
uint8 KIND_STOP_AT=1
uint8 KIND_RESUME=2
uint8 KIND_ABORT=3
uint8 KIND_PARK=4

string id
uint8 kind
uint64 seq
uint64 issued_at_ms
IrWaypoint[] waypoints
uint64 deadline_ms      # 0 = no deadline
string source_adapter
