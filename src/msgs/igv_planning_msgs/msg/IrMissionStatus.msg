# IR-aligned mission status. Mirrors vehicle_agent::ir::MissionState (9 states).

uint8 STATE_IDLE=0
uint8 STATE_RECEIVED=1
uint8 STATE_VALIDATING=2
uint8 STATE_EXECUTING=3
uint8 STATE_PAUSED=4
uint8 STATE_ABORTING=5
uint8 STATE_PREEMPTED=6
uint8 STATE_DONE=7
uint8 STATE_FAILED=8

string mission_id
uint8 state
uint64 stamp_ms
float64 progress      # 0..1
string detail
