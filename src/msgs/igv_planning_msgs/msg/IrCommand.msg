# IR-aligned command. Out-of-band control to active mission.

uint8 KIND_PAUSE=0
uint8 KIND_RESUME=1
uint8 KIND_ABORT=2
uint8 KIND_ESTOP=3
uint8 KIND_CLEAR_FAULT=4

string mission_id
uint8 kind
uint64 issued_at_ms
string source_adapter
