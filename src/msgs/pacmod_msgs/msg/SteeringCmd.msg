# User command to control steering system

std_msgs/Header header

bool enable             # Set true to enable steering system
bool ignore_overrides   # Set true for steering system to ignore steering overrides
bool clear_override     # Set true to clear steering override flag

float64 axis1_command   # Set steering command in rad in the range specified in user manual
float64 axis2_command
float64 axis3_command
float64 axis4_command
float64 rotation_rate   # Set rotation rate in rad/sec in the range specified in user manual
