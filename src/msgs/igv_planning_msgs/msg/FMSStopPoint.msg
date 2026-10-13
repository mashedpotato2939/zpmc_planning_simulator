# FMS Stop Point — Stop point command
# Aligned with ICD Section 2.3 Stop Point Data.

std_msgs/Header header
string vehicle_id              # AHT_NO
float64 x                      # POS_X: UTM X coordinate
float64 y                      # POS_Y: UTM Y coordinate
bool clear                     # true = clear stop point (resume)
