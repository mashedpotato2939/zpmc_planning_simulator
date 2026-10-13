# ZPMC HD Map tile information
# Published by zpmc_map_loader_node with transient_local QoS.
# Downstream nodes use tile_directory to load their own HdMap instance.

std_msgs/Header header
string tile_directory         # Path to directory containing tile_*.bin + map_header.bin
float32 tile_size             # Tile size in meters (e.g., 300.0)
string frame_id               # Coordinate frame (e.g., "map")
float64 origin_lat            # WGS-84 origin latitude
float64 origin_lon            # WGS-84 origin longitude
float64 origin_alt            # WGS-84 origin altitude
uint32 tile_count             # Number of tiles loaded
uint32 lane_count             # Total lanes across all tiles
