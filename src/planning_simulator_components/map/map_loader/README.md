# map_loader

The Lanelet2 loader reads OSM maps with `local_x`/`local_y` coordinates and publishes
`autoware_map_msgs/msg/LaneletMapBin`. The visualization node publishes lane
boundaries and centerlines as RViz markers. The pointcloud loader publishes a
`map`-frame `sensor_msgs/msg/PointCloud2` from a PCD file.

For the Nanhui planning map, launch both maps with the planning launch file:

```bash
ros2 launch igv_launch igv_planning.launch.xml
```

The map files default to the installed `nanhui_osm_v1` package. Override
`map_path`, `lanelet2_map_file`, and `pointcloud_map_file` when needed.
The vector map, markers, and pointcloud are published on
`/map/vector_map_humble`, `/map/vector_map_marker_humble`, and
`/map/pointcloud_map_humble` respectively. All three publishers use transient
local durability so RViz and the planner can start after the loaders.
