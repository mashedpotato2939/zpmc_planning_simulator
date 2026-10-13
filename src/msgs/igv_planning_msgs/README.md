# igv_planning_msgs

Custom ROS 2 message definitions for the IGV FMS planning chain. This is a pure
interface package (no nodes) so that producers and consumers can share the
message types without pulling in any node code.

## Consumers

- **`igv_path_planner`** — FMS route/path/uplink messages
  (`FMSRouteWindow`, `FMSStopPoint`, `FMSPath`, `VehicleStatus`,
  `TrajectoryReport`, `JobEvent`, `AdapterStatus`, …).
- **`vehicle_agent`** — IR mission/command messages
  (`IrMission`, `IrCommand`, `IrMissionStatus`, `IrWaypoint`).
- **`fms_bridge_rviz_plugin`** — route-window / stop-point / adapter-status
  visualization.

## Build

```bash
colcon build --packages-select igv_planning_msgs
```

C++ messages are generated under the `igv_planning_msgs::msg` namespace, e.g.
`#include <igv_planning_msgs/msg/fms_route_window.hpp>`.
