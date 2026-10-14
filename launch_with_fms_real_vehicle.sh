#!/bin/bash

# Launch Autoware Planning Terminal for the Real Vehicle (scale IGV)
#
# This terminal owns: the selected Xiamen HD map, vehicle interface, planning (FMS),
# vehicle_agent (cloud FMS edge gateway), perception relay, system diagnostics,
# autoware API, and optionally RViz. Backed by igv_planning.launch.xml.
#
# Control is intentionally NOT launched here. In production the external
# control team supplies the full chain downstream of
# /planning/scenario_planning/trajectory (motion_control → gate → vehicle).
# During integration testing, the companion launch_fms_test_components.sh
# terminal stands in for the external control team and brings up the full
# gate / OMT / AEB / shift_decider / motion_control stack.
#
# External topics expected from other teams at runtime:
#   /localization/kinematic_state                        (localization team)
#   /perception/... (Galactic format, relayed to Humble) (perception team)
#   /control/command/control_cmd                         (external control team)

echo "======================================================================"
echo "  Launching IGV Planning Terminal (REAL SCALE IGV)"
echo "======================================================================"

# Source the workspace
source /opt/ros/galactic/setup.bash
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
source "$SCRIPT_DIR/install/setup.bash"

# ===================== PLANNING TERMINAL FLAGS =====================

IS_SIMULATION=${IS_SIMULATION:-true}
USE_SIM_TIME=${USE_SIM_TIME:-true}

LAUNCH_VEHICLE_INTERFACE=${LAUNCH_VEHICLE_INTERFACE:-true}
LAUNCH_PLANNING=${LAUNCH_PLANNING:-true}

# FMS-compatible system diagnostics (all modes always OK so Auto can engage)
LAUNCH_SYSTEM=${LAUNCH_SYSTEM:-true}

# Perception relay converts external-team Galactic msgs to Humble format
ENABLE_PERCEPTION_RELAY=${ENABLE_PERCEPTION_RELAY:-true}

# ===================== FMS CONFIGURATION (default / ICD protocol) =====================
#
# FMS_PROTOCOL_TYPE now drives both the legacy igv_path_converter plugin AND
# vehicle_agent's adapter selection. Recognised values:
#   default          → VA adapter: ahtfms_ns  (the ICD-aligned production path)
#   legacy           → VA adapter: ahtfms_ns  (kept as a one-release alias)
#   ahtfms_ns        → VA adapter: ahtfms_ns
#   internal_zpmc    → VA adapter: internal_zpmc (MQTT5 + HMAC; requires paho)
#   scenario_loopback→ VA adapter: scenario_loopback (file-driven simulation)
#
# To override the VA adapter list directly (e.g. run two adapters concurrently),
# set VEHICLE_AGENT_ADAPTERS=ahtfms_ns,scenario_loopback and the script will
# pass it through.

FMS_ENABLED=${FMS_ENABLED:-true}
FMS_PROTOCOL_TYPE=${FMS_PROTOCOL_TYPE:-default}
ROUTE_WINDOW_TOPIC=${ROUTE_WINDOW_TOPIC:-/fms/route_window}
STOP_POINT_TOPIC=${STOP_POINT_TOPIC:-/fms/stop_point}

USE_VEHICLE_AGENT=${USE_VEHICLE_AGENT:-true}
VEHICLE_AGENT_ADAPTERS=${VEHICLE_AGENT_ADAPTERS:-ros_injector}
VEHICLE_AGENT_SCENARIO_PATH=${VEHICLE_AGENT_SCENARIO_PATH:-/igv}

# Legacy fallbacks (unused when protocol=default)
FMS_TOPIC=${FMS_TOPIC:-/fms/reference_path}
USE_JSON_INPUT=${USE_JSON_INPUT:-false}

# Vestigial — igv_velocity_shaper + igv_kinematic_smoother subsume the old
# motion_velocity_planner / trajectory_relay path in FMS mode. The FMS launch
# now starts only the external velocity limit selector and planning validator.
SKIP_MOTION_VELOCITY_PLANNER=${SKIP_MOTION_VELOCITY_PLANNER:-true}
ENABLE_TRAJECTORY_RELAY=${ENABLE_TRAJECTORY_RELAY:-false}

# Drivable-area synthesis from FMS path
LANE_WIDTH=${LANE_WIDTH:-3.5}
GENERATE_DRIVABLE_AREA=${GENERATE_DRIVABLE_AREA:-true}

# Close-range obstacle handling: rasterize DetectedObjects into an identity-free
# OccupancyGrid the velocity shaper stops on (replaces the promoter). Set to
# false to disable the whole feature (raw predicted objects only).
USE_OCCUPANCY_GRID=${USE_OCCUPANCY_GRID:-true}
# Fallback occupancy source: rasterize DetectedObjects into a grid instead of
# consuming /perception/occupancy_grid_map/map. Off by default.
ENABLE_DETECTION_GRID=${ENABLE_DETECTION_GRID:-false}

# ===================== MAP AND VEHICLE =====================

MAP_FORMAT=${MAP_FORMAT:-osm}
case "$MAP_FORMAT" in
  osm|tiles) ;;
  *) echo "Invalid MAP_FORMAT=$MAP_FORMAT (expected osm or tiles)" >&2; exit 2 ;;
esac
ZPMC_TILE_DIRECTORY=${ZPMC_TILE_DIRECTORY:-$SCRIPT_DIR/src/map/xiamen_tiles_v1}
MAP_PATH=${MAP_PATH:-$SCRIPT_DIR/src/map/xiamen_tiles_v1}
LANELET2_MAP_FILE=${LANELET2_MAP_FILE:-xiamen-prod-galactic.osm}
POINTCLOUD_MAP_FILE=${POINTCLOUD_MAP_FILE:-global.pcd}
LAUNCH_POINTCLOUD_MAP=${LAUNCH_POINTCLOUD_MAP:-false}

# Real scale IGV hardware
# VEHICLE_MODEL=scale_igv_vehicle
VEHICLE_MODEL=igv_vehicle
VEHICLE_ID=${VEHICLE_ID:-v001}

# ===================== OPTIONAL =====================

LAUNCH_RVIZ=${LAUNCH_RVIZ:-true}
LOG_TO_FILE=${LOG_TO_FILE:-false}
LOG_FILE=${LOG_FILE:-/tmp/fms_real_vehicle_debug.txt}

# ===================== DISPLAY =====================

echo "Planning terminal:"
echo "  launch_planning=$LAUNCH_PLANNING  launch_vehicle_interface=$LAUNCH_VEHICLE_INTERFACE"
echo "  enable_perception_relay=$ENABLE_PERCEPTION_RELAY"
echo "  launch_system=$LAUNCH_SYSTEM  is_simulation=$IS_SIMULATION  use_sim_time=$USE_SIM_TIME"
echo ""
echo "FMS:"
echo "  protocol=$FMS_PROTOCOL_TYPE  route_window=$ROUTE_WINDOW_TOPIC  stop_point=$STOP_POINT_TOPIC"
echo "  velocity_shaper: igv_velocity_shaper (Phase 3 — single owner of obstacle-driven v(s))"
echo "  lane_width=$LANE_WIDTH  generate_drivable_area=$GENERATE_DRIVABLE_AREA"
echo "  use_occupancy_grid=$USE_OCCUPANCY_GRID (free-space occupancy stop/slow-down)"
echo "  enable_detection_grid=$ENABLE_DETECTION_GRID (fallback occupancy source)"
echo "  use_vehicle_agent=$USE_VEHICLE_AGENT  vehicle_agent_adapters='${VEHICLE_AGENT_ADAPTERS:-<derived from protocol>}'"
echo ""
echo "Map/Vehicle:"
echo "  MAP_FORMAT=$MAP_FORMAT"
echo "  ZPMC_TILE_DIRECTORY=$ZPMC_TILE_DIRECTORY"
echo "  MAP_PATH=$MAP_PATH  LANELET2_MAP_FILE=$LANELET2_MAP_FILE"
echo "  launch_pointcloud_map=$LAUNCH_POINTCLOUD_MAP  POINTCLOUD_MAP_FILE=$POINTCLOUD_MAP_FILE"
echo "  VEHICLE_MODEL=$VEHICLE_MODEL  VEHICLE_ID=$VEHICLE_ID"
echo ""
echo "Optional: launch_rviz=$LAUNCH_RVIZ  log_to_file=$LOG_TO_FILE"
echo "======================================================================"
echo "Starting in 3 seconds..."
sleep 3

if [ "$LOG_TO_FILE" = "true" ]; then
  > "$LOG_FILE"
fi

LAUNCH_CMD=(ros2 launch igv_launch igv_planning.launch.xml
  map_format:=$MAP_FORMAT
  zpmc_tile_directory:=$ZPMC_TILE_DIRECTORY
  map_path:=$MAP_PATH
  lanelet2_map_file:=$LANELET2_MAP_FILE
  pointcloud_map_file:=$POINTCLOUD_MAP_FILE
  launch_pointcloud_map:=$LAUNCH_POINTCLOUD_MAP
  vehicle_model:=$VEHICLE_MODEL
  vehicle_id:=$VEHICLE_ID
  is_simulation:=$IS_SIMULATION
  use_sim_time:=$USE_SIM_TIME
  launch_vehicle_interface:=$LAUNCH_VEHICLE_INTERFACE
  launch_planning:=$LAUNCH_PLANNING
  launch_system:=$LAUNCH_SYSTEM
  launch_rviz:=$LAUNCH_RVIZ
  fms_enabled:=$FMS_ENABLED
  fms_protocol_type:=$FMS_PROTOCOL_TYPE
  fms_topic:=$FMS_TOPIC
  use_json_input:=$USE_JSON_INPUT
  route_window_topic:=$ROUTE_WINDOW_TOPIC
  stop_point_topic:=$STOP_POINT_TOPIC
  use_vehicle_agent:=$USE_VEHICLE_AGENT
  vehicle_agent_adapters:=$VEHICLE_AGENT_ADAPTERS
  vehicle_agent_scenario_path:=$VEHICLE_AGENT_SCENARIO_PATH
  skip_motion_velocity_planner:=$SKIP_MOTION_VELOCITY_PLANNER
  enable_trajectory_relay:=$ENABLE_TRAJECTORY_RELAY
  enable_perception_relay:=$ENABLE_PERCEPTION_RELAY
  lane_width:=$LANE_WIDTH
  generate_drivable_area:=$GENERATE_DRIVABLE_AREA
  use_occupancy_grid:=$USE_OCCUPANCY_GRID \
  enable_detection_grid:=$ENABLE_DETECTION_GRID)

# Run ros2 launch in the foreground so Ctrl+C reaches its signal handler.
# Its launch service stops only the nodes started by this invocation.
if [ "$LOG_TO_FILE" = "true" ]; then
  exec "${LAUNCH_CMD[@]}" > >(tee -a "$LOG_FILE") 2>&1
else
  exec "${LAUNCH_CMD[@]}"
fi
