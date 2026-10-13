#!/bin/bash

# Launch IGV Test Components Terminal — companion to launch_with_fms_real_vehicle.sh.
#
# This terminal stands in for the "other teams" during integration tests of
# the planning terminal (igv_planning.launch.xml):
#   - Simulator: provides /localization/kinematic_state and a dummy vehicle
#   - Dummy perception
#   - simple_object_predictor (stand-in for the external perception team's prediction)
#   - Full control stack: gate / OMT / AEB / shift_decider / validator + motion_control
#     (stand-in for the external control team)
#   - Shared pointcloud container (the optional .pcd loader remains disabled)
#
# Backed by igv_test_components.launch.xml. Every component in that file runs
# unconditionally — no per-component gates from this script.
#
# Run order:
#   Terminal 1: ./launch_fms_test_components.sh       (this script)
#   Terminal 2: ./launch_with_fms_real_vehicle.sh
#   Terminal 3: RViz FmsBridgePanel (or fms_helper.py) to send routes

echo "======================================================================"
echo "  Launching IGV Test Components Terminal"
echo "======================================================================"

# Source the workspace
source /opt/ros/galactic/setup.bash
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
source "$SCRIPT_DIR/install/setup.bash"

# ===================== MAP AND VEHICLE (must match real vehicle script) =====================

MAP_PATH=${MAP_PATH:-$SCRIPT_DIR/src/map/xiamen_tiles_v1}
POINTCLOUD_MAP_FILE=${POINTCLOUD_MAP_FILE:-global.pcd}

# VEHICLE_MODEL=${VEHICLE_MODEL:-scale_igv_vehicle}
VEHICLE_MODEL=igv_vehicle
VEHICLE_ID=${VEHICLE_ID:-v001}

IS_SIMULATION=${IS_SIMULATION:-true}
USE_SIM_TIME=${USE_SIM_TIME:-true}
LAUNCH_SIMULATION_CLOCK=${LAUNCH_SIMULATION_CLOCK:-false}
LOCALIZATION_SIM_MODE=${LOCALIZATION_SIM_MODE:-api}

# External perception is the default integration source. Set these to true only for standalone planning tests.
LAUNCH_DUMMY_PERCEPTION=${LAUNCH_DUMMY_PERCEPTION:-false}
USE_PERCEPTION_LAUNCH_MODULES=${USE_PERCEPTION_LAUNCH_MODULES:-false}
LAUNCH_SIMPLE_OBJECT_PREDICTOR=${LAUNCH_SIMPLE_OBJECT_PREDICTOR:-false}

# Initial simulator pose — should correspond to a valid location on the tile map
INITIAL_POSE=${INITIAL_POSE:-"[80.0, 20.0, 0.0, 0.0, 0.0, 0.0, 1.0]"}

# ===================== OPTIONAL =====================

LOG_TO_FILE=${LOG_TO_FILE:-false}
LOG_FILE=${LOG_FILE:-/tmp/fms_test_components_debug.txt}

# ===================== DISPLAY =====================

echo "Test components (all run unconditionally):"
echo "  pointcloud_container (legacy pointcloud map loading is disabled)"
echo "  simulator and dummy_vehicle; external perception is enabled by default"
echo "  control: gate/OMT/AEB/shift_decider/validator + motion_control"
echo ""
echo "Map/Vehicle (must match real-vehicle script):"
echo "  MAP_PATH=$MAP_PATH"
echo "  POINTCLOUD_MAP_FILE=$POINTCLOUD_MAP_FILE"
echo "  VEHICLE_MODEL=$VEHICLE_MODEL  VEHICLE_ID=$VEHICLE_ID"
echo "  IS_SIMULATION=$IS_SIMULATION  USE_SIM_TIME=$USE_SIM_TIME  LOCALIZATION_SIM_MODE=$LOCALIZATION_SIM_MODE"
echo "  LAUNCH_SIMULATION_CLOCK=$LAUNCH_SIMULATION_CLOCK"
echo "  LAUNCH_DUMMY_PERCEPTION=$LAUNCH_DUMMY_PERCEPTION"
echo "  USE_PERCEPTION_LAUNCH_MODULES=$USE_PERCEPTION_LAUNCH_MODULES"
echo "  LAUNCH_SIMPLE_OBJECT_PREDICTOR=$LAUNCH_SIMPLE_OBJECT_PREDICTOR"
echo "  INITIAL_POSE=$INITIAL_POSE"
echo "======================================================================"
echo "Starting in 3 seconds..."
sleep 3

if [ "$LOG_TO_FILE" = "true" ]; then
  > "$LOG_FILE"
fi

LAUNCH_CMD=(ros2 launch autoware_launch igv_test_components.launch.xml
  map_path:=$MAP_PATH
  pointcloud_map_file:=$POINTCLOUD_MAP_FILE
  vehicle_model:=$VEHICLE_MODEL
  vehicle_id:=$VEHICLE_ID
  is_simulation:=$IS_SIMULATION
  use_sim_time:=$USE_SIM_TIME
  launch_simulation_clock:=$LAUNCH_SIMULATION_CLOCK
  localization_sim_mode:=$LOCALIZATION_SIM_MODE
  launch_dummy_perception:=$LAUNCH_DUMMY_PERCEPTION
  use_perception_launch_modules:=$USE_PERCEPTION_LAUNCH_MODULES
  launch_simple_object_predictor:=$LAUNCH_SIMPLE_OBJECT_PREDICTOR
  user_defined_initial_pose/enable:=true
  "user_defined_initial_pose/pose:=$INITIAL_POSE")

# Run ros2 launch in the foreground so Ctrl+C reaches its signal handler.
# Its launch service stops only the nodes started by this invocation.
if [ "$LOG_TO_FILE" = "true" ]; then
  exec "${LAUNCH_CMD[@]}" > >(tee -a "$LOG_FILE") 2>&1
else
  exec "${LAUNCH_CMD[@]}"
fi
