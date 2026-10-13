# Perception Relay

## Overview

The `perception_relay` package converts perception messages from `autoware_auto_perception_msgs` (Galactic) to `autoware_perception_msgs` (Humble) format. This is needed for real vehicle operation where perception runs on a separate system using Galactic ROS 2.

## Architecture

```
Galactic Perception System              Humble Planning System
┌─────────────────────────┐             ┌─────────────────────────┐
│ map_based_prediction    │             │ planning modules        │
│ object_merger           │─────────────│ (behavior planner,      │
│ multi_object_tracker    │  Relay      │  motion planner, etc.)  │
└─────────────────────────┘             └─────────────────────────┘
```

## Message Conversion

The node converts three perception message streams:

| Message Type | Input (Galactic) | Output (Humble) |
|--------------|------------------|-----------------|
| Predicted Objects | `autoware_auto_perception_msgs/PredictedObjects` | `autoware_perception_msgs/PredictedObjects` |
| Detected Objects | `autoware_auto_perception_msgs/DetectedObjects` | `autoware_perception_msgs/DetectedObjects` |
| Tracked Objects | `autoware_auto_perception_msgs/TrackedObjects` | `autoware_perception_msgs/TrackedObjects` |

Field mappings are 1:1 (no transformation needed), only type conversion between message versions.

## Topics

### Subscriptions (Input - Galactic format)

| Topic | Type | Description |
|-------|------|-------------|
| `~/input/predicted_objects` | `autoware_auto_perception_msgs/PredictedObjects` | Predicted objects from map-based prediction |
| `~/input/detected_objects` | `autoware_auto_perception_msgs/DetectedObjects` | Detected objects from object merger |
| `~/input/tracked_objects` | `autoware_auto_perception_msgs/TrackedObjects` | Tracked objects from multi-object tracker |

### Publishers (Output - Humble format)

| Topic | Type | Description |
|-------|------|-------------|
| `~/output/predicted_objects` | `autoware_perception_msgs/PredictedObjects` | Converted predicted objects |
| `~/output/detected_objects` | `autoware_perception_msgs/DetectedObjects` | Converted detected objects |
| `~/output/tracked_objects` | `autoware_perception_msgs/TrackedObjects` | Converted tracked objects |

## Usage

### Launch with default topic mappings

```bash
ros2 launch perception_relay perception_relay.launch.xml
```

### Launch with FMS integration

```bash
ros2 launch autoware_launch complete_autoware_system_fms.launch.xml \
  enable_perception_relay:=true \
  launch_simulator:=false
```

### Custom topic mappings

```bash
ros2 launch perception_relay perception_relay.launch.xml \
  input_predicted_objects_topic:=/custom/predicted_objects \
  output_predicted_objects_topic:=/planning/predicted_objects
```

## Default Topic Mappings

The launch file provides default topic remappings for Autoware integration:

| Input Topic (Galactic) | Output Topic (Humble) |
|------------------------|----------------------|
| `/perception/map_based_prediction/objects` | `/perception/object_recognition/objects` |
| `/perception/object_merger/objects` | `/perception/object_recognition/detection/objects` |
| `/perception/multi_object_tracker/objects` | `/perception/object_recognition/tracking/objects` |

## Parameters

See `config/perception_relay.param.yaml` for configurable parameters.

## Build

```bash
colcon build --packages-select perception_relay
```

## Dependencies

This package uses message definitions from:
- `autoware_auto_perception_msgs` (from `src/core/external/`)
- `autoware_perception_msgs` (from `src/core/autoware_msgs/`)
