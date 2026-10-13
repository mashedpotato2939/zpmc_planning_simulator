# Copyright 2026 ZPMC
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Launches the va_route_bridge node that translates IR-aligned topics
# (/vehicle_agent/mission, /vehicle_agent/command) into the legacy FMS
# topic shapes (/fms/route_window, /fms/stop_point) that fms_path_converter
# consumes.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    vehicle_id_arg = DeclareLaunchArgument(
        "vehicle_id", default_value="IGV2001",
        description="vehicle_id stamped onto the outgoing FMSRouteWindow / FMSStopPoint",
    )
    mission_topic_arg = DeclareLaunchArgument(
        "mission_topic", default_value="/vehicle_agent/mission",
        description="Source IrMission topic published by vehicle_agent",
    )
    command_topic_arg = DeclareLaunchArgument(
        "command_topic", default_value="/vehicle_agent/command",
        description="Source IrCommand topic published by vehicle_agent",
    )
    odom_topic_arg = DeclareLaunchArgument(
        "odometry_topic", default_value="/localization/kinematic_state",
        description="Ego odometry — latched so PAUSE commands carry an x/y",
    )
    route_window_topic_arg = DeclareLaunchArgument(
        "route_window_topic", default_value="/fms/route_window",
        description="Output FMSRouteWindow topic (consumed by fms_path_converter)",
    )
    stop_point_topic_arg = DeclareLaunchArgument(
        "stop_point_topic", default_value="/fms/stop_point",
        description="Output FMSStopPoint topic (consumed by fms_path_converter)",
    )
    window_ahead_m_arg = DeclareLaunchArgument(
        "window_ahead_m", default_value="100.0",
        description="Meters ahead of ego included in each published FMSRouteWindow",
    )
    window_lookback_m_arg = DeclareLaunchArgument(
        "window_lookback_m", default_value="10.0",
        description="Meters behind ego included in each published FMSRouteWindow",
    )
    window_rate_hz_arg = DeclareLaunchArgument(
        "window_rate_hz", default_value="1.0",
        description="Republish rate for the ego-centric route window — must stay "
                    "above fms_path_converter's stale_threshold (~5s) to keep "
                    "the cached path alive when the FMS is quiet",
    )

    node = Node(
        package="vehicle_agent",
        executable="va_route_bridge",
        name="va_route_bridge",
        output="screen",
        parameters=[{
            "vehicle_id": LaunchConfiguration("vehicle_id"),
            "mission_topic": LaunchConfiguration("mission_topic"),
            "command_topic": LaunchConfiguration("command_topic"),
            "odometry_topic": LaunchConfiguration("odometry_topic"),
            "route_window_topic": LaunchConfiguration("route_window_topic"),
            "stop_point_topic": LaunchConfiguration("stop_point_topic"),
            "window_ahead_m": LaunchConfiguration("window_ahead_m"),
            "window_lookback_m": LaunchConfiguration("window_lookback_m"),
            "window_rate_hz": LaunchConfiguration("window_rate_hz"),
        }],
    )

    return LaunchDescription([
        vehicle_id_arg, mission_topic_arg, command_topic_arg, odom_topic_arg,
        route_window_topic_arg, stop_point_topic_arg,
        window_ahead_m_arg, window_lookback_m_arg, window_rate_hz_arg,
        node,
    ])
