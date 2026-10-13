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

# Sprint-1 simulation entrypoint. Runs vehicle_agent with the scenario_loopback
# adapter against scenarios/basic_pickup.yaml (override via `scenario:=`).

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory("vehicle_agent")

    scenario_arg = DeclareLaunchArgument(
        "scenario",
        default_value=PathJoinSubstitution([pkg_share, "scenarios", "basic_pickup.yaml"]),
        description="Path to YAML scenario file consumed by scenario_loopback adapter",
    )

    publish_ns_arg = DeclareLaunchArgument(
        "publish_ns",
        default_value="/vehicle_agent",
        description="Topic namespace prefix for IR publishers",
    )

    node = Node(
        package="vehicle_agent",
        executable="vehicle_agent",
        name="vehicle_agent",
        output="screen",
        parameters=[{
            "adapters": ["scenario_loopback"],
            "scenario_path": LaunchConfiguration("scenario"),
            "publish_ns": LaunchConfiguration("publish_ns"),
            "event_bus_capacity": 1024,
        }],
    )

    return LaunchDescription([scenario_arg, publish_ns_arg, node])
