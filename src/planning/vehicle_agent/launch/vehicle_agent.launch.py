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

# Production entrypoint for vehicle_agent. Selects one or more cloud-protocol
# adapters and starts the composable node. The vehicle_agent_sim variant in
# this same directory is a thin wrapper that hard-codes adapters=[scenario_loopback].

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _build_node(context, *args, **kwargs):
    # Convert the CSV `adapters` arg into a Python list — the underlying ROS
    # parameter is declared as std::vector<std::string>, and rclcpp will not
    # coerce a CSV string into a list.
    adapters_csv = LaunchConfiguration("adapters").perform(context)
    adapter_list = [s.strip() for s in adapters_csv.split(",") if s.strip()]
    return [Node(
        package="vehicle_agent",
        executable="vehicle_agent",
        name="vehicle_agent",
        output="screen",
        parameters=[{
            "adapters": adapter_list,
            "vehicle_id": LaunchConfiguration("vehicle_id").perform(context),
            "publish_ns": LaunchConfiguration("publish_ns").perform(context),
            "odometry_topic": LaunchConfiguration("odometry_topic").perform(context),
            "scenario_path": LaunchConfiguration("scenario_path").perform(context),
            "queue_db_path": LaunchConfiguration("queue_db_path").perform(context),
            "telemetry_rate_hz": float(
                LaunchConfiguration("telemetry_rate_hz").perform(context)),
            "ahtfms_ns.vehicle_id": LaunchConfiguration("vehicle_id").perform(context),
            "ahtfms_ns.broker_host":
                LaunchConfiguration("ahtfms_ns.broker_host").perform(context),
            "ahtfms_ns.broker_port": int(
                LaunchConfiguration("ahtfms_ns.broker_port").perform(context)),
            "internal_zpmc.vehicle_id":
                LaunchConfiguration("vehicle_id").perform(context),
            "internal_zpmc.broker_host":
                LaunchConfiguration("internal_zpmc.broker_host").perform(context),
            "internal_zpmc.broker_port": int(
                LaunchConfiguration("internal_zpmc.broker_port").perform(context)),
            "internal_zpmc.hmac_key":
                LaunchConfiguration("internal_zpmc.hmac_key").perform(context),
        }],
    )]


def generate_launch_description():
    # Adapter selection is the main knob — comma-separated names map 1:1 onto
    # vehicle_agent::AdapterMultiplexer's runtime registry.
    adapters_arg = DeclareLaunchArgument(
        "adapters",
        default_value="ahtfms_ns",
        description=(
            "Comma-separated list of adapters to start. Known names: "
            "ahtfms_ns | internal_zpmc | scenario_loopback"
        ),
    )

    vehicle_id_arg = DeclareLaunchArgument(
        "vehicle_id", default_value="IGV2001",
        description="Vehicle identifier shared across adapters (sender_sys / topic suffix)",
    )
    publish_ns_arg = DeclareLaunchArgument(
        "publish_ns", default_value="/vehicle_agent",
        description="ROS namespace for VA's IR publishers",
    )
    odometry_topic_arg = DeclareLaunchArgument(
        "odometry_topic", default_value="/localization/kinematic_state",
        description="ROS topic VA subscribes to for ir::Telemetry uplink",
    )
    scenario_path_arg = DeclareLaunchArgument(
        "scenario_path", default_value="",
        description="Path to scenario YAML — required only when scenario_loopback is in `adapters`",
    )
    queue_db_path_arg = DeclareLaunchArgument(
        "queue_db_path", default_value="~/.ros/vehicle_agent/queue.db",
        description="SQLite path for the durable offline queue",
    )
    telemetry_rate_hz_arg = DeclareLaunchArgument(
        "telemetry_rate_hz", default_value="10.0",
        description="Max uplink telemetry rate (Hz)",
    )

    # AHTFMS-NS adapter parameters (forwarded only when that adapter is active,
    # but declared up front so launches can override without rclcpp warnings).
    ahtfms_host_arg = DeclareLaunchArgument(
        "ahtfms_ns.broker_host", default_value="127.0.0.1",
        description="AHTFMS-NS CM gateway host",
    )
    ahtfms_port_arg = DeclareLaunchArgument(
        "ahtfms_ns.broker_port", default_value="8801",
        description="AHTFMS-NS CM gateway TCP port",
    )

    # Internal ZPMC adapter parameters.
    zpmc_host_arg = DeclareLaunchArgument(
        "internal_zpmc.broker_host", default_value="127.0.0.1",
        description="Internal ZPMC MQTT broker host",
    )
    zpmc_port_arg = DeclareLaunchArgument(
        "internal_zpmc.broker_port", default_value="8883",
        description="Internal ZPMC MQTT broker TLS port",
    )
    zpmc_key_arg = DeclareLaunchArgument(
        "internal_zpmc.hmac_key", default_value="",
        description="HMAC-SHA256 shared secret (provision via Ansible vault)",
    )

    def csv_to_list(text):
        return [s.strip() for s in text.split(",") if s.strip()]

    return LaunchDescription([
        adapters_arg, vehicle_id_arg, publish_ns_arg, odometry_topic_arg,
        scenario_path_arg, queue_db_path_arg, telemetry_rate_hz_arg,
        ahtfms_host_arg, ahtfms_port_arg,
        zpmc_host_arg, zpmc_port_arg, zpmc_key_arg,
        OpaqueFunction(function=_build_node),
    ])
