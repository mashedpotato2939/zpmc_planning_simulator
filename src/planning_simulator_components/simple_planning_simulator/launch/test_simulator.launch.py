#!/usr/bin/env python3

import launch
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import launch_ros.parameter_descriptions
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    # Get package share directory
    pkg_share = FindPackageShare("simple_planning_simulator").perform(context)
    
    # Create the simple planning simulator node
    simple_planning_simulator_node = Node(
        package="simple_planning_simulator",
        executable="simple_planning_simulator_exe",
        name="simple_planning_simulator",
        namespace="simulation",
        output="screen",
        parameters=[
            # Vehicle model parameters
            {
                "vehicle_model_type": "IDEAL_STEER_VEL",
                "initialize_source": "ORIGIN",
                "initial_engage_state": True,
                "add_measurement_noise": False,
                "simulated_frame_id": "base_link",
                "origin_frame_id": "odom",  # Changed from "map" to "odom"
                "timer_sampling_time_ms": 25,
                "vel_lim": 30.0,
                "vel_rate_lim": 30.0,
                "steer_lim": 0.6,
                "steer_rate_lim": 6.28,
            },
            # Vehicle information parameters (required)
            {
                "wheel_radius": 0.39,
                "wheel_width": 0.42,
                "wheel_base": 2.74,
                "wheel_tread": 1.63,
                "front_overhang": 1.0,
                "rear_overhang": 1.03,
                "left_overhang": 0.1,
                "right_overhang": 0.1,
                "vehicle_height": 2.5,
                "max_steer_angle": 0.70,
            },
            # Vehicle characteristics parameters
            {
                "cg_to_front_m": 1.228,
                "cg_to_rear_m": 1.5618,
                "front_corner_stiffness": 17000.0,
                "rear_corner_stiffness": 20000.0,
                "mass_kg": 1460.0,
                "yaw_inertia_kgm2": 2170.0,
                "width_m": 2.0,
            }
        ],
        remappings=[
            ("input/ackermann_control_command", "/control/command/control_cmd"),
            ("input/gear_command", "/control/command/gear_cmd"),
            ("input/initialpose", "/initialpose3d"),
            ("output/odometry", "/localization/kinematic_state"),
            ("output/pose", "/localization/pose_estimator/pose_with_covariance"),
            ("output/twist", "/vehicle/status/velocity_status"),
            ("output/steering", "/vehicle/status/steering_status"),
        ],
    )

    # Add static transform publisher for map->odom frame
    static_transform_publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
        output="screen",
    )

    return [static_transform_publisher, simple_planning_simulator_node]


def generate_launch_description():
    return launch.LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])
