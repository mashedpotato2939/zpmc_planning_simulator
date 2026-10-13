#!/usr/bin/env python3

"""
Enhanced Simple Object Predictor for Simulation with Debug Logging
Converts DetectedObjects to PredictedObjects for use with dummy perception in simulation.

Debug Logging:
  - To enable detailed debug logs: Set parameter 'enable_debug_log' to True (default)
  - To disable debug logs: Set parameter 'enable_debug_log' to False
  
  Example: ros2 run <package> simple_object_predictor_debug.py --ros-args -p enable_debug_log:=false
"""

import copy
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rclpy.duration import Duration as RclpyDuration
from autoware_perception_msgs.msg import (
    DetectedObjects,
    PredictedObjects,
    PredictedObject,
    PredictedObjectKinematics,
    PredictedPath
)
from geometry_msgs.msg import AccelWithCovariance, Accel, Vector3, Pose, PoseStamped, TwistStamped
from unique_identifier_msgs.msg import UUID
from builtin_interfaces.msg import Duration as MsgDuration
from tf2_ros import Buffer, TransformListener, LookupException, TransformException
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_pose
from tf_transformations import quaternion_matrix
import uuid
import numpy as np


class SimpleObjectPredictor(Node):
    def __init__(self):
        super().__init__('simple_object_predictor')

        # QoS settings to match the perception pipeline
        # RELIABLE + VOLATILE with depth 1 to ensure latest data
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Subscribe to detected objects from dummy perception
        self.subscription = self.create_subscription(
            DetectedObjects,
            '/perception/object_recognition/detection/objects',
            self.detected_objects_callback,
            qos_profile
        )

        # Publish predicted objects for planning stack
        self.publisher = self.create_publisher(
            PredictedObjects,
            '/perception/object_recognition/objects',
            qos_profile
        )

        # TF2 setup for coordinate transformation
        self.target_frame = 'map'  # Planning expects objects in map frame
        self.tf_buffer = Buffer(cache_time=RclpyDuration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

        # Debug logging parameter (can be set via ROS parameter)
        self.declare_parameter('enable_debug_log', True)
        self.enable_debug_log = self.get_parameter('enable_debug_log').value

        self.get_logger().info('=' * 60)
        self.get_logger().info('Simple Object Predictor started')
        self.get_logger().info('=' * 60)
        self.get_logger().info(f'Subscribing to: /perception/object_recognition/detection/objects')
        self.get_logger().info(f'Publishing to: /perception/object_recognition/objects')
        self.get_logger().info(f'Target frame: {self.target_frame}')
        self.get_logger().info(f'QoS: RELIABLE, VOLATILE, depth=1')
        self.get_logger().info(f'Debug logging: {"ENABLED" if self.enable_debug_log else "DISABLED"}')
        self.get_logger().info('=' * 60)

        self.object_count = 0
        self.publish_count = 0

    def detected_objects_callback(self, detected_msg):
        """Convert DetectedObjects to PredictedObjects"""

        self.object_count += 1
        num_objects = len(detected_msg.objects)

        if self.enable_debug_log:
            self.get_logger().info('=' * 60)
            self.get_logger().info(f'[{self.object_count}] Received DetectedObjects message')
            self.get_logger().info(f'  Frame: {detected_msg.header.frame_id}')
            self.get_logger().info(f'  Number of objects: {num_objects}')

        if num_objects == 0:
            if self.enable_debug_log:
                self.get_logger().warn('  [WARNING] No objects in message - will publish empty PredictedObjects')

        # Create predicted objects message
        predicted_msg = PredictedObjects()
        predicted_msg.header = copy.deepcopy(detected_msg.header)
        predicted_msg.header.frame_id = self.target_frame  # Publish in map frame

        # Convert each detected object to a predicted object
        for idx, detected_obj in enumerate(detected_msg.objects):
            if self.enable_debug_log:
                self.get_logger().info(f'  Object {idx + 1}/{num_objects}:')

                # Log classification
                if len(detected_obj.classification) > 0:
                    label = detected_obj.classification[0].label
                    prob = detected_obj.classification[0].probability
                    label_str = self._get_label_string(label)
                    self.get_logger().info(f'    Classification: {label_str} (label={label}, prob={prob:.2f})')
                else:
                    self.get_logger().warn(f'    [WARNING] No classification!')

                # Log position
                pos = detected_obj.kinematics.pose_with_covariance.pose.position
                self.get_logger().info(f'    Position: x={pos.x:.2f}, y={pos.y:.2f}, z={pos.z:.2f}')

                # Log shape
                if detected_obj.shape.dimensions.x > 0:
                    self.get_logger().info(f'    Shape: L={detected_obj.shape.dimensions.x:.2f}m, '
                                         f'W={detected_obj.shape.dimensions.y:.2f}m, '
                                         f'H={detected_obj.shape.dimensions.z:.2f}m')

            # Transform object to target frame (map)
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.target_frame,
                    detected_msg.header.frame_id,
                    rclpy.time.Time(),  # Use latest available transform
                    timeout=RclpyDuration(seconds=0.2),
                )
            except (LookupException, TransformException) as exc:
                self.get_logger().error(
                    f'    [ERROR] Failed to transform from {detected_msg.header.frame_id} '
                    f'to {self.target_frame}: {exc}'
                )
                continue  # Skip this object

            # Transform pose using tf2
            # do_transform_pose in Python expects Pose and TransformStamped, returns Pose
            pose_in_source = detected_obj.kinematics.pose_with_covariance.pose
            pose_in_target = do_transform_pose(pose_in_source, transform)

            if self.enable_debug_log:
                pos = detected_obj.kinematics.pose_with_covariance.pose.position
                self.get_logger().info(
                    f'    Transformed: ({pos.x:.2f},{pos.y:.2f}) in {detected_msg.header.frame_id} '
                    f'-> ({pose_in_target.position.x:.2f},{pose_in_target.position.y:.2f}) in {self.target_frame}'
                )

            # Transform twist (rotate velocity vectors)
            twist_stamped = TwistStamped()
            twist_stamped.header = detected_msg.header
            twist_stamped.twist = detected_obj.kinematics.twist_with_covariance.twist

            rotation_matrix = quaternion_matrix([
                transform.transform.rotation.x,
                transform.transform.rotation.y,
                transform.transform.rotation.z,
                transform.transform.rotation.w,
            ])
            
            linear_vel = np.array([
                twist_stamped.twist.linear.x,
                twist_stamped.twist.linear.y,
                twist_stamped.twist.linear.z,
            ])
            rotated_linear = rotation_matrix[:3, :3].dot(linear_vel)

            angular_vel = np.array([
                twist_stamped.twist.angular.x,
                twist_stamped.twist.angular.y,
                twist_stamped.twist.angular.z,
            ])
            rotated_angular = rotation_matrix[:3, :3].dot(angular_vel)

            # Create predicted object
            predicted_obj = PredictedObject()

            # Generate unique ID for tracking
            object_id = UUID()
            id_bytes = uuid.uuid4().bytes
            object_id.uuid = list(id_bytes)
            predicted_obj.object_id = object_id

            # Convert kinematics from DetectedObjectKinematics to PredictedObjectKinematics
            predicted_kinematics = PredictedObjectKinematics()
            
            # Use transformed pose
            predicted_kinematics.initial_pose_with_covariance = copy.deepcopy(
                detected_obj.kinematics.pose_with_covariance
            )
            predicted_kinematics.initial_pose_with_covariance.pose = pose_in_target
            
            # Use transformed twist
            predicted_kinematics.initial_twist_with_covariance = copy.deepcopy(
                detected_obj.kinematics.twist_with_covariance
            )
            predicted_kinematics.initial_twist_with_covariance.twist.linear.x = rotated_linear[0]
            predicted_kinematics.initial_twist_with_covariance.twist.linear.y = rotated_linear[1]
            predicted_kinematics.initial_twist_with_covariance.twist.linear.z = rotated_linear[2]
            predicted_kinematics.initial_twist_with_covariance.twist.angular.x = rotated_angular[0]
            predicted_kinematics.initial_twist_with_covariance.twist.angular.y = rotated_angular[1]
            predicted_kinematics.initial_twist_with_covariance.twist.angular.z = rotated_angular[2]

            # Set zero acceleration with covariance
            accel_with_cov = AccelWithCovariance()
            accel_with_cov.accel = Accel()
            accel_with_cov.accel.linear = Vector3(x=0.0, y=0.0, z=0.0)
            accel_with_cov.accel.angular = Vector3(x=0.0, y=0.0, z=0.0)
            accel_with_cov.covariance = [0.0] * 36
            predicted_kinematics.initial_acceleration_with_covariance = accel_with_cov

            # Create a predicted path for static obstacles
            # The planner needs at least one path to properly process the obstacle
            predicted_path = PredictedPath()
            predicted_path.confidence = 1.0  # 100% confidence

            # Set the time step between consecutive poses (0.5 seconds)
            predicted_path.time_step = MsgDuration()
            predicted_path.time_step.sec = 0
            predicted_path.time_step.nanosec = int(0.5 * 1e9)  # 0.5s interval

            # For a static obstacle, create a short path with the object staying in place
            # Use 5 seconds horizon with 0.5s intervals = 10 points
            # Use the TRANSFORMED pose in map frame
            for i in range(10):
                pose = Pose()
                pose.position.x = pose_in_target.position.x
                pose.position.y = pose_in_target.position.y
                pose.position.z = pose_in_target.position.z
                pose.orientation = pose_in_target.orientation
                predicted_path.path.append(pose)

            predicted_kinematics.predicted_paths = [predicted_path]
            
            if self.enable_debug_log:
                self.get_logger().info(f'    Predicted path: {len(predicted_path.path)} poses, confidence=1.0')

            predicted_obj.kinematics = predicted_kinematics

            # Copy classification
            predicted_obj.classification = detected_obj.classification

            # Copy shape
            predicted_obj.shape = detected_obj.shape

            # Set existence probability - critical for planning to consider the object!
            # Use high probability for detected objects (0.0-1.0, where 1.0 = certain)
            predicted_obj.existence_probability = 1.0
            
            if self.enable_debug_log:
                self.get_logger().info(f'    Existence probability: {predicted_obj.existence_probability}')

            predicted_msg.objects.append(predicted_obj)

        # Publish the predicted objects
        self.publish_count += 1
        self.publisher.publish(predicted_msg)

        if self.enable_debug_log:
            self.get_logger().info(f'[{self.publish_count}] Published {len(predicted_msg.objects)} PredictedObjects')
            self.get_logger().info('=' * 60)

    def _get_label_string(self, label):
        """Convert numeric label to string for logging"""
        labels = {
            0: 'UNKNOWN',
            1: 'CAR',
            2: 'TRUCK',
            3: 'BUS',
            4: 'TRAILER',
            5: 'MOTORCYCLE',
            6: 'BICYCLE',
            7: 'PEDESTRIAN'
        }
        return labels.get(label, f'LABEL_{label}')


def main(args=None):
    rclpy.init(args=args)
    node = SimpleObjectPredictor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
