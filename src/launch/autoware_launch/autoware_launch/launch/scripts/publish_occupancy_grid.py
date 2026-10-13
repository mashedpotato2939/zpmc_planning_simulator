#!/usr/bin/env python3

"""
Occupancy Grid Publisher for Autoware Planning
Publishes minimal empty occupancy grid required by behavior path planner
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Pose
import numpy as np

class OccupancyGridPublisher(Node):
    def __init__(self):
        super().__init__('occupancy_grid_publisher')
        
        # Publisher for occupancy grid
        self.grid_publisher = self.create_publisher(
            OccupancyGrid,
            '/perception/occupancy_grid_map/map',
            10
        )
        
        # Timer to publish occupancy grid at 1Hz
        self.timer = self.create_timer(1.0, self.publish_occupancy_grid)
        
        self.get_logger().info("Occupancy Grid Publisher started")
        self.get_logger().info("Publishing to /perception/occupancy_grid_map/map")
        
    def publish_occupancy_grid(self):
        """Publish a minimal empty occupancy grid"""
        grid_msg = OccupancyGrid()
        
        # Header
        grid_msg.header.stamp = self.get_clock().now().to_msg()
        grid_msg.header.frame_id = 'map'
        
        # Map metadata
        grid_msg.info.resolution = 0.2  # 0.2 meters per cell
        grid_msg.info.width = 500       # 500 cells wide (100m)
        grid_msg.info.height = 500      # 500 cells tall (100m)
        
        # Origin pose (center the grid around our static position)
        grid_msg.info.origin.position.x = 5.1303   # Center around static position
        grid_msg.info.origin.position.y = -35.9575 # Center around static position  
        grid_msg.info.origin.position.z = 0.0
        grid_msg.info.origin.orientation.x = 0.0
        grid_msg.info.origin.orientation.y = 0.0
        grid_msg.info.origin.orientation.z = 0.0
        grid_msg.info.origin.orientation.w = 1.0
        
        # Create empty grid data (all cells are free space = 0)
        # -1 = unknown, 0 = free, 100 = occupied
        total_cells = grid_msg.info.width * grid_msg.info.height
        grid_msg.data = [0] * total_cells  # All free space
        
        # Publish the grid
        self.grid_publisher.publish(grid_msg)

def main(args=None):
    rclpy.init(args=args)
    
    try:
        occupancy_publisher = OccupancyGridPublisher()
        rclpy.spin(occupancy_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        if 'occupancy_publisher' in locals():
            occupancy_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()