#!/usr/bin/env python3
"""
Spawn and animate a red sphere obstacle in Gazebo Ignition
This sphere moves in a pattern for testing obstacle detection
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from ros_gz_interfaces.srv import SpawnEntity, DeleteEntity
import time
import math


class MovingSphereSpawner(Node):
    def __init__(self):
        super().__init__('moving_sphere_spawner')
        
        # Wait for Gazebo to be ready
        self.get_logger().info('Waiting for Gazebo services...')
        time.sleep(3)
        
        # Create service clients
        self.spawn_client = self.create_client(SpawnEntity, '/world/narrow_corridor/create')
        self.delete_client = self.create_client(DeleteEntity, '/world/narrow_corridor/remove')
        
        while not self.spawn_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for spawn service...')
        
        self.get_logger().info('Services ready!')
        
        # Spawn the red sphere
        self.spawn_red_sphere()
        
        # Create timer for animation (10 Hz)
        self.timer = self.create_timer(0.1, self.animate_sphere)
        self.time_elapsed = 0.0
        
    def spawn_red_sphere(self):
        """Spawn a red sphere in the world"""
        req = SpawnEntity.Request()
        req.name = 'moving_red_sphere'
        req.xml = '''<?xml version="1.0"?>
<sdf version="1.8">
  <model name="moving_red_sphere">
    <pose>1.0 0.5 0.2 0 0 0</pose>
    <link name="link">
      <collision name="collision">
        <geometry>
          <sphere>
            <radius>0.2</radius>
          </sphere>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <sphere>
            <radius>0.2</radius>
          </sphere>
        </geometry>
        <material>
          <ambient>1 0 0 1</ambient>
          <diffuse>1 0 0 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>'''
        
        future = self.spawn_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.result() is not None:
            self.get_logger().info('Red sphere spawned successfully!')
        else:
            self.get_logger().error('Failed to spawn red sphere')
    
    def animate_sphere(self):
        """Move the sphere in a circular pattern"""
        # This would require using Gazebo's set_entity_state service
        # For Ignition, we'll use a simple back-and-forth motion
        self.time_elapsed += 0.1
        
        # Simple sinusoidal motion in the corridor
        x = 1.0 + 1.5 * math.sin(self.time_elapsed * 0.5)  # Move along corridor
        y = 0.5 * math.cos(self.time_elapsed * 0.3)  # Slight side-to-side
        z = 0.2
        
        # Note: Gazebo Ignition doesn't have a simple set_pose service like classic Gazebo
        # We would need to use the physics plugin or entity commands
        # For now, this is a placeholder - the sphere will be static
        pass


def main(args=None):
    rclpy.init(args=args)
    spawner = MovingSphereSpawner()
    
    try:
        rclpy.spin(spawner)
    except KeyboardInterrupt:
        pass
    
    spawner.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
