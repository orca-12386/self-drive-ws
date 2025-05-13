#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
from builtin_interfaces.msg import Time as TimeMsg

class OdomMapTransformPublisher(Node):
    """
    A ROS 2 node that publishes transforms to establish both the map frame
    and the relationship between map and odom frames.
    """
    
    def __init__(self):
        super().__init__('odom_map_tf_publisher')
        
        # Create a static transform broadcaster for persistent transforms
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        
        # Create a regular transform broadcaster for the dynamic updates
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Publish both transforms initially
        self.publish_world_to_map_transform()
        
        # Set up timer to publish the transform at a fixed rate (50Hz)
        # Having a continuous publication ensures the transform is always available
        self.timer = self.create_timer(0.02, self.publish_odom_to_map_transform)
        
        self.get_logger().info('Odom-Map transform publisher started')
    
    def publish_world_to_map_transform(self):
        """Publish a static transform from world to map (establishing the map frame)."""
        t = TransformStamped()
        
        # Use a zero timestamp for static transforms
        zero_time = TimeMsg()
        zero_time.sec = 0
        zero_time.nanosec = 0
        t.header.stamp = zero_time
        
        # For the map frame, we can anchor it to "world" or use another existing frame
        t.header.frame_id = 'world'  # This could be any well-established frame in your system
        t.child_frame_id = 'robot/odom'
        
        # Identity transform since map is at the origin of the world
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        
        self.static_tf_broadcaster.sendTransform(t)
    
    def publish_odom_to_map_transform(self):
        """Publish a transform that makes the odom and map frames identical."""
        t = TransformStamped()
        
        # Use current time for the dynamic transform
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'robot/base_link'  # Parent frame
        t.child_frame_id = 'robot/odom'  # Child frame
        
        # Identity transform
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = OdomMapTransformPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()