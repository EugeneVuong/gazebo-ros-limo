#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# ultralytics_ros
# Copyright (C) 2023-2024

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, TransformStamped
import tf2_ros
from tf2_geometry_msgs import do_transform_point
from rclpy.time import Time
from rclpy.duration import Duration
import math

class ObjectTransformNode(Node):
    def __init__(self):
        super().__init__("object_transform_node")
        
        # Declare parameters
        self.declare_parameter("input_topic", "/object_position")
        self.declare_parameter("output_topic", "/object_position_world")
        self.declare_parameter("use_fixed_frame", True)
        self.declare_parameter("target_frame", "world")
        self.declare_parameter("tf_timeout", 1.0)  # seconds
        self.declare_parameter("retry_rate", 5.0)  # Hz
        self.declare_parameter("tf_broadcast_rate", 10.0)  # Hz - how often to broadcast the static transform
        self.declare_parameter("use_tf_tree", False)  # Whether to use TF tree or fixed transform
        
        # Get parameters
        self.input_topic = self.get_parameter("input_topic").get_parameter_value().string_value
        self.output_topic = self.get_parameter("output_topic").get_parameter_value().string_value
        self.use_fixed_frame = self.get_parameter("use_fixed_frame").get_parameter_value().bool_value
        self.target_frame = self.get_parameter("target_frame").get_parameter_value().string_value
        self.tf_timeout = self.get_parameter("tf_timeout").get_parameter_value().double_value
        self.retry_rate = self.get_parameter("retry_rate").get_parameter_value().double_value
        self.tf_broadcast_rate = self.get_parameter("tf_broadcast_rate").get_parameter_value().double_value
        self.use_tf_tree = self.get_parameter("use_tf_tree").get_parameter_value().bool_value
        
        # Initialize TF buffer, listener, and broadcaster
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Use TransformBroadcaster (not static) for more reliable publishing
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # Create subscriber and publisher
        self.subscription = self.create_subscription(
            PointStamped,
            self.input_topic,
            self.transform_callback,
            10)
        self.publisher = self.create_publisher(PointStamped, self.output_topic, 10)
        
        # Create static transform for broadcasting
        self.static_transform = self.create_static_transform()
        
        # Create a timer to periodically broadcast the transform
        if self.use_fixed_frame:
            self.transform_broadcast_timer = self.create_timer(
                1.0 / self.tf_broadcast_rate, 
                self.broadcast_transform_callback
            )
            self.get_logger().info(f"Broadcasting transform from {self.target_frame} to depth_camera_link at {self.tf_broadcast_rate} Hz")
        
        # Create a timer to check for available transforms if using TF tree
        if self.use_tf_tree:
            self.transform_check_timer = self.create_timer(
                1.0 / self.retry_rate, 
                self.check_transform_availability
            )
        
        # Flag to track transform availability
        self.transform_available = False
        self.warned_frames = set()
        
        self.get_logger().info(f'Object transform node initialized, transforming positions to {self.target_frame} frame at origin (0,0,0)')
    
    def create_static_transform(self):
        """Create a static transform from world frame to depth_camera_link at origin"""
        static_transform = TransformStamped()
        
        # Set header
        static_transform.header.frame_id = self.target_frame
        static_transform.child_frame_id = "depth_camera_link"
        
        # Set translation (all zeros for origin)
        static_transform.transform.translation.x = 0.0
        static_transform.transform.translation.y = 0.0
        static_transform.transform.translation.z = 0.0
        
        # Set rotation (identity quaternion)
        static_transform.transform.rotation.x = 0.0
        static_transform.transform.rotation.y = 0.0
        static_transform.transform.rotation.z = 0.0
        static_transform.transform.rotation.w = 1.0
        
        return static_transform
    
    def broadcast_transform_callback(self):
        """Callback to broadcast the transform regularly"""
        # Update timestamp
        self.static_transform.header.stamp = self.get_clock().now().to_msg()
        # Broadcast the transform
        self.tf_broadcaster.sendTransform(self.static_transform)

    def check_transform_availability(self):
        # Only check if using TF tree
        if not self.use_tf_tree:
            return
            
        try:
            # Try to get a list of frames
            frames = self.tf_buffer.all_frames_as_string()
            
            # Check if we can find our target frame
            if frames:
                self.get_logger().debug(f"Available frames: {frames}")
                
                # If we haven't already confirmed transform availability
                if not self.transform_available:
                    self.get_logger().debug("TF frames are available, but waiting for specific frames needed...")
            
            # Try to check specific transform
            if self.tf_buffer.can_transform("depth_camera_link", self.target_frame, Time()):
                if not self.transform_available:
                    self.transform_available = True
                    self.get_logger().info(f"Transform from depth_camera_link to {self.target_frame} is now available")
            else:
                frame_pair = f"depth_camera_link -> {self.target_frame}"
                if frame_pair not in self.warned_frames:
                    self.warned_frames.add(frame_pair)
                    self.get_logger().warning(f"Transform from depth_camera_link to {self.target_frame} is not available. "
                                              f"Make sure robot_state_publisher or static_transform_publisher is running.")
        
        except Exception as e:
            self.get_logger().error(f"Error checking transform availability: {e}")

    def transform_callback(self, msg):
        # Always attempt to use our own transform first
        if self.use_fixed_frame:
            try:
                # Make sure our transform is available
                self.broadcast_transform_callback()
                
                # Try to lookup transform with a timeout
                start_time = self.get_clock().now()
                transform = None
                
                # Loop until we get a transform or timeout
                while (self.get_clock().now() - start_time).nanoseconds / 1e9 < self.tf_timeout:
                    try:
                        transform = self.tf_buffer.lookup_transform(
                            self.target_frame,
                            msg.header.frame_id,
                            Time(),
                            timeout=Duration(seconds=0.1))
                        break
                    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, 
                            tf2_ros.ExtrapolationException) as e:
                        # Broadcast transform again
                        self.broadcast_transform_callback()
                        self.get_logger().debug(f"Retrying transform lookup...")
                        continue
                
                # If we didn't get a transform, raise exception
                if transform is None:
                    raise tf2_ros.LookupException("Could not find transform after retries")
                
                # Transform the point
                transformed_point = do_transform_point(msg, transform)
                transformed_point.header.frame_id = self.target_frame
                
                # Publish the transformed point
                self.publisher.publish(transformed_point)
                
                if not self.transform_available:
                    self.transform_available = True
                    self.get_logger().info(f"Successfully transformed and published points to {self.target_frame} frame")
                    
                self.get_logger().debug(f'Transformed object position from {msg.header.frame_id} to {self.target_frame}: '
                                   f'[{transformed_point.point.x:.2f}, {transformed_point.point.y:.2f}, {transformed_point.point.z:.2f}]')
            
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                self.get_logger().warning(f'TF error after retries: {e}')
                # Fall back to direct coordinate publishing
                self.publish_direct_coordinates(msg)
        else:
            # Direct assignment approach
            self.publish_direct_coordinates(msg)
    
    def publish_direct_coordinates(self, msg):
        """Publish coordinates directly without transformation"""
        transformed_point = PointStamped()
        transformed_point.header.stamp = self.get_clock().now().to_msg()
        transformed_point.header.frame_id = self.target_frame
        transformed_point.point.x = msg.point.x
        transformed_point.point.y = msg.point.y
        transformed_point.point.z = msg.point.z
        self.publisher.publish(transformed_point)
        
        self.get_logger().debug(f'Published direct object position in {self.target_frame} frame: '
                           f'[{transformed_point.point.x:.2f}, {transformed_point.point.y:.2f}, {transformed_point.point.z:.2f}]')


def main(args=None):
    rclpy.init(args=args)
    node = ObjectTransformNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()