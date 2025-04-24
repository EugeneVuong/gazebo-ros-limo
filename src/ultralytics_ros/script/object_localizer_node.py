#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# ultralytics_ros
# Copyright (C) 2023-2024

import cv_bridge
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection2DArray
from ultralytics_ros.msg import YoloResult
from geometry_msgs.msg import PointStamped
import message_filters
import tf2_ros
import math


class ObjectLocalizerNode(Node):
    def __init__(self):
        super().__init__("object_localizer_node")
        
        # Declare parameters
        self.declare_parameter("depth_image_topic", "/limo/depth_camera_link/depth/image_raw")
        self.declare_parameter("yolo_result_topic", "/yolo_result")
        self.declare_parameter("object_position_topic", "/object_position")
        self.declare_parameter("frame_id", "depth_camera_link")

        # Get parameters
        self.depth_image_topic = self.get_parameter("depth_image_topic").get_parameter_value().string_value
        self.yolo_result_topic = self.get_parameter("yolo_result_topic").get_parameter_value().string_value
        self.object_position_topic = self.get_parameter("object_position_topic").get_parameter_value().string_value
        self.frame_id = self.get_parameter("frame_id").get_parameter_value().string_value
        
        # Initialize CV bridge
        self.bridge = cv_bridge.CvBridge()
        
        # Setup message filters to synchronize depth image and YOLO detection results
        self.depth_sub = message_filters.Subscriber(self, Image, self.depth_image_topic)
        self.detection_sub = message_filters.Subscriber(self, YoloResult, self.yolo_result_topic)
        
        # Create synchronizer for message filters
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.depth_sub, self.detection_sub],
            queue_size=10,
            slop=0.1)
        self.ts.registerCallback(self.synchronized_callback)
        
        # Create publisher for object positions
        self.position_pub = self.create_publisher(PointStamped, self.object_position_topic, 10)
        
        self.get_logger().info('Object localizer node is ready')

    def synchronized_callback(self, depth_msg, yolo_msg):
        try:
            # Convert depth image to numpy array
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough')
            
            # Process each detection
            for detection in yolo_msg.detections.detections:
                # Get detection center coordinates
                cx = int(detection.bbox.center.position.x)
                cy = int(detection.bbox.center.position.y)
                
                # Get the class and confidence
                class_id = detection.results[0].hypothesis.class_id
                confidence = detection.results[0].hypothesis.score
                
                # Get a small region around the detection center to find best depth value
                x_min = max(cx - 2, 0)
                x_max = min(cx + 3, depth_image.shape[1])
                y_min = max(cy - 2, 0)
                y_max = min(cy + 3, depth_image.shape[0])
                
                # Extract depth values in region and filter out zeros and nans
                depth_region = depth_image[y_min:y_max, x_min:x_max]
                valid_depths = depth_region[np.nonzero(depth_region)]
                valid_depths = valid_depths[~np.isnan(valid_depths)]
                
                # If we have valid depth values
                if valid_depths.size > 0:
                    # Get median depth (more robust than mean)
                    depth_value = float(np.median(valid_depths))
                    
                    # Skip if depth is too large (likely invalid)
                    if depth_value > 10.0:
                        continue
                    
                    # Calculate 3D coordinates (simple pinhole model)
                    # Assuming depth camera's focal length and center point
                    # These would ideally come from camera info
                    fx = float(depth_image.shape[1] / 2)  # approximate focal length
                    fy = float(depth_image.shape[0] / 2)  # approximate focal length
                    cx_offset = float(depth_image.shape[1] / 2)  # principal point x
                    cy_offset = float(depth_image.shape[0] / 2)  # principal point y
                    
                    # Convert from pixel coordinates to camera coordinates
                    x = float((cx - cx_offset) * depth_value / fx)
                    y = float((cy - cy_offset) * depth_value / fy)
                    z = float(depth_value)
                    
                    # Create and publish PointStamped message
                    point_msg = PointStamped()
                    point_msg.header.stamp = self.get_clock().now().to_msg()
                    point_msg.header.frame_id = self.frame_id
                    point_msg.point.x = float(z)  # typically depth is z-axis in camera frame
                    point_msg.point.y = float(-x)  # typically right is negative y in camera frame
                    point_msg.point.z = float(-y)  # typically down is negative z in camera frame
                    
                    self.position_pub.publish(point_msg)
                    
                    self.get_logger().info(
                        f"Object: {class_id} at ({point_msg.point.x:.2f}, "
                        f"{point_msg.point.y:.2f}, {point_msg.point.z:.2f}) "
                        f"meters with confidence {confidence:.2f}"
                    )
        except Exception as e:
            self.get_logger().error(f"Error processing object location: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = ObjectLocalizerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()