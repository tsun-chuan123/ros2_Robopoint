#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Header
from cv_bridge import CvBridge
import cv2
import numpy as np
import threading
import time
import os

class CameraPublisherNode(Node):
    def __init__(self):
        super().__init__('camera_publisher_node')
        
        # Parameters - 基於你的測試結果
        self.declare_parameter('d405_device', 4)      # RealSense D405 (gripper camera)
        self.declare_parameter('d435i_device', 6)     # RealSense D435i (main camera)
        self.declare_parameter('enable_d405', True)
        self.declare_parameter('enable_d435i', True)
        self.declare_parameter('publish_rate', 10.0)  # Hz
        self.declare_parameter('d405_width', 640)
        self.declare_parameter('d405_height', 480)
        self.declare_parameter('d405_fps', 30)
        self.declare_parameter('d435i_width', 1280)
        self.declare_parameter('d435i_height', 720)
        self.declare_parameter('d435i_fps', 10)
        
        # Get parameters
        self.d405_device = self.get_parameter('d405_device').value
        self.d435i_device = self.get_parameter('d435i_device').value
        self.enable_d405 = self.get_parameter('enable_d405').value
        self.enable_d435i = self.get_parameter('enable_d435i').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.d405_width = self.get_parameter('d405_width').value
        self.d405_height = self.get_parameter('d405_height').value
        self.d405_fps = self.get_parameter('d405_fps').value
        self.d435i_width = self.get_parameter('d435i_width').value
        self.d435i_height = self.get_parameter('d435i_height').value
        self.d435i_fps = self.get_parameter('d435i_fps').value
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # Camera objects and control
        self.d405_cap = None
        self.d435i_cap = None
        self.running = True
        
        # Publishers for D405 (gripper camera)
        if self.enable_d405:
            self.d405_image_pub = self.create_publisher(
                Image, 
                '/gripper_camera/gripper_camera/color/image_raw', 
                10
            )
            self.d405_info_pub = self.create_publisher(
                CameraInfo, 
                '/gripper_camera/gripper_camera/color/camera_info', 
                10
            )
            
        # Publishers for D435i (main camera)
        if self.enable_d435i:
            self.d435i_image_pub = self.create_publisher(
                Image, 
                '/camera/camera/color/image_raw', 
                10
            )
            self.d435i_info_pub = self.create_publisher(
                CameraInfo, 
                '/camera/camera/color/camera_info', 
                10
            )
            # Also publish to the main camera topic for GUI compatibility
            self.main_camera_pub = self.create_publisher(
                Image, 
                '/camera/image_raw', 
                10
            )
        
        # Initialize cameras
        self.init_cameras()
        
        # Start publishing threads
        if self.enable_d405 and self.d405_cap:
            self.d405_thread = threading.Thread(target=self.d405_publisher_loop, daemon=True)
            self.d405_thread.start()
            
        if self.enable_d435i and self.d435i_cap:
            self.d435i_thread = threading.Thread(target=self.d435i_publisher_loop, daemon=True)
            self.d435i_thread.start()
        
        self.get_logger().info("Camera publisher node initialized")
        self.get_logger().info(f"D405 (gripper): {'Enabled' if self.enable_d405 and self.d405_cap else 'Disabled'}")
        self.get_logger().info(f"D435i (main): {'Enabled' if self.enable_d435i and self.d435i_cap else 'Disabled'}")
    
    def init_cameras(self):
        """Initialize camera devices based on your test results"""
        if self.enable_d405:
            self.get_logger().info(f"Initializing D405 on /dev/video{self.d405_device}...")
            if self.init_single_camera(self.d405_device, self.d405_width, self.d405_height, self.d405_fps, 'd405'):
                self.d405_cap = cv2.VideoCapture(self.d405_device)
                if self.d405_cap.isOpened():
                    self.setup_camera_properties(self.d405_cap, self.d405_width, self.d405_height, self.d405_fps)
                    self.get_logger().info("D405 initialized successfully")
                else:
                    self.get_logger().error("Failed to open D405 camera")
                    self.d405_cap = None
        
        if self.enable_d435i:
            self.get_logger().info(f"Initializing D435i on /dev/video{self.d435i_device}...")
            if self.init_single_camera(self.d435i_device, self.d435i_width, self.d435i_height, self.d435i_fps, 'd435i'):
                self.d435i_cap = cv2.VideoCapture(self.d435i_device)
                if self.d435i_cap.isOpened():
                    self.setup_camera_properties(self.d435i_cap, self.d435i_width, self.d435i_height, self.d435i_fps)
                    self.get_logger().info("D435i initialized successfully")
                else:
                    self.get_logger().error("Failed to open D435i camera")
                    self.d435i_cap = None

    def init_single_camera(self, device_id, width, height, fps, camera_name):
        """Test if a single camera can be initialized"""
        device_path = f"/dev/video{device_id}"
        if not os.path.exists(device_path):
            self.get_logger().warn(f"Camera device {device_path} does not exist")
            return False
            
        # Quick test
        test_cap = cv2.VideoCapture(device_id)
        if not test_cap.isOpened():
            self.get_logger().warn(f"Cannot open {camera_name} on {device_path}")
            test_cap.release()
            return False
            
        ret, frame = test_cap.read()
        test_cap.release()
        
        if not ret:
            self.get_logger().warn(f"{camera_name} opened but cannot read frames")
            return False
            
        self.get_logger().info(f"{camera_name} test successful")
        return True

    def setup_camera_properties(self, cap, width, height, fps):
        """Setup camera properties"""
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        cap.set(cv2.CAP_PROP_FPS, fps)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # Reduce latency
        
        # Log actual properties
        actual_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = cap.get(cv2.CAP_PROP_FPS)
        self.get_logger().info(f"Camera properties: {actual_width}x{actual_height} @ {actual_fps} FPS")

    def create_camera_info(self, width, height, camera_name):
        """Create a basic CameraInfo message"""
        info = CameraInfo()
        info.header.frame_id = f"{camera_name}_color_optical_frame"
        info.width = width
        info.height = height
        
        # Basic camera matrix (approximate values for RealSense cameras)
        if camera_name == "gripper_camera":  # D405
            fx = fy = 600.0  # Approximate focal length for D405
        else:  # D435i
            fx = fy = 900.0  # Approximate focal length for D435i
            
        info.k = [fx, 0.0, width/2.0,
                  0.0, fy, height/2.0,
                  0.0, 0.0, 1.0]
        
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]  # No distortion (should be calibrated)
        
        info.r = [1.0, 0.0, 0.0,
                  0.0, 1.0, 0.0,
                  0.0, 0.0, 1.0]
        
        info.p = [fx, 0.0, width/2.0, 0.0,
                  0.0, fy, height/2.0, 0.0,
                  0.0, 0.0, 1.0, 0.0]
        
        return info
    
    def d405_publisher_loop(self):
        """D405 camera publishing loop"""
        self.get_logger().info("D405 publishing thread started")
        rate_sleep = 1.0 / self.publish_rate
        
        while self.running and rclpy.ok():
            try:
                if self.d405_cap and self.d405_cap.isOpened():
                    ret, frame = self.d405_cap.read()
                    if ret:
                        # Create ROS message
                        header = Header()
                        header.stamp = self.get_clock().now().to_msg()
                        header.frame_id = "gripper_camera_color_optical_frame"
                        
                        # Convert to ROS Image
                        ros_image = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                        ros_image.header = header
                        
                        # Create camera info
                        camera_info = self.create_camera_info(self.d405_width, self.d405_height, "gripper_camera")
                        camera_info.header = header
                        
                        # Publish
                        self.d405_image_pub.publish(ros_image)
                        self.d405_info_pub.publish(camera_info)
                        
                    else:
                        self.get_logger().warn_throttle(5.0, "Failed to read from D405")
                else:
                    self.get_logger().warn_throttle(5.0, "D405 camera not available")
                    
            except Exception as e:
                self.get_logger().error(f"D405 thread error: {e}")
            
            time.sleep(rate_sleep)
    
    def d435i_publisher_loop(self):
        """D435i camera publishing loop"""
        self.get_logger().info("D435i publishing thread started")
        rate_sleep = 1.0 / self.publish_rate
        
        while self.running and rclpy.ok():
            try:
                if self.d435i_cap and self.d435i_cap.isOpened():
                    ret, frame = self.d435i_cap.read()
                    if ret:
                        # Create ROS message
                        header = Header()
                        header.stamp = self.get_clock().now().to_msg()
                        header.frame_id = "camera_color_optical_frame"
                        
                        # Convert to ROS Image
                        ros_image = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                        ros_image.header = header
                        
                        # Create camera info
                        camera_info = self.create_camera_info(self.d435i_width, self.d435i_height, "camera")
                        camera_info.header = header
                        
                        # Publish to multiple topics
                        self.d435i_image_pub.publish(ros_image)
                        self.d435i_info_pub.publish(camera_info)
                        self.main_camera_pub.publish(ros_image)  # For GUI compatibility
                        
                    else:
                        self.get_logger().warn_throttle(5.0, "Failed to read from D435i")
                else:
                    self.get_logger().warn_throttle(5.0, "D435i camera not available")
                    
            except Exception as e:
                self.get_logger().error(f"D435i thread error: {e}")
            
            time.sleep(rate_sleep)
    
    def destroy_node(self):
        """Clean up resources"""
        self.get_logger().info("Shutting down camera publisher node...")
        self.running = False
        
        # Wait for threads to finish
        if hasattr(self, 'd405_thread') and self.d405_thread.is_alive():
            self.d405_thread.join(timeout=1.0)
        if hasattr(self, 'd435i_thread') and self.d435i_thread.is_alive():
            self.d435i_thread.join(timeout=1.0)
        
        # Release cameras
        if self.d405_cap:
            self.d405_cap.release()
        if self.d435i_cap:
            self.d435i_cap.release()
            
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        camera_publisher = CameraPublisherNode()
        rclpy.spin(camera_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        if 'camera_publisher' in locals():
            camera_publisher.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()