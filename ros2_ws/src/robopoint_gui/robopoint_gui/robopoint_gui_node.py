#!/usr/bin/env python3
import sys
import rclpy
from rclpy.node import Node
from PyQt5 import QtWidgets, QtGui, QtCore
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from robopoint_interfaces.msg import SpatialAffordanceQuery
from cv_bridge import CvBridge
import numpy as np
import cv2

class RoboPointGUI(Node):
    def __init__(self):
        super().__init__('robopoint_gui')
        self.bridge = CvBridge()
        
        # Track camera activity - 修正topic名稱
        self.camera_activity = {
            '/camera/camera/color/image_raw': 0,  # D435i主相機
            '/gripper_camera/gripper_camera/color/image_rect_raw': 0  # D405夾爪相機
        }

        # Store latest images for query selection
        self.latest_main_image = None  # D435i (旋轉後的影像)
        self.latest_gripper_image = None  # D405 (原始影像)
        
        # Store original images for display
        self.original_main_image = None  # D435i原始影像
        self.original_gripper_image = None  # D405原始影像
        
        # Store selected camera info for result processing
        self.last_query_camera = "Main Camera (D435i)"  # 追蹤最後使用的相機
        
        # Publishers
        self.query_pub = self.create_publisher(SpatialAffordanceQuery, 'spatial_affordance_query', 10)
        self.external_image_pub = self.create_publisher(Image, 'external_camera/image_raw', 10)

        # Subscribe to correct camera topics
        self.create_subscription(Image, '/camera/camera/color/image_raw',
                               lambda msg: self.update_main_camera(msg, '/camera/camera/color/image_raw'), 10)
        self.create_subscription(Image, '/gripper_camera/gripper_camera/color/image_rect_raw', 
                               lambda msg: self.update_gripper_camera(msg, '/gripper_camera/gripper_camera/color/image_rect_raw'), 10)
        self.create_subscription(Image, 'affordance_visualization', self.update_vis_image, 10)

        # GUI Setup
        self.app = QtWidgets.QApplication(sys.argv)
        self.win = QtWidgets.QWidget()
        self.win.setWindowTitle("RoboPoint GUI - Multi-Camera View")
        self.win.resize(1400, 800)  # 擴大窗口尺寸
        
        # Main layout
        main_layout = QtWidgets.QHBoxLayout(self.win)

        # Left panel: Control area
        left_panel = QtWidgets.QVBoxLayout()
        left_widget = QtWidgets.QWidget()
        left_widget.setLayout(left_panel)
        left_widget.setMaximumWidth(350)
        
        # Query input area
        query_group = QtWidgets.QGroupBox("Spatial Affordance Query")
        query_layout = QtWidgets.QVBoxLayout(query_group)
        
        # Camera selection
        self.camera_select = QtWidgets.QComboBox()
        self.camera_select.addItems(["Main Camera (D435i)", "Gripper Camera (D405)"])
        self.camera_select.setStyleSheet("QComboBox { padding: 5px; font-size: 11px; }")
        
        query_layout.addWidget(QtWidgets.QLabel("Select camera for query:"))
        query_layout.addWidget(self.camera_select)
        
        self.text_input = QtWidgets.QTextEdit()
        self.text_input.setPlaceholderText("Enter your spatial affordance query here...")
        self.text_input.setMaximumHeight(120)
        
        self.send_button = QtWidgets.QPushButton("Send Query")
        self.send_button.clicked.connect(self.send_command)
        self.send_button.setStyleSheet("QPushButton { background-color: #4CAF50; color: white; font-weight: bold; padding: 10px; }")
        
        query_layout.addWidget(self.text_input)
        query_layout.addWidget(self.send_button)
        
        # Status area
        status_group = QtWidgets.QGroupBox("System Status")
        status_layout = QtWidgets.QVBoxLayout(status_group)
        
        self.status_label = QtWidgets.QLabel("Ready")
        self.status_label.setStyleSheet("color: green; font-weight: bold; font-size: 12px;")
        
        # Camera status indicators
        self.main_camera_status = QtWidgets.QLabel("Main Camera: Waiting...")
        self.main_camera_status.setStyleSheet("color: orange; font-size: 10px;")
        
        self.gripper_camera_status = QtWidgets.QLabel("Gripper Camera: Waiting...")
        self.gripper_camera_status.setStyleSheet("color: orange; font-size: 10px;")
        
        self.camera_activity_label = QtWidgets.QLabel("Checking cameras...")
        self.camera_activity_label.setStyleSheet("color: gray; font-size: 9px;")
        
        status_layout.addWidget(self.status_label)
        status_layout.addWidget(QtWidgets.QLabel("Camera Status:"))
        status_layout.addWidget(self.main_camera_status)
        status_layout.addWidget(self.gripper_camera_status)
        status_layout.addWidget(self.camera_activity_label)
        self.last_query_source_label = QtWidgets.QLabel("Last query camera: (none)")
        self.last_query_source_label.setStyleSheet("color: #333; font-size: 10px; font-weight: bold;")
        status_layout.addWidget(self.last_query_source_label)
        
        # Add groups to left panel
        left_panel.addWidget(query_group)
        left_panel.addWidget(status_group)
        left_panel.addStretch()  # Push everything to top

        # Right panel: Camera displays
        right_layout = QtWidgets.QVBoxLayout()
        
        # Top row: Live camera feeds
        camera_row = QtWidgets.QHBoxLayout()
        
        # Main camera (D435i) display
        main_camera_group = QtWidgets.QGroupBox("Main Camera (D435i) - Rotated 90°")
        main_camera_layout = QtWidgets.QVBoxLayout(main_camera_group)
        
        self.main_camera_label = QtWidgets.QLabel("Main Camera Feed")
        self.main_camera_label.setFixedSize(480, 360)
        self.main_camera_label.setStyleSheet("border: 2px solid #2196F3; background-color: #f0f0f0;")
        self.main_camera_label.setAlignment(QtCore.Qt.AlignCenter)
        self.main_camera_label.setScaledContents(False)
        
        self.main_camera_info = QtWidgets.QLabel("Resolution: - | FPS: -")
        self.main_camera_info.setStyleSheet("color: gray; font-size: 9px;")
        
        main_camera_layout.addWidget(self.main_camera_label)
        main_camera_layout.addWidget(self.main_camera_info)
        
        # Gripper camera (D405) display  
        gripper_camera_group = QtWidgets.QGroupBox("Gripper Camera (D405) - Original")
        gripper_camera_layout = QtWidgets.QVBoxLayout(gripper_camera_group)
        
        self.gripper_camera_label = QtWidgets.QLabel("Gripper Camera Feed")
        self.gripper_camera_label.setFixedSize(480, 360)
        self.gripper_camera_label.setStyleSheet("border: 2px solid #FF9800; background-color: #f0f0f0;")
        self.gripper_camera_label.setAlignment(QtCore.Qt.AlignCenter)
        self.gripper_camera_label.setScaledContents(False)
        
        self.gripper_camera_info = QtWidgets.QLabel("Resolution: - | FPS: -")
        self.gripper_camera_info.setStyleSheet("color: gray; font-size: 9px;")
        
        gripper_camera_layout.addWidget(self.gripper_camera_label)
        gripper_camera_layout.addWidget(self.gripper_camera_info)
        
        camera_row.addWidget(main_camera_group)
        camera_row.addWidget(gripper_camera_group)
        
        # Bottom row: Affordance visualization
        vis_group = QtWidgets.QGroupBox("Affordance Visualization Result")
        vis_layout = QtWidgets.QVBoxLayout(vis_group)
        
        self.vis_image_label = QtWidgets.QLabel("Affordance Result")
        self.vis_image_label.setFixedSize(980, 360)
        self.vis_image_label.setStyleSheet("border: 2px solid #4CAF50; background-color: #f0f0f0;")
        self.vis_image_label.setAlignment(QtCore.Qt.AlignCenter)
        self.vis_image_label.setScaledContents(False)
        
        self.vis_info_label = QtWidgets.QLabel("Waiting for query results...")
        self.vis_info_label.setStyleSheet("color: gray; font-size: 10px;")
        
        vis_layout.addWidget(self.vis_image_label)
        vis_layout.addWidget(self.vis_info_label)
        
        # Assemble right layout
        right_layout.addLayout(camera_row)
        right_layout.addWidget(vis_group)
        
        # Add to main layout
        main_layout.addWidget(left_widget)
        main_layout.addLayout(right_layout)
        
        self.win.show()

        # Timers
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(self.app.processEvents)
        self.timer.start(50)
        
        self.activity_timer = QtCore.QTimer()
        self.activity_timer.timeout.connect(self.update_camera_activity_display)
        self.activity_timer.start(2000)

        # Frame counters for FPS calculation
        self.main_camera_frames = 0
        self.gripper_camera_frames = 0
        self.last_fps_update = self.get_clock().now()

        self.get_logger().info("RoboPoint GUI initialized - D435i rotated, D405 original")

    def update_main_camera(self, msg, topic_name):
        """Update main camera display (D435i) with 90-degree rotation"""
        self.camera_activity[topic_name] += 1
        self.main_camera_frames += 1
        
        self.main_camera_status.setText("Main Camera: D435i Active")
        self.main_camera_status.setStyleSheet("color: green; font-size: 10px; font-weight: bold;")
        
        try:
            # 保存原始影像
            self.original_main_image = msg
            
            # Convert to OpenCV format and rotate 90 degrees clockwise
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            rotated_image = cv2.rotate(cv_image, cv2.ROTATE_90_CLOCKWISE)
            
            # Convert back to ROS message for query use
            msg_rotated = self.bridge.cv2_to_imgmsg(rotated_image, encoding="bgr8")
            msg_rotated.header = msg.header
            
            # Store rotated image for query use
            self.latest_main_image = msg_rotated
            
            # Display rotated image
            self.display_image(msg_rotated, self.main_camera_label, "main")
            
            # Update camera info with rotated dimensions
            h, w = rotated_image.shape[:2]
            self.main_camera_info.setText(f"Resolution: {w}x{h} | Active (Rotated 90°)")
            
        except Exception as e:
            self.get_logger().error(f"Error processing main camera: {e}")
            self.main_camera_info.setText(f"Error: {str(e)}")

    def update_gripper_camera(self, msg, topic_name):
        """Update gripper camera display (D405) - 保持原始方向"""
        self.get_logger().debug(f"Received gripper camera image: {msg.width}x{msg.height}, encoding: {msg.encoding}")
        
        self.camera_activity[topic_name] += 1
        self.gripper_camera_frames += 1
        
        self.gripper_camera_status.setText("Gripper Camera: D405 Active")
        self.gripper_camera_status.setStyleSheet("color: green; font-size: 10px; font-weight: bold;")
        
        try:
            # 保存原始影像 (D405不需要旋轉)
            self.original_gripper_image = msg
            self.latest_gripper_image = msg  # D405使用原始影像進行查詢
            
            # 直接顯示原始影像 (不旋轉)
            self.display_image(msg, self.gripper_camera_label, "gripper")
            
            # Update camera info
            self.gripper_camera_info.setText(f"Resolution: {msg.width}x{msg.height} | Active (Original)")
            self.get_logger().debug(f"Successfully processed gripper camera image: {msg.width}x{msg.height}")
            
        except Exception as e:
            self.get_logger().error(f"Error processing gripper camera: {e}")
            self.gripper_camera_info.setText(f"Error: {str(e)}")

    def update_vis_image(self, msg):
        """Update affordance visualization - 根據傳回影像的 header.frame_id 或最後查詢相機決定是否需要旋轉"""
        self.get_logger().debug("Received affordance visualization")

        try:
            # Prefer using the incoming visualization message's header to decide which camera it came from.
            source_frame = ''
            try:
                source_frame = msg.header.frame_id
            except Exception:
                source_frame = self.last_query_camera or ''

            self.get_logger().debug(f"Visualization header.frame_id: {source_frame}")

            # If the visualization is for the main camera (D435i), rotate to match display
            # Only rotate if the visualization was generated for the original D435i orientation
            should_rotate = False
            if (('/camera/camera' in source_frame or 'main_camera' in source_frame) and not source_frame.endswith('_rotated')):
                should_rotate = True
            if (self.last_query_camera == "Main Camera (D435i)" and 'gripper' not in source_frame and not source_frame.endswith('_rotated')):
                should_rotate = True

            if should_rotate:
                cv_result = self.bridge.imgmsg_to_cv2(msg, "bgr8")
                rotated_result = cv2.rotate(cv_result, cv2.ROTATE_90_CLOCKWISE)
                rotated_msg = self.bridge.cv2_to_imgmsg(rotated_result, encoding="bgr8")
                # keep timestamp but set frame to source_frame to be explicit
                try:
                    rotated_msg.header.stamp = msg.header.stamp
                    rotated_msg.header.frame_id = source_frame
                except Exception:
                    pass

                self.display_image(rotated_msg, self.vis_image_label, "visualization")
                self.vis_info_label.setText(f"Latest affordance analysis result (D435i - Rotated)")
                self.get_logger().debug("Rotated visualization result for D435i display")
            else:
                # For gripper camera (D405) results, display directly
                display_msg = msg
                self.display_image(display_msg, self.vis_image_label, "visualization")
                self.vis_info_label.setText(f"Latest affordance analysis result (D405 - Original)")
                self.get_logger().debug("Direct visualization result for D405 display")

            self.vis_info_label.setStyleSheet("color: green; font-size: 10px; font-weight: bold;")

        except Exception as e:
            self.get_logger().error(f"Error processing visualization: {e}")

    def display_image(self, msg, label, image_type):
        """Display image in the specified label"""
        try:
            self.get_logger().debug(f"Displaying {image_type} image: {msg.width}x{msg.height}")
            
            # Convert ROS Image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Convert BGR to RGB
            rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            h, w, ch = rgb_image.shape
            bytes_per_line = ch * w
            
            # Create Qt image
            qt_image = QtGui.QImage(rgb_image.data, w, h, bytes_per_line, QtGui.QImage.Format_RGB888)
            
            # Scale image to fit label while maintaining aspect ratio
            pixmap = QtGui.QPixmap.fromImage(qt_image)
            scaled_pixmap = pixmap.scaled(
                label.width(), label.height(), 
                QtCore.Qt.KeepAspectRatio, 
                QtCore.Qt.SmoothTransformation
            )
            
            # Display image
            label.setPixmap(scaled_pixmap)
            label.repaint()
            
            self.get_logger().debug(f"Successfully displayed {image_type} image")
            
        except Exception as e:
            self.get_logger().error(f"Image display error ({image_type}): {e}")
            label.setText(f"Image Error: {str(e)}")

    def update_camera_activity_display(self):
        """Update camera activity display and FPS info"""
        active_cameras = []
        total_activity = 0
        
        for topic, count in self.camera_activity.items():
            if count > 0:
                total_activity += count
                if 'gripper_camera' in topic:
                    active_cameras.append(f"D405({count})")
                elif 'camera/camera' in topic:  # 修正檢測邏輯
                    active_cameras.append(f"D435i({count})")
        
        # Update activity display
        if active_cameras:
            activity_text = f"Active streams: {', '.join(active_cameras)}"
            self.camera_activity_label.setStyleSheet("color: green; font-size: 9px;")
        else:
            activity_text = "No camera activity detected"
            self.camera_activity_label.setStyleSheet("color: red; font-size: 9px;")
        
        self.camera_activity_label.setText(activity_text)
        
        # Calculate and display FPS
        current_time = self.get_clock().now()
        time_diff = (current_time - self.last_fps_update).nanoseconds / 1e9
        
        if time_diff >= 2.0:  # Update FPS every 2 seconds
            main_fps = self.main_camera_frames / time_diff
            gripper_fps = self.gripper_camera_frames / time_diff
            
            # Update info labels with FPS
            current_main_text = self.main_camera_info.text().split(' | ')[0]
            self.main_camera_info.setText(f"{current_main_text} | FPS: {main_fps:.1f}")
            
            current_gripper_text = self.gripper_camera_info.text().split(' | ')[0]
            self.gripper_camera_info.setText(f"{current_gripper_text} | FPS: {gripper_fps:.1f}")
            
            # Reset counters
            self.main_camera_frames = 0
            self.gripper_camera_frames = 0
            self.last_fps_update = current_time
        
        # Reset activity counters
        for topic in self.camera_activity:
            self.camera_activity[topic] = 0

    def send_command(self):
        """Send spatial affordance query with selected camera"""
        text = self.text_input.toPlainText().strip()
        if not text:
            self.status_label.setText("Please enter a query")
            self.status_label.setStyleSheet("color: red; font-weight: bold; font-size: 12px;")
            return

        try:
            # Get selected camera
            selected_cam = self.camera_select.currentText()
            self.last_query_camera = selected_cam  # 記錄最後使用的相機

            # Update on-screen indicator for which camera this query uses
            self.last_query_source_label.setText(f"Last query camera: {selected_cam}")
            self.last_query_source_label.setStyleSheet(
                "color: #2e7d32; font-size: 10px; font-weight: bold;"
            )

            # Determine topic/frame_id and send corresponding image
            if "Main Camera" in selected_cam:
                # Use a distinct frame_id to indicate we sent the rotated D435i image
                topic_name = '/camera/camera/color/image_raw_rotated'  # D435i rotated
                image_to_send = self.latest_main_image
            else:  # Gripper Camera
                topic_name = '/gripper_camera/gripper_camera/color/image_rect_raw'  # D405 original
                image_to_send = self.latest_gripper_image

            if image_to_send is None:
                self.status_label.setText(f"{selected_cam} image not available")
                self.status_label.setStyleSheet("color: red; font-weight: bold; font-size: 12px;")
                return

            # Ensure the image header uses the topic/frame we want the backend to use
            try:
                image_to_send.header.frame_id = topic_name
                image_to_send.header.stamp = self.get_clock().now().to_msg()
            except Exception:
                # If header not present, create one
                try:
                    image_to_send.header = Header()
                    image_to_send.header.frame_id = topic_name
                    image_to_send.header.stamp = self.get_clock().now().to_msg()
                except Exception:
                    pass

            # Publish the image that corresponds to the selected camera
            self.external_image_pub.publish(image_to_send)
            self.get_logger().info(f"Published image for query from {selected_cam} with header.frame_id={topic_name}")

            # Create and send query message with the same frame_id/topic so backend can correlate
            msg = SpatialAffordanceQuery()
            msg.header = Header()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = topic_name
            msg.query_text = text
            msg.image_process_mode = "Pad"

            # Send query
            self.query_pub.publish(msg)

            # Update status
            self.get_logger().info(f"Sent spatial affordance query ({selected_cam}): {text} with frame_id: {topic_name}")
            self.status_label.setText(f"Query sent ({selected_cam}): {text[:20]}...")
            self.status_label.setStyleSheet("color: blue; font-weight: bold; font-size: 12px;")

            # Update visualization info
            self.vis_info_label.setText(f"Processing query with {selected_cam}, waiting for results...")
            self.vis_info_label.setStyleSheet("color: orange; font-size: 10px;")

            # Clear input
            self.text_input.clear()

        except Exception as e:
            self.get_logger().error(f"Failed to send query: {e}")
            self.status_label.setText(f"Send failed: {str(e)}")
            self.status_label.setStyleSheet("color: red; font-weight: bold; font-size: 12px;")

def main(args=None):
    rclpy.init(args=args)
    gui = RoboPointGUI()

    try:
        # Create executor
        executor = rclpy.executors.SingleThreadedExecutor()
        executor.add_node(gui)

        # Use QTimer to process ROS messages periodically
        timer = QtCore.QTimer()
        timer.timeout.connect(lambda: executor.spin_once(timeout_sec=0.001))
        timer.start(10)  # Process every 10ms

        # Enter PyQt main event loop
        sys.exit(gui.app.exec_())
        
    except KeyboardInterrupt:
        gui.get_logger().info("GUI shutdown requested")
    finally:
        gui.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()