#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from sensor_msgs.msg import Image
from std_msgs.msg import String, Empty
from geometry_msgs.msg import Point, PointStamped
from robopoint_interfaces.msg import SpatialAffordanceQuery, SpatialAffordanceResponse, AffordancePoint
from robopoint_interfaces.srv import (
    ProcessImage, 
    RefreshAllWorkers, 
    ListModels, 
    GetWorkerAddress, 
    WorkerGenerateStream
)
from cv_bridge import CvBridge
import cv2
import numpy as np
from PIL import Image as PILImage, ImageDraw
import json
import hashlib
import datetime
import os
import re
import time
import base64
import io
import threading
from concurrent.futures import TimeoutError
from robopoint_controller.conversation import (default_conversation, conv_templates, SeparatorStyle)
from robopoint_controller.constants import LOGDIR
from robopoint_controller.utils import (
    build_logger, server_error_msg, violates_moderation, moderation_msg
)


class RoboPointNode(Node):
    def __init__(self):
        super().__init__('robopoint_node')
        
        # Initialize logger
        self.logger = build_logger("robopoint_ros2", "robopoint_ros2.log")
        
        # Priority for model selection
        self.priority = {
            "vicuna-13b": "aaaaaaa",
            "koala-13b": "aaaaaab",
        }
        
        # Parameters
        self.declare_parameter('moderate', False)
        self.declare_parameter('temperature', 1.0)
        self.declare_parameter('top_p', 0.7)
        self.declare_parameter('max_output_tokens', 512)
        self.declare_parameter('model_name', '')
        self.declare_parameter('service_timeout', 30.0)
        
        self.moderate = self.get_parameter('moderate').value
        self.temperature = self.get_parameter('temperature').value
        self.top_p = self.get_parameter('top_p').value
        self.max_output_tokens = self.get_parameter('max_output_tokens').value
        self.model_name = self.get_parameter('model_name').value
        self.service_timeout = self.get_parameter('service_timeout').value
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Initialize conversation state
        self.conversation_state = default_conversation.copy()
        
        # Get available models
        self.models = []
        self.model_init_timer = self.create_timer(5.0, self.delayed_model_init)
        self.model_initialized = False
        
        # Publishers
        self.affordance_pub = self.create_publisher(
            SpatialAffordanceResponse, 
            'spatial_affordance_response', 
            10
        )
        
        self.visualization_pub = self.create_publisher(
            Image, 
            'affordance_visualization', 
            10
        )
        
        # Subscribers
        self.query_sub = self.create_subscription(
            SpatialAffordanceQuery,
            'spatial_affordance_query',
            self.handle_affordance_query,
            10
        )
        
        # Subscribe to external camera images (from GUI)
        self.image_sub = self.create_subscription(
            Image,
            'external_camera/image_raw',
            self.handle_image_callback,
            10
        )
        
        # Subscribe to RealSense cameras (from camera publisher node)
        self.realsense_main_sub = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.handle_realsense_callback,
            10
        )
        
        self.realsense_gripper_sub = self.create_subscription(
            Image,
            '/gripper_camera/gripper_camera/color/image_raw',
            self.handle_realsense_callback,
            10
        )
        
        # Services
        self.process_service = self.create_service(
            ProcessImage,
            'process_image_affordance',
            self.process_image_service
        )
        
        # Store latest image
        self.latest_image = None
        self.latest_pil_image = None
        # Store external images keyed by header.frame_id (so GUI can send images for a specific frame_id)
        self.external_images = {}

        self.get_logger().info('RoboPoint node initialized - processing node only')
        self.get_logger().info('Waiting for camera images from camera_publisher_node or GUI')

    def call_service(self, service_type, service_name, request, timeout_sec=30.0):
        """Isolated service call method"""
        tmp_node = rclpy.create_node('tmp_client_node')
        tmp_client = tmp_node.create_client(service_type, service_name)
        if not tmp_client.wait_for_service(timeout_sec=5.0):
            tmp_node.destroy_node()
            return None

        future = tmp_client.call_async(request)
        tmp_exec = SingleThreadedExecutor()
        tmp_exec.add_node(tmp_node)
        tmp_exec.spin_until_future_complete(future, timeout_sec=timeout_sec)
        tmp_exec.shutdown()

        result = future.result() if future.done() else None
        tmp_node.destroy_node()
        return result

    def get_model_list(self):
        """Get available models from controller via ROS2 services"""
        try:
            # Refresh all workers first
            refresh_request = RefreshAllWorkers.Request()
            refresh_result = self.call_service(
                RefreshAllWorkers,
                '/refresh_all_workers',
                refresh_request,
                timeout_sec=15.0
            )
            
            if refresh_result is None:
                self.get_logger().warn("Failed to refresh workers - no result")
                return []
            elif not refresh_result.success:
                self.get_logger().warn(f"Failed to refresh workers: {refresh_result.message}")
                return []
            
            self.get_logger().info("Workers refreshed successfully")
            
            # Get model list
            list_request = ListModels.Request()
            list_result = self.call_service(
                ListModels,
                '/list_models',
                list_request,
                timeout_sec=15.0
            )
            
            if list_result is None:
                self.get_logger().warn("Failed to get model list - no result")
                return []
            
            self.get_logger().info(f"[DEBUG] list_models result: success={list_result.success}, models={list_result.model_names}, workers={list_result.worker_ids}")

            if not list_result.success:
                self.get_logger().warn(f"Failed to get model list: {list_result.message}")
                return []
                
            models = list_result.model_names
            models.sort(key=lambda x: self.priority.get(x, x))
            self.get_logger().info(f"Models: {models}")
            return models
            
        except Exception as e:
            self.get_logger().error(f"Error getting model list: {e}")
            return []

    def handle_realsense_callback(self, msg):
        """Handle incoming RealSense camera image messages"""
        try:
            # Convert ROS Image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Update latest image (RealSense has priority)
            self.latest_image = cv_image
            
            # Convert to PIL for processing
            rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            self.latest_pil_image = PILImage.fromarray(rgb_image)
            
            # Log which camera is being used (throttled to avoid spam)
            if hasattr(self, '_last_camera_log_time'):
                if time.time() - self._last_camera_log_time > 10.0:
                    self.get_logger().info(f"Using RealSense camera: {msg.header.frame_id}")
                    self._last_camera_log_time = time.time()
            else:
                self.get_logger().info(f"RealSense camera connected: {msg.header.frame_id}")
                self._last_camera_log_time = time.time()
            
        except Exception as e:
            self.get_logger().error(f"Error processing RealSense image: {e}")

    def handle_image_callback(self, msg):
        """Handle incoming external image messages (from GUI)"""
        try:
            # Convert ROS Image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # Always store external images by their header.frame_id so queries can target them
            frame_id = ''
            try:
                frame_id = msg.header.frame_id
            except Exception:
                frame_id = ''

            rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            pil_image = PILImage.fromarray(rgb_image)
            if frame_id:
                self.external_images[frame_id] = pil_image
                self.get_logger().debug(f"Stored external image under frame_id: {frame_id}")

            # If no other source (realsense) has provided an image yet, set latest as fallback
            if self.latest_image is None:
                self.latest_image = cv_image
                self.latest_pil_image = pil_image
                self.get_logger().info("Using external image from GUI as fallback")

        except Exception as e:
            self.get_logger().error(f"Error processing external image: {e}")

    def handle_affordance_query(self, msg):
        """Handle spatial affordance query messages"""
        # Prefer an image that matches the query's header.frame_id (published by GUI)
        requested_frame = ''
        try:
            requested_frame = msg.header.frame_id
        except Exception:
            requested_frame = ''

        pil_image_to_use = None
        if requested_frame and requested_frame in self.external_images:
            pil_image_to_use = self.external_images[requested_frame]
            self.get_logger().debug(f"Using external image matched to query.frame_id: {requested_frame}")
        else:
            pil_image_to_use = self.latest_pil_image

        if pil_image_to_use is None:
            self.get_logger().warn("No image available for processing")
            return

        try:
            # Process the query
            response = self.process_affordance_request(
                pil_image_to_use, 
                msg.query_text,
                msg.image_process_mode
            )

            # Publish response
            if response:
                self.affordance_pub.publish(response)

                # Create and publish visualization
                vis_image = self.create_visualization(
                    pil_image_to_use, 
                    response.affordance_points
                )
                if vis_image is not None:
                    vis_msg = self.bridge.cv2_to_imgmsg(vis_image, "bgr8")
                    vis_msg.header = msg.header
                    self.visualization_pub.publish(vis_msg)

        except Exception as e:
            self.get_logger().error(f"Error handling affordance query: {e}")

    def process_image_service(self, request, response):
        """Service to process image with affordance query"""
        try:
            # Convert ROS Image to PIL
            cv_image = self.bridge.imgmsg_to_cv2(request.image, "bgr8")
            rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
            pil_image = PILImage.fromarray(rgb_image)
            
            # Process the query
            affordance_response = self.process_affordance_request(
                pil_image,
                request.query_text,
                request.image_process_mode
            )
            
            if affordance_response:
                response.success = True
                response.affordance_response = affordance_response
                
                # Create visualization
                vis_image = self.create_visualization(
                    pil_image, 
                    affordance_response.affordance_points
                )
                if vis_image is not None:
                    response.visualization = self.bridge.cv2_to_imgmsg(vis_image, "bgr8")
            else:
                response.success = False
                response.error_message = "Failed to process affordance query"
                
        except Exception as e:
            response.success = False
            response.error_message = str(e)
            self.get_logger().error(f"Service error: {e}")
            
        return response

    def process_affordance_request(self, pil_image, query_text, image_process_mode):
        """Process affordance request and return response"""
        try:
            # Check moderation if enabled
            if self.moderate and violates_moderation(query_text):
                self.get_logger().warn("Query flagged by moderation")
                return None
            
            # Prepare query text with formatting instructions
            formatted_query = query_text + " Your answer should be formatted as a list of tuples, " \
                            "i.e. [(x1, y1), (x2, y2), ...], where each tuple contains the " \
                            "x and y coordinates of a point satisfying the conditions above." \
                            " The coordinates should be between 0 and 1, indicating the " \
                            "normalized pixel locations of the points in the image."
            
            # Truncate text
            formatted_query = formatted_query[:1536]
            if pil_image is not None:
                formatted_query = formatted_query[:1200]
            
            # Setup conversation
            state = default_conversation.copy()
            if pil_image is not None:
                text_input = ('<image>\n' + formatted_query, pil_image, image_process_mode)
            else:
                text_input = formatted_query
                
            state.append_message(state.roles[0], text_input)
            state.append_message(state.roles[1], None)
            
            # Get model response
            model_response = self.query_model(state)
            
            if model_response:
                # Parse response for coordinate points
                points = self.find_vectors(model_response)
                
                # Create response message
                response_msg = SpatialAffordanceResponse()
                response_msg.header.stamp = self.get_clock().now().to_msg()
                response_msg.query_text = query_text
                response_msg.model_response = model_response
                response_msg.success = True
                
                # Convert points to AffordancePoint messages
                for point in points:
                    if len(point) == 2:  # 2D points
                        affordance_point = AffordancePoint()
                        affordance_point.x = float(point[0])
                        affordance_point.y = float(point[1])
                        affordance_point.confidence = 1.0  # Could be extended
                        response_msg.affordance_points.append(affordance_point)
                
                return response_msg
            
        except Exception as e:
            self.get_logger().error(f"Error processing affordance request: {e}")
            
        return None

    def pil_image_to_base64(self, pil_image):
        """Convert PIL image to base64 string"""
        buffered = io.BytesIO()
        pil_image.save(buffered, format="JPEG", quality=85)
        img_str = base64.b64encode(buffered.getvalue()).decode()
        return img_str

    def query_model(self, state):
        """Query the vision-language model via ROS2 services"""
        try:
            # Get worker address
            get_worker_request = GetWorkerAddress.Request()
            get_worker_request.model = self.model_name
            
            worker_result = self.call_service(
                GetWorkerAddress,
                '/get_worker_address',
                get_worker_request,
                timeout_sec=30.0
            )
            
            if worker_result is None:
                self.get_logger().error("Failed to get worker address - no result")
                return None
                
            if not worker_result.success or not worker_result.address:
                self.get_logger().error(f"No available worker for model {self.model_name}")
                return None
            
            worker_name = worker_result.address
            
            # Select template based on model name
            if 'vicuna' in self.model_name.lower():
                template_name = "vicuna_v1"
            elif "llama" in self.model_name.lower():
                template_name = "llava_llama_2"
            elif "mistral" in self.model_name.lower():
                template_name = "mistral_instruct"
            elif "mpt" in self.model_name.lower():
                template_name = "mpt"
            else:
                template_name = "llava_v1"

            new_state = conv_templates[template_name].copy()
            new_state.append_message(new_state.roles[0], state.messages[-2][1])
            new_state.append_message(new_state.roles[1], None)
            state = new_state
            
            # Prepare request
            prompt = state.get_prompt()
            pil_images, images, transforms = state.get_images()
            
            # Convert PIL images to base64 strings for ROS2 message
            image_data = []
            if pil_images:
                for pil_img in pil_images:
                    img_base64 = self.pil_image_to_base64(pil_img)
                    image_data.append(img_base64)
                    
                    # Save images for logging
                    image_hash = hashlib.md5(pil_img.tobytes()).hexdigest()
                    t = datetime.datetime.now()
                    filename = os.path.join(LOGDIR, "serve_images", 
                                          f"{t.year}-{t.month:02d}-{t.day:02d}", 
                                          f"{image_hash}.jpg")
                    if not os.path.isfile(filename):
                        os.makedirs(os.path.dirname(filename), exist_ok=True)
                        pil_img.save(filename)
            
            # Prepare generate request
            generate_request = WorkerGenerateStream.Request()
            generate_request.model = self.model_name
            generate_request.prompt = prompt
            generate_request.temperature = self.temperature
            generate_request.top_p = self.top_p
            generate_request.max_new_tokens = min(int(self.max_output_tokens), 1536)
            generate_request.images = image_data
            
            # Determine stop tokens
            if state.sep_style in [SeparatorStyle.SINGLE, SeparatorStyle.MPT]:
                generate_request.stop = state.sep
            else:
                generate_request.stop = state.sep2
            
            # Call worker service
            service_name = f'/model_worker_{worker_name}/generate_stream'
            generate_result = self.call_service(
                WorkerGenerateStream,
                service_name,
                generate_request,
                timeout_sec=self.service_timeout
            )
            
            if generate_result is None:
                self.get_logger().error("Worker generate service - no result")
                return None
                
            if not generate_result.success:
                self.get_logger().error(f"Worker generate error: {generate_result.error_message}")
                return None
            
            # Extract generated text (remove prompt part)
            output = generate_result.text
            if output.startswith(prompt):
                output = output[len(prompt):].strip()
            
            return output
            
        except Exception as e:
            self.get_logger().error(f"Error querying model: {e}")
            return None

    def find_vectors(self, text):
        """Extract coordinate vectors from model response"""
        pattern = r"\(([-+]?\d+\.?\d*(?:,\s*[-+]?\d+\.?\d*)*?)\)"
        matches = re.findall(pattern, text)
        
        vectors = []
        for match in matches:
            try:
                vector = [float(num.strip()) if '.' in num else int(num.strip()) 
                         for num in match.split(',')]
                vectors.append(vector)
            except ValueError:
                continue
                
        return vectors

    def create_visualization(self, pil_image, affordance_points, cross_size=9, cross_width=4):
        """Create visualization of affordance points on image"""
        try:
            if not affordance_points:
                return None

            # Convert PIL to OpenCV
            cv_image = cv2.cvtColor(np.array(pil_image), cv2.COLOR_RGB2BGR)
            h, w = cv_image.shape[:2]

            # Find highest confidence point
            best_pt = max(affordance_points, key=lambda p: p.confidence)

            for pt in affordance_points:
                # Normalize → pixel
                x = int(pt.x * w) if pt.x <= 1.0 else int(pt.x)
                y = int(pt.y * h) if pt.y <= 1.0 else int(pt.y)

                # Best point in green, larger; others in red
                if pt is best_pt:
                    color = (0, 255, 0)
                    size  = int(cross_size * 1.5)
                    width = int(cross_width * 1.5)
                else:
                    color = (0, 0, 255)
                    size  = cross_size
                    width = cross_width

                # Draw cross
                cv2.line(cv_image, (x - size, y - size), (x + size, y + size), color, width)
                cv2.line(cv_image, (x - size, y + size), (x + size, y - size), color, width)

                # Show confidence near point
                cv2.putText(
                    cv_image,
                    f"{pt.confidence:.2f}",
                    (x + size + 2, y),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.4, color, 1
                )

            return cv_image

        except Exception as e:
            self.get_logger().error(f"Error creating visualization: {e}")
            return None

    def delayed_model_init(self):
        """Initialize models once and stop the timer"""
        if self.model_initialized:
            return
            
        self.get_logger().info("Attempting to get model list (delayed)...")
        self.models = self.get_model_list()
        if not self.model_name and self.models:
            self.model_name = self.models[0]
            self.get_logger().info(f"Model selected: {self.model_name}")
        elif self.models:
            self.get_logger().info(f"Using specified model: {self.model_name}")
        else:
            self.get_logger().warn("No models available after delayed init.")
            return
            
        # Mark as initialized and destroy the timer
        self.model_initialized = True
        self.model_init_timer.destroy()
        self.get_logger().info("Model initialization completed")

    def get_conv_log_filename(self):
        """Get conversation log filename"""
        t = datetime.datetime.now()
        name = os.path.join(LOGDIR, f"{t.year}-{t.month:02d}-{t.day:02d}-conv.json")
        return name


def main(args=None):
    rclpy.init(args=args)
    
    node = RoboPointNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()