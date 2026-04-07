import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from vision_msgs.msg import Detection2DArray, Detection2D
from custom_interfaces.msg import IntruderAlert, IntruderDetection # type: ignore
from cv_bridge import CvBridge
from transformers import CLIPProcessor, CLIPModel
from PIL import Image as PILImage
import torch
import cv2
import numpy as np
from datetime import datetime

class IntruderDetectionNode(Node):
    def __init__(self):
        super().__init__('intruder_detection_node')
        
        # Declare parameters
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('detection_topic', '/camera/detections')
        self.declare_parameter('image_type', 'raw')  # raw or compressed
        self.declare_parameter('clip_model', 'openai/clip-vit-base-patch32')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('device', 'auto')  # auto, cuda, cpu
        self.declare_parameter('visualize', True)
        self.declare_parameter('debug', True)
        
        # Get parameters
        self.image_topic = self.get_parameter('image_topic').value
        self.detection_topic = self.get_parameter('detection_topic').value
        self.image_type = self.get_parameter('image_type').value
        self.clip_model_name = self.get_parameter('clip_model').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').get_parameter_value().double_value
        self.visualize = self.get_parameter('visualize').value
        self.debug = self.get_parameter('debug').value
        
        # Set device
        device_param = self.get_parameter('device').value
        if device_param == 'auto':
            self.device = 'cuda:0' if torch.cuda.is_available() else 'cpu'
        else:
            self.device = device_param
        
        self.get_logger().info(f"Using device: {self.device}")
        self.get_logger().info(f"CLIP Model: {self.clip_model_name}")
        self.get_logger().info(f"Confidence Threshold: {self.confidence_threshold}")
        
        # Initialize CLIP model
        self.load_clip_model()
        
        # Define classification labels (non-intruders vs intruders)
        self.labels = [
            "a person in a white lab coat",      # Researcher/Not intruder
            "a student with casual clothes",      # Student/Not intruder
            "a professor with formal clothes",    # Professor/Not intruder
            "a person in military uniform",       # Intruder
            "a construction worker with helmet and safety vest",  # Intruder
            "a security guard in uniform",        # Could be intruder depending on context
            "a delivery person in uniform",       # Intruder
            "a stranger in casual clothes"        # Potential intruder
        ]
        
        # Define which labels are considered intruders
        self.intruder_labels = [
            "a person in military uniform",
            "a construction worker with helmet and safety vest",
            "a delivery person in uniform",
            "a stranger in casual clothes"
        ]
        
        # Define non-intruder labels (for logging purposes)
        self.non_intruder_labels = [
            "a person in a white lab coat",
            "a student with casual clothes",
            "a professor with formal clothes"
        ]
        
        # CvBridge for image conversion
        self.bridge = CvBridge()
        
        # Subscriptions
        if self.image_type == 'compressed':
            self.image_sub = self.create_subscription(
                CompressedImage,
                self.image_topic,
                self.image_callback_compressed,
                10
            )
        else:
            self.image_sub = self.create_subscription(
                Image,
                self.image_topic,
                self.image_callback_raw,
                10
            )
        
        # Subscribe to YOLO detections
        self.detection_sub = self.create_subscription(
            Detection2DArray,
            self.detection_topic,
            self.detection_callback,
            10
        )
        
        # Publishers
        self.intruder_alert_pub = self.create_publisher(
            IntruderAlert, 
            '/intruder/alerts', 
            10
        )
        self.intruder_detection_pub = self.create_publisher(
            IntruderDetection,
            '/intruder/detections',
            10
        )
        
        # Publisher for annotated intruder images
        if self.visualize:
            self.annotated_pub = self.create_publisher(
                Image,
                '/intruder/annotated_image',
                10
            )
        
        # Store latest image and detections
        self.latest_image = None
        self.latest_detections = None
        self.latest_image_header = None
        
        # Statistics
        self.intruder_count = 0
        self.total_people_detected = 0
        
        self.get_logger().info("Intruder Detection Node Started")
        self.get_logger().info(f"Monitoring for intruders among: {self.labels}")
    
    def load_clip_model(self):
        """Load CLIP model and processor"""
        try:
            self.get_logger().info(f"Loading CLIP model: {self.clip_model_name}")
            self.clip_model = CLIPModel.from_pretrained(self.clip_model_name).to(self.device)
            self.clip_processor = CLIPProcessor.from_pretrained(self.clip_model_name)
            self.get_logger().info("CLIP model loaded successfully")
        except Exception as e:
            self.get_logger().error(f"Failed to load CLIP model: {e}")
            raise
    
    def image_callback_raw(self, msg):
        """Store latest raw image"""
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.latest_image_header = msg.header
        except Exception as e:
            self.get_logger().error(f"Error converting raw image: {e}")
    
    def image_callback_compressed(self, msg):
        """Store latest compressed image"""
        try:
            self.latest_image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.latest_image_header = msg.header
        except Exception as e:
            self.get_logger().error(f"Error converting compressed image: {e}")
    
    def detection_callback(self, msg):
        """Process YOLO detections and classify intruders"""
        if self.latest_image is None:
            if self.debug:
                self.get_logger().warn("No image received yet")
            return
        
        # Process each person detection
        for detection in msg.detections:
            # Check if detection is a person (class_id '0' for YOLO person class)
            if detection.results:
                class_id = detection.results[0].hypothesis.class_id
                if class_id == '0':  # Person class in YOLO
                    self.total_people_detected += 1
                    self.process_person_detection(detection, msg.header)
    
    def process_person_detection(self, detection, header):
        """Extract person region and classify using CLIP"""
        try:
            # Get bounding box coordinates
            bbox = detection.bbox
            x_center = int(bbox.center.position.x)
            y_center = int(bbox.center.position.y)
            width = int(bbox.size_x)
            height = int(bbox.size_y)
            
            # Calculate corner coordinates
            x1 = max(0, x_center - width // 2)
            y1 = max(0, y_center - height // 2)
            x2 = min(self.latest_image.shape[1], x_center + width // 2)
            y2 = min(self.latest_image.shape[0], y_center + height // 2)
            
            # Crop person region
            person_roi = self.latest_image[y1:y2, x1:x2]
            
            if person_roi.size == 0:
                if self.debug:
                    self.get_logger().warn("Empty person ROI")
                return
            
            # Convert to PIL Image for CLIP
            person_pil = PILImage.fromarray(cv2.cvtColor(person_roi, cv2.COLOR_BGR2RGB))
            
            # Run CLIP classification
            inputs = self.clip_processor(
                text=self.labels, 
                images=person_pil, 
                return_tensors="pt", 
                padding=True
            ).to(self.device)
            
            with torch.no_grad():
                outputs = self.clip_model(**inputs)
                logits_per_image = outputs.logits_per_image
                probs = logits_per_image.softmax(dim=1).cpu().numpy()[0]
            
            # Get top prediction
            top_index = probs.argmax()
            top_label = self.labels[top_index]
            top_confidence = float(probs[top_index])
            
            # Check if this is an intruder
            is_intruder = top_label in self.intruder_labels
            
            # Only consider high confidence predictions
            if top_confidence >= self.confidence_threshold:
                if is_intruder:
                    self.intruder_count += 1
                    
                    # Create intruder alert message
                    alert_msg = IntruderAlert()
                    alert_msg.header = header
                    alert_msg.is_intruder = True
                    alert_msg.intruder_type = top_label
                    alert_msg.confidence = top_confidence
                    alert_msg.timestamp = self.get_clock().now().to_msg()
                    alert_msg.detection_id = self.intruder_count
                    
                    # Add bounding box information
                    alert_msg.bbox_center_x = float(x_center)
                    alert_msg.bbox_center_y = float(y_center)
                    alert_msg.bbox_width = float(width)
                    alert_msg.bbox_height = float(height)
                    
                    self.intruder_alert_pub.publish(alert_msg)
                    
                    # Create detailed detection message
                    detection_msg = IntruderDetection()
                    detection_msg.header = header
                    detection_msg.intruder_type = top_label
                    detection_msg.confidence = top_confidence
                    detection_msg.bbox_center_x = float(x_center)
                    detection_msg.bbox_center_y = float(y_center)
                    detection_msg.bbox_width = float(width)
                    detection_msg.bbox_height = float(height)
                    
                    # Add all classification scores
                    for label, score in zip(self.labels, probs):
                        detection_msg.all_scores.append(score)
                        detection_msg.all_labels.append(label)
                    
                    self.intruder_detection_pub.publish(detection_msg)
                    
                    self.get_logger().warn(
                        f"INTRUDER DETECTED! Type: {top_label}, "
                        f"Confidence: {top_confidence:.3f}, "
                        f"Location: ({x_center}, {y_center})"
                    )
                    
                    # Annotate image if visualization is enabled
                    if self.visualize:
                        self.annotate_intruder_image(person_roi, top_label, top_confidence, header)
                
                elif self.debug:
                    self.get_logger().info(
                        f"Person classified as: {top_label} "
                        f"(Confidence: {top_confidence:.3f}) - Not an intruder"
                    )
            
            # Log all classification scores for debugging
            if self.debug and top_confidence >= self.confidence_threshold:
                self.get_logger().debug("Classification scores:")
                for label, score in zip(self.labels, probs):
                    if score > 0.01:  # Only show meaningful scores
                        self.get_logger().debug(f"  {label}: {score:.3f}")
        
        except Exception as e:
            self.get_logger().error(f"Error processing person detection: {e}")
    
    def annotate_intruder_image(self, person_roi, intruder_type, confidence, header):
        """Create and publish annotated image of intruder"""
        try:
            # Create annotation on the person ROI
            annotated = person_roi.copy()
            
            # Add text overlay
            text = f"INTRUDER: {intruder_type} ({confidence:.2f})"
            cv2.putText(
                annotated,
                text,
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 0, 255),  # Red color for intruder
                2,
                cv2.LINE_AA
            )
            
            # Add timestamp
            timestamp_str = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            cv2.putText(
                annotated,
                timestamp_str,
                (10, annotated.shape[0] - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 255, 255),
                1,
                cv2.LINE_AA
            )
            
            # Add red border around intruder
            cv2.rectangle(annotated, (0, 0), (annotated.shape[1]-1, annotated.shape[0]-1), (0, 0, 255), 3)
            
            # Convert to ROS message and publish
            annotated_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
            annotated_msg.header = header
            self.annotated_pub.publish(annotated_msg)
            
        except Exception as e:
            self.get_logger().error(f"Error creating intruder annotation: {e}")
    
    def __del__(self):
        """Print statistics on node destruction"""
        if self.total_people_detected > 0:
            self.get_logger().info(
                f"Intruder Detection Statistics: "
                f"{self.intruder_count} intruders detected out of "
                f"{self.total_people_detected} people ({self.intruder_count/self.total_people_detected*100:.1f}%)"
            )

def main(args=None):
    rclpy.init(args=args)
    node = IntruderDetectionNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down intruder detection node")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()