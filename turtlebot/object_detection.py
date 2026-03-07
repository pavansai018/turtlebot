import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose, BoundingBox2D, Pose2D
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory
from ultralytics import YOLO
import torch
import cv2
import os
import random
import time

class YoloDetectionNode(Node):
    def __init__(self):
        super().__init__('yolo_detection_node')
        # Declare a ROS parameter
        self.declare_parameter('rate_limit', 1.0)
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('image_type', 'raw') # raw or compressed

        # Get Parameter value
        self.rate_limit = self.get_parameter('rate_limit').get_parameter_value().double_value
        self.image_topic = self.get_parameter('image_topic').value
        self.image_type = self.get_parameter('image_type').value

        self.get_logger().info(f"Detection Rate Limit: {self.rate_limit:.2f}")
        self.get_logger().info(f"Image Topic: {self.image_topic}")
        self.get_logger().info(f"Image Type: {self.image_type}")

        self.last_detection_time = time.time()
        self.detection_interval = 1.0/self.rate_limit if self.rate_limit > 0 else 0 # minimum time between detections

        # Subscriptions
        if self.image_type == 'compressed':
            self.subscription = self.create_subscription(
                CompressedImage,
                self.image_topic,
                self.compressed_image_callback,
                10
            )
        else:
            self.subscription = self.create_subscription(
                Image,
                self.image_topic,
                self.raw_image_callback,
                10
            )
        
        # Publishers
        self.detections_pub = self.create_publisher(Detection2DArray, '/camera/detections', 10)
        self.image_pub = self.create_publisher(Image, '/camera/image_annotated', 10)
        
        # CvBridge for converting ROS Image <-> OpenCV
        self.bridge = CvBridge()
        
        self.get_logger().info("YOLOv8 Detection Node Started")

        # Load YOLOv8 model
        pkg_dir = get_package_share_directory('turtlebot')
        # Full path to model file (Within installation directory)
        self.model_path = os.path.join(pkg_dir, 'model_weights', 'yolov8l.pt')
        self.model = YOLO(self.model_path)
        self.device = 'cuda:0' if torch.cuda.is_available() else 'cpu'
        self.model.to(self.device)

        self.class_colors = self.generate_class_colors(self.model.names)

    def generate_class_colors(self, class_names):
        """
        Generate a fixed unique BGR color for each class.
        Using a fixed seed makes colors consistent across runs.
        """
        random.seed(42)
        colors = {}
        for cls_id in class_names.keys():
            colors[cls_id] = (
                random.randint(0, 255),  # Blue
                random.randint(0, 255),  # Green
                random.randint(0, 255)   # Red
            )
        return colors
    
    def raw_image_callback(self, msg):
        # Control detection frame rate
        if (self.detection_interval > (time.time() - self.last_detection_time)):
            # If a detection_interval has not yet passed since last detection, ignore image
            return
        self.last_detection_time = time.time()
        # Convert ROS Image to OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.process_image(cv_image, msg.header)

    def compressed_image_callback(self, msg):
         # Control detection frame rate
        if (self.detection_interval > (time.time() - self.last_detection_time)):
            # If a detection_interval has not yet passed since last detection, ignore image
            return
        self.last_detection_time = time.time()
        # Convert ROS Image to OpenCV
        cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.process_image(cv_image, msg.header)

    def process_image(self, cv_image, header):
        # Run YOLOv8 inference
        results = self.model.predict(cv_image, imgsz=640, device=self.device, verbose=False)
        
        # Prepare Detection2DArray message
        detections_msg = Detection2DArray()
        detections_msg.header = header
        
        # Annotate image
        annotated_image = cv_image.copy()
        
        for r in results:
            boxes = r.boxes
            for box in boxes:
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                conf = float(box.conf[0].cpu().numpy())
                cls_id = int(box.cls[0].cpu().numpy())
                
                # Fill Detection2D message
                detection = Detection2D()
                detection.bbox.center.position.x = float((x1 + x2) / 2)
                detection.bbox.center.position.y = float((y1 + y2) / 2)
                detection.bbox.size_x = float(x2 - x1)
                detection.bbox.size_y = float(y2 - y1)

                hypothesis = ObjectHypothesisWithPose()
                hypothesis.hypothesis.class_id = str(cls_id)
                label_name = self.model.names[cls_id]  # get human-readable label
                hypothesis.hypothesis.score = float(conf)
                detection.results.append(hypothesis)
                detections_msg.detections.append(detection)

                color = self.class_colors[cls_id]
                
                # Draw bounding box on annotated image
                cv2.rectangle(annotated_image, (int(x1), int(y1)), (int(x2), int(y2)), color, 2)
                # Draw label background for readability
                label = f"{label_name}:{conf:.2f}"
                (text_w, text_h), baseline = cv2.getTextSize(
                    label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1
                )
                cv2.rectangle(
                    annotated_image,
                    (int(x1), int(y1) - text_h - 8),
                    (int(x1) + text_w, int(y1)),
                    color,
                    -1
                )
                cv2.putText(
                    annotated_image,
                    label,
                    (int(x1), int(y1) - 5),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (255, 255, 255),
                    1
                )
        
        # Publish detections
        self.detections_pub.publish(detections_msg)
        
        # Publish annotated image
        annotated_msg = self.bridge.cv2_to_imgmsg(annotated_image, encoding='bgr8')
        annotated_msg.header = header
        self.image_pub.publish(annotated_msg)
        # Check execution time
        if (self.detection_interval < (time.time() -self.last_detection_time)):
            # The detection process itself took a longer time than allocated detection_interval
            # The hardware is not capable enough to run detection at given rate
            self.get_logger().warn(
                "Possible : Rate limit too high! Should be less than : %.2f Hz"%(
                    (1/(time.time()-self.last_detection_time))))
        # Log info
        num_detections = len(detections_msg.detections)
        det_labels = [self.model.names[int(d.results[0].hypothesis.class_id)] for d in detections_msg.detections]
        self.get_logger().info(f"Published {num_detections} detections: {det_labels}")

def main(args=None):
    rclpy.init(args=args)
    node = YoloDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()