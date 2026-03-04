#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose, BoundingBox2D, Pose2D
from cv_bridge import CvBridge
from ultralytics import YOLO
import torch
import cv2

class YoloDetectionNode(Node):
    def __init__(self):
        super().__init__('yolo_detection_node')
        
        # Subscriptions
        self.subscription = self.create_subscription(
            Image,
            '/camera/image',
            self.image_callback,
            10
        )
        
        # Publishers
        self.detections_pub = self.create_publisher(Detection2DArray, '/camera/detections', 10)
        self.image_pub = self.create_publisher(Image, '/camera/image_annotated', 10)
        
        # CvBridge for converting ROS Image <-> OpenCV
        self.bridge = CvBridge()
        
        self.get_logger().info("YOLOv8 Detection Node Started")

        # Load YOLOv8 model
        self.model = YOLO("yolov8s.pt")
        self.model.to('cuda' if torch.cuda.is_available() else 'cpu')

    def image_callback(self, msg):
        # Convert ROS Image to OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        
        # Run YOLOv8 inference
        results = self.model.predict(cv_image, imgsz=640, device=0, verbose=False)
        
        # Prepare Detection2DArray message
        detections_msg = Detection2DArray()
        detections_msg.header = msg.header
        
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
                hypothesis.hypothesis.score = float(conf)
                detection.results.append(hypothesis)
                detections_msg.detections.append(detection)
                
                # Draw bounding box on annotated image
                cv2.rectangle(annotated_image, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
                label = f"{cls_id}:{conf:.2f}"
                cv2.putText(annotated_image, label, (int(x1), int(y1)-5),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        
        # Publish detections
        self.detections_pub.publish(detections_msg)
        
        # Publish annotated image
        annotated_msg = self.bridge.cv2_to_imgmsg(annotated_image, encoding='bgr8')
        annotated_msg.header = msg.header
        self.image_pub.publish(annotated_msg)

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