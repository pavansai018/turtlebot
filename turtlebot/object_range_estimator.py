#!/usr/bin/env python3

import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from sensor_msgs.msg import LaserScan, CameraInfo
from vision_msgs.msg import Detection2DArray
from geometry_msgs.msg import PointStamped
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException
from tf2_geometry_msgs import do_transform_point


class ObjectRangeEstimator(Node):
    def __init__(self):
        super().__init__('object_range_estimator')

        # Topics
        self.declare_parameter('detections_topic', '/camera/detections')
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('camera_info_topic', '/camera/camera_info')

        # Frames
        self.declare_parameter('camera_frame', 'camera')      # optical frame used by image + detections
        self.declare_parameter('laser_frame', 'base_scan')    # fallback only
        self.declare_parameter('target_frame', 'base_link')   # output frame

        # Geometry / association
        self.declare_parameter('scan_window_half_width', 1)   # use ±N beams around projected center beam
        self.declare_parameter('max_scan_age_sec', 3.0)       # allow delayed detections
        self.declare_parameter('min_bbox_width', 8.0)
        self.declare_parameter('min_bbox_height', 8.0)

        # Small tuning offset if camera and lidar are not perfectly aligned in yaw
        self.declare_parameter('camera_yaw_offset_rad', 0.0)

        # Logging
        self.declare_parameter('debug', True)

        self.detections_topic = self.get_parameter('detections_topic').value
        self.scan_topic = self.get_parameter('scan_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value

        self.camera_frame_param = self.get_parameter('camera_frame').value
        self.laser_frame_param = self.get_parameter('laser_frame').value
        self.target_frame = self.get_parameter('target_frame').value

        self.scan_window_half_width = int(self.get_parameter('scan_window_half_width').value)
        self.max_scan_age_sec = float(self.get_parameter('max_scan_age_sec').value)
        self.min_bbox_width = float(self.get_parameter('min_bbox_width').value)
        self.min_bbox_height = float(self.get_parameter('min_bbox_height').value)
        self.camera_yaw_offset_rad = float(self.get_parameter('camera_yaw_offset_rad').value)
        self.debug = bool(self.get_parameter('debug').value)

        self.latest_scan = None
        self.latest_camera_info = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        scan_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.scan_sub = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scan_callback,
            scan_qos
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self.camera_info_callback,
            10
        )
        self.detections_sub = self.create_subscription(
            Detection2DArray,
            self.detections_topic,
            self.detections_callback,
            10
        )

        self.ranged_pub = self.create_publisher(
            Detection2DArray,
            '/camera/detections_ranged',
            10
        )

        self.get_logger().info(f"Detections topic: {self.detections_topic}")
        self.get_logger().info(f"Scan topic: {self.scan_topic}")
        self.get_logger().info(f"CameraInfo topic: {self.camera_info_topic}")
        self.get_logger().info(f"Camera frame: {self.camera_frame_param}")
        self.get_logger().info(f"Laser frame fallback: {self.laser_frame_param}")
        self.get_logger().info(f"Target frame: {self.target_frame}")
        self.get_logger().info(f"scan_window_half_width: {self.scan_window_half_width}")
        self.get_logger().info(f"camera_yaw_offset_rad: {self.camera_yaw_offset_rad:.4f}")
        self.get_logger().info("ObjectRangeEstimator started")

    def scan_callback(self, msg: LaserScan):
        self.latest_scan = msg

    def camera_info_callback(self, msg: CameraInfo):
        self.latest_camera_info = msg

    def detections_callback(self, msg: Detection2DArray):
        if self.latest_scan is None:
            self.get_logger().warn("No /scan yet")
            return

        if self.latest_camera_info is None:
            self.get_logger().warn("No /camera/camera_info yet")
            return

        det_time = self.to_sec(msg.header.stamp)
        scan_time = self.to_sec(self.latest_scan.header.stamp)
        dt = abs(det_time - scan_time) if det_time > 0.0 and scan_time > 0.0 else 0.0

        if self.debug and dt > self.max_scan_age_sec:
            self.get_logger().warn(f"Using stale scan/detection pair, dt={dt:.3f}s")

        out = Detection2DArray()
        out.header = msg.header
        out.header.frame_id = self.target_frame

        for det in msg.detections:
            try:
                ok, point_target, range_used, angle_used = self.estimate_detection_position(det, msg.header)
            except Exception as e:
                self.get_logger().warn(f"Failed detection range estimate: {str(e)}")
                continue

            if not ok:
                continue

            ranged_det = det
            ranged_det.header = det.header
            ranged_det.header.frame_id = self.target_frame

            if len(ranged_det.results) > 0:
                ranged_det.results[0].pose.pose.position.x = float(point_target.point.x)
                ranged_det.results[0].pose.pose.position.y = float(point_target.point.y)
                ranged_det.results[0].pose.pose.position.z = float(point_target.point.z)
                ranged_det.results[0].pose.pose.orientation.x = 0.0
                ranged_det.results[0].pose.pose.orientation.y = 0.0
                ranged_det.results[0].pose.pose.orientation.z = 0.0
                ranged_det.results[0].pose.pose.orientation.w = 1.0

                cov = [0.0] * 36
                cov[0] = 0.02
                cov[7] = 0.02
                cov[14] = 0.05
                ranged_det.results[0].pose.covariance = cov

            out.detections.append(ranged_det)

            if self.debug:
                cls_id = ranged_det.results[0].hypothesis.class_id if len(ranged_det.results) > 0 else "unknown"
                dist_xyz = math.sqrt(
                    point_target.point.x ** 2 +
                    point_target.point.y ** 2 +
                    point_target.point.z ** 2
                )
                dist_xy = math.sqrt(
                    point_target.point.x ** 2 +
                    point_target.point.y ** 2
                )
                self.get_logger().info(
                    f"class={cls_id} "
                    f"scan_range={range_used:.3f}m "
                    f"scan_angle={math.degrees(angle_used):.1f}deg "
                    f"{self.target_frame}_xyz=({point_target.point.x:.3f}, "
                    f"{point_target.point.y:.3f}, {point_target.point.z:.3f}) "
                    f"dist_xy={dist_xy:.3f}m dist_xyz={dist_xyz:.3f}m"
                )

        self.ranged_pub.publish(out)

    def estimate_detection_position(self, det, detections_header):
        if det.bbox.size_x < self.min_bbox_width or det.bbox.size_y < self.min_bbox_height:
            return False, None, None, None

        cam_info = self.latest_camera_info
        scan = self.latest_scan

        # Use detection header frame if present, else camera_info frame, else parameter
        camera_frame = (
            detections_header.frame_id
            if detections_header.frame_id
            else (cam_info.header.frame_id if cam_info.header.frame_id else self.camera_frame_param)
        )

        if not camera_frame:
            raise RuntimeError("No camera frame available")

        laser_frame = scan.header.frame_id if scan.header.frame_id else self.laser_frame_param
        if not laser_frame:
            raise RuntimeError("No laser frame available")

        # Use bbox center X only.
        # This is intentional because 2D lidar is horizontal; vertical pixel does not help.
        u = float(det.bbox.center.position.x)

        K = np.array(cam_info.k, dtype=np.float64).reshape(3, 3)
        fx = float(K[0, 0])
        cx = float(K[0, 2])

        if fx == 0.0:
            raise RuntimeError(f"Invalid focal length fx={fx}")

        # Horizontal normalized coordinate in optical frame
        x_n = (u - cx) / fx

        if not np.isfinite(x_n):
            raise RuntimeError(f"Invalid normalized x_n={x_n}")

        # Optical frame ray: +z forward, +x right, +y down
        # Since lidar is horizontal, ignore image vertical component here.
        ray_cam = np.array([x_n, 0.0, 1.0], dtype=np.float64)
        norm = np.linalg.norm(ray_cam)
        if norm == 0.0 or not np.isfinite(norm):
            raise RuntimeError(f"Invalid ray norm={norm}")
        ray_cam = ray_cam / norm

        stamp = detections_header.stamp if self.to_sec(detections_header.stamp) > 0.0 else scan.header.stamp

        p0_cam = PointStamped()
        p0_cam.header.frame_id = camera_frame
        p0_cam.header.stamp = stamp
        p0_cam.point.x = 0.0
        p0_cam.point.y = 0.0
        p0_cam.point.z = 0.0

        p1_cam = PointStamped()
        p1_cam.header.frame_id = camera_frame
        p1_cam.header.stamp = stamp
        p1_cam.point.x = float(ray_cam[0])
        p1_cam.point.y = float(ray_cam[1])
        p1_cam.point.z = float(ray_cam[2])

        tf_cam_to_laser = self.tf_buffer.lookup_transform(
            laser_frame,
            camera_frame,
            rclpy.time.Time()
        )

        p0_laser = do_transform_point(p0_cam, tf_cam_to_laser)
        p1_laser = do_transform_point(p1_cam, tf_cam_to_laser)

        dx = p1_laser.point.x - p0_laser.point.x
        dy = p1_laser.point.y - p0_laser.point.y

        if not np.isfinite(dx) or not np.isfinite(dy):
            raise RuntimeError(f"Invalid transformed ray dx={dx}, dy={dy}")

        angle_laser = math.atan2(dy, dx) + self.camera_yaw_offset_rad

        if not np.isfinite(angle_laser):
            raise RuntimeError(f"Invalid angle_laser={angle_laser}")

        if not np.isfinite(scan.angle_min) or not np.isfinite(scan.angle_increment):
            raise RuntimeError("Invalid scan angle metadata")
        if scan.angle_increment == 0.0:
            raise RuntimeError("scan.angle_increment is zero")

        # Normalize to scan convention
        angle_used = self.normalize_to_scan(angle_laser, scan)

        center_idx = int(round((angle_used - scan.angle_min) / scan.angle_increment))

        if center_idx < 0 or center_idx >= len(scan.ranges):
            raise RuntimeError(
                f"Projected angle outside scan FOV: angle={angle_used}, idx={center_idx}, "
                f"angle_min={scan.angle_min}, inc={scan.angle_increment}, n={len(scan.ranges)}"
            )

        # Search a small neighborhood and take nearest valid hit
        start_idx = max(0, center_idx - self.scan_window_half_width)
        end_idx = min(len(scan.ranges) - 1, center_idx + self.scan_window_half_width)

        best_idx = None
        best_range = None

        for i in range(start_idx, end_idx + 1):
            r = scan.ranges[i]
            if math.isfinite(r) and (scan.range_min <= r <= scan.range_max):
                if best_range is None or r < best_range:
                    best_range = float(r)
                    best_idx = i

        if best_range is None:
            raise RuntimeError("No valid lidar range near projected index")

        range_used = best_range
        angle_used = scan.angle_min + best_idx * scan.angle_increment

        obj_laser = PointStamped()
        obj_laser.header.frame_id = laser_frame
        obj_laser.header.stamp = scan.header.stamp
        obj_laser.point.x = range_used * math.cos(angle_used)
        obj_laser.point.y = range_used * math.sin(angle_used)
        obj_laser.point.z = 0.0

        tf_laser_to_target = self.tf_buffer.lookup_transform(
            self.target_frame,
            laser_frame,
            rclpy.time.Time()
        )

        obj_target = do_transform_point(obj_laser, tf_laser_to_target)

        return True, obj_target, range_used, angle_used

    @staticmethod
    def normalize_to_scan(angle, scan):
        scan_span = scan.angle_increment * len(scan.ranges)
        while angle < scan.angle_min:
            angle += 2.0 * math.pi
        while angle >= scan.angle_min + scan_span:
            angle -= 2.0 * math.pi
        return angle

    @staticmethod
    def to_sec(stamp):
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def main(args=None):
    rclpy.init(args=args)
    node = ObjectRangeEstimator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except (LookupException, ConnectivityException, ExtrapolationException) as e:
        node.get_logger().error(f"TF error: {str(e)}")
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()