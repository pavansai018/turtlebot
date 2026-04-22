#!/usr/bin/env python3

from typing import Optional

import cv2
import mediapipe as mp
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import CompressedImage
from geometry_msgs.msg import Twist, Point
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Duration


class HandFollowerNode(Node):
    def __init__(self) -> None:
        super().__init__('hand_follower_node')

        # -----------------------------
        # Parameters
        # -----------------------------
        self.declare_parameter('image_topic', '/camera/image_raw/compressed')
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('marker_topic', '/hand_markers')
        self.declare_parameter('annotated_topic', '/hand_annotated/compressed')
        self.declare_parameter('camera_frame', 'camera_link')

        self.declare_parameter('linear_speed', 0.20)
        self.declare_parameter('publish_rate_hz', 10.0)

        self.declare_parameter('open_frames_required', 4)
        self.declare_parameter('fist_frames_required', 2)
        self.declare_parameter('unknown_frames_to_stop', 8)

        self.declare_parameter('min_detection_confidence', 0.65)
        self.declare_parameter('min_tracking_confidence', 0.65)
        self.declare_parameter('max_num_hands', 1)

        image_topic = self.get_parameter('image_topic').value
        cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        marker_topic = self.get_parameter('marker_topic').value
        annotated_topic = self.get_parameter('annotated_topic').value
        self.camera_frame = self.get_parameter('camera_frame').value

        self.linear_speed = float(self.get_parameter('linear_speed').value)
        self.publish_rate_hz = float(self.get_parameter('publish_rate_hz').value)

        self.open_frames_required = int(self.get_parameter('open_frames_required').value)
        self.fist_frames_required = int(self.get_parameter('fist_frames_required').value)
        self.unknown_frames_to_stop = int(self.get_parameter('unknown_frames_to_stop').value)

        min_detection_confidence = float(self.get_parameter('min_detection_confidence').value)
        min_tracking_confidence = float(self.get_parameter('min_tracking_confidence').value)
        max_num_hands = int(self.get_parameter('max_num_hands').value)

        # -----------------------------
        # ROS interfaces
        # -----------------------------
        self.image_sub = self.create_subscription(
            CompressedImage,
            image_topic,
            self.image_callback,
            10
        )
        self.cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)
        self.marker_pub = self.create_publisher(MarkerArray, marker_topic, 10)
        self.annotated_pub = self.create_publisher(CompressedImage, annotated_topic, 10)

        # Fixed-rate cmd_vel publisher
        timer_period = 1.0 / max(self.publish_rate_hz, 1.0)
        self.cmd_timer = self.create_timer(timer_period, self.cmd_timer_callback)

        # -----------------------------
        # MediaPipe
        # -----------------------------
        self.mp_hands = mp.solutions.hands
        self.mp_draw = mp.solutions.drawing_utils
        self.hands = self.mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=max_num_hands,
            min_detection_confidence=min_detection_confidence,
            min_tracking_confidence=min_tracking_confidence
        )

        # -----------------------------
        # State
        # -----------------------------
        self.current_linear = 0.0
        self.current_angular = 0.0

        self.last_raw_gesture = 'NO_HAND'
        self.same_gesture_count = 0
        self.unknown_count = 0
        self.active_state = 'STOP'   # FORWARD / STOP

        self.last_stamp = None
        self.last_debug_msg = ''

        self.get_logger().info('HandFollowerNode started.')

    # =========================================================
    # Fixed rate cmd_vel publish
    # =========================================================
    def cmd_timer_callback(self) -> None:
        msg = Twist()
        msg.linear.x = float(self.current_linear)
        msg.angular.z = float(self.current_angular)
        self.cmd_pub.publish(msg)

    # =========================================================
    # Image callback
    # =========================================================
    def image_callback(self, msg: CompressedImage) -> None:
        frame = self.compressed_to_bgr(msg)
        if frame is None:
            self.get_logger().warn('Failed to decode compressed image.')
            return

        self.last_stamp = msg.header.stamp
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.hands.process(rgb)

        raw_gesture = 'NO_HAND'
        marker_array = self.make_delete_all_marker_array()

        if results.multi_hand_landmarks and results.multi_handedness:
            hand_landmarks = results.multi_hand_landmarks[0]
            handedness = results.multi_handedness[0].classification[0].label

            self.mp_draw.draw_landmarks(
                frame,
                hand_landmarks,
                self.mp_hands.HAND_CONNECTIONS
            )

            raw_gesture = self.classify_gesture(hand_landmarks, handedness)

            marker_array.markers.append(
                self.create_landmark_marker(hand_landmarks, marker_id=1)
            )
            marker_array.markers.append(
                self.create_connection_marker(hand_landmarks, marker_id=2)
            )

        self.marker_pub.publish(marker_array)

        self.update_state_machine(raw_gesture)

        overlay_text_1 = f'Raw Gesture: {raw_gesture}'
        overlay_text_2 = f'State: {self.active_state}'
        overlay_text_3 = f'Cmd: x={self.current_linear:.2f}, z={self.current_angular:.2f}'
        overlay_text_4 = f'Count: {self.same_gesture_count}, Unknown: {self.unknown_count}'

        cv2.putText(frame, overlay_text_1, (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2, cv2.LINE_AA)
        cv2.putText(frame, overlay_text_2, (20, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2, cv2.LINE_AA)
        cv2.putText(frame, overlay_text_3, (20, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2, cv2.LINE_AA)
        cv2.putText(frame, overlay_text_4, (20, 120), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (200, 200, 255), 2, cv2.LINE_AA)

        annotated_msg = self.bgr_to_compressed(frame, msg.header.stamp)
        self.annotated_pub.publish(annotated_msg)

    # =========================================================
    # Gesture state machine
    # =========================================================
    def update_state_machine(self, raw_gesture: str) -> None:
        if raw_gesture == self.last_raw_gesture:
            self.same_gesture_count += 1
        else:
            self.last_raw_gesture = raw_gesture
            self.same_gesture_count = 1

        if raw_gesture in ['UNKNOWN', 'NO_HAND']:
            self.unknown_count += 1
        else:
            self.unknown_count = 0

        prev_state = self.active_state

        # Forward only after enough stable OPEN frames
        if raw_gesture == 'OPEN' and self.same_gesture_count >= self.open_frames_required:
            self.active_state = 'FORWARD'
            self.current_linear = self.linear_speed
            self.current_angular = 0.0

        # Stop quickly on stable FIST
        elif raw_gesture == 'FIST' and self.same_gesture_count >= self.fist_frames_required:
            self.active_state = 'STOP'
            self.current_linear = 0.0
            self.current_angular = 0.0

        # Stop if long uncertainty / no hand
        elif self.unknown_count >= self.unknown_frames_to_stop:
            self.active_state = 'STOP'
            self.current_linear = 0.0
            self.current_angular = 0.0

        # No else: keep previous command during short uncertainty

        debug_msg = (
            f'raw={raw_gesture}, same_count={self.same_gesture_count}, '
            f'unknown_count={self.unknown_count}, state={self.active_state}, '
            f'cmd=({self.current_linear:.2f}, {self.current_angular:.2f})'
        )

        if debug_msg != self.last_debug_msg or prev_state != self.active_state:
            self.get_logger().info(debug_msg)
            self.last_debug_msg = debug_msg

    # =========================================================
    # Gesture classification
    # =========================================================
    def classify_gesture(self, hand_landmarks, handedness: str) -> str:
        lm = hand_landmarks.landmark

        # Non-thumb fingers
        fingers_up = 0
        for tip_id, pip_id in [(8, 6), (12, 10), (16, 14), (20, 18)]:
            if lm[tip_id].y < lm[pip_id].y:
                fingers_up += 1

        # Thumb
        thumb_tip = lm[4]
        thumb_ip = lm[3]

        thumb_open = False
        if handedness == 'Right':
            thumb_open = thumb_tip.x < thumb_ip.x
        else:
            thumb_open = thumb_tip.x > thumb_ip.x

        if thumb_open:
            fingers_up += 1

        # Additional palm openness sanity check
        # Fist generally has fingertips close to wrist
        wrist = lm[0]
        fingertip_ids = [4, 8, 12, 16, 20]
        avg_tip_dist = np.mean([
            np.hypot(lm[i].x - wrist.x, lm[i].y - wrist.y) for i in fingertip_ids
        ])

        # Heuristic thresholds
        if fingers_up >= 4 and avg_tip_dist > 0.25:
            return 'OPEN'
        elif fingers_up <= 1 and avg_tip_dist < 0.22:
            return 'FIST'
        else:
            return 'UNKNOWN'

    # =========================================================
    # Marker publishing
    # =========================================================
    def make_delete_all_marker_array(self) -> MarkerArray:
        arr = MarkerArray()
        delete_marker = Marker()
        delete_marker.header.stamp = self.get_clock().now().to_msg()
        delete_marker.header.frame_id = self.camera_frame
        delete_marker.ns = 'hand'
        delete_marker.id = 0
        delete_marker.action = Marker.DELETEALL
        arr.markers.append(delete_marker)
        return arr

    def create_landmark_marker(self, hand_landmarks, marker_id: int) -> Marker:
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = self.camera_frame
        marker.ns = 'hand_landmarks'
        marker.id = marker_id
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.015
        marker.scale.y = 0.015
        marker.scale.z = 0.015
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.lifetime = Duration(sec=0, nanosec=300000000)

        for lm in hand_landmarks.landmark:
            p = Point()
            p.x = float(lm.x)
            p.y = float(1.0 - lm.y)
            p.z = 0.0
            marker.points.append(p)

        return marker

    def create_connection_marker(self, hand_landmarks, marker_id: int) -> Marker:
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = self.camera_frame
        marker.ns = 'hand_connections'
        marker.id = marker_id
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.004
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.5
        marker.color.b = 0.0
        marker.lifetime = Duration(sec=0, nanosec=300000000)

        for start_idx, end_idx in self.mp_hands.HAND_CONNECTIONS:
            p1 = Point()
            p1.x = float(hand_landmarks.landmark[start_idx].x)
            p1.y = float(1.0 - hand_landmarks.landmark[start_idx].y)
            p1.z = 0.0

            p2 = Point()
            p2.x = float(hand_landmarks.landmark[end_idx].x)
            p2.y = float(1.0 - hand_landmarks.landmark[end_idx].y)
            p2.z = 0.0

            marker.points.append(p1)
            marker.points.append(p2)

        return marker

    # =========================================================
    # Image conversion
    # =========================================================
    def compressed_to_bgr(self, msg: CompressedImage) -> Optional[np.ndarray]:
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            return img
        except Exception as e:
            self.get_logger().error(f'compressed_to_bgr failed: {e}')
            return None

    def bgr_to_compressed(self, frame: np.ndarray, stamp) -> CompressedImage:
        msg = CompressedImage()
        msg.header.stamp = stamp
        msg.header.frame_id = self.camera_frame
        msg.format = 'jpeg'

        ok, enc = cv2.imencode('.jpg', frame)
        if ok:
            msg.data = enc.tobytes()
        return msg


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HandFollowerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Force stop once on exit
        stop_msg = Twist()
        node.cmd_pub.publish(stop_msg)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()