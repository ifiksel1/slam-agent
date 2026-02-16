#!/usr/bin/env python3
"""
SLAM Vision Bridge for MAVROS (ROS 2 Humble)

Mirrors the approach from vision_to_mavros.cpp:
- Primary mode: TF2 lookup (map -> base_link) with rate-limited output
- Fallback mode: Subscribe to /Odometry topic
- Applies empirically validated orientation correction (180° Y similarity transform)

Topic Mapping:
  TF Input:  map -> base_link (via TF2 buffer)
  Odom Fallback: /Odometry (nav_msgs/msg/Odometry) from FAST-LIO
  Output: /mavros/vision_pose/pose (geometry_msgs/msg/PoseStamped) to MAVROS

The vision_pose plugin in MAVROS converts this to VISION_POSITION_ESTIMATE
MAVLink messages for ArduPilot's EKF3 fusion.
"""

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformListener, Buffer, LookupException, ConnectivityException, ExtrapolationException
import math


def quaternion_multiply(q1, q2):
    """
    Multiply two quaternions: q1 * q2

    Args:
        q1, q2: tuples (w, x, y, z)

    Returns:
        tuple (w, x, y, z) of the product quaternion
    """
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2

    w = w1*w2 - x1*x2 - y1*y2 - z1*z2
    x = w1*x2 + x1*w2 + y1*z2 - z1*y2
    y = w1*y2 - x1*z2 + y1*w2 + z1*x2
    z = w1*z2 + x1*y2 - y1*x2 + z1*w2

    return (w, x, y, z)


def quaternion_from_euler(roll, pitch, yaw):
    """
    Convert Euler angles (roll, pitch, yaw) to quaternion.

    Args:
        roll, pitch, yaw: angles in radians

    Returns:
        tuple (w, x, y, z)
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy

    return (w, x, y, z)


def normalize_quaternion(q):
    """Normalize a quaternion."""
    w, x, y, z = q
    norm = math.sqrt(w*w + x*x + y*y + z*z)
    if norm < 1e-10:
        return (1.0, 0.0, 0.0, 0.0)
    return (w/norm, x/norm, y/norm, z/norm)


class VisionBridge(Node):
    def __init__(self):
        super().__init__('vision_bridge')

        # Declare parameters matching the C++ implementation
        self.declare_parameter('use_tf', True)
        self.declare_parameter('output_rate', 30.0)
        self.declare_parameter('target_frame_id', 'map')
        self.declare_parameter('source_frame_id', 'base_link')
        self.declare_parameter('input_odom_topic', '/Odometry')
        self.declare_parameter('gamma_world', 0.0)
        self.declare_parameter('roll_cam', 0.0)
        self.declare_parameter('pitch_cam', 0.0)
        self.declare_parameter('yaw_cam', 0.0)
        self.declare_parameter('apply_y180_correction', True)

        # Read parameters
        self.use_tf = self.get_parameter('use_tf').value
        self.output_rate = self.get_parameter('output_rate').value
        self.target_frame_id = self.get_parameter('target_frame_id').value
        self.source_frame_id = self.get_parameter('source_frame_id').value
        self.input_odom_topic = self.get_parameter('input_odom_topic').value
        self.gamma_world = self.get_parameter('gamma_world').value
        self.roll_cam = self.get_parameter('roll_cam').value
        self.pitch_cam = self.get_parameter('pitch_cam').value
        self.yaw_cam = self.get_parameter('yaw_cam').value
        self.apply_y180_correction = self.get_parameter('apply_y180_correction').value

        # Log configuration
        self.get_logger().info('=== Vision Bridge Configuration ===')
        self.get_logger().info(f'  use_tf: {self.use_tf}')
        self.get_logger().info(f'  output_rate: {self.output_rate} Hz')
        self.get_logger().info(f'  target_frame_id: {self.target_frame_id}')
        self.get_logger().info(f'  source_frame_id: {self.source_frame_id}')
        self.get_logger().info(f'  input_odom_topic: {self.input_odom_topic}')
        self.get_logger().info(f'  gamma_world: {self.gamma_world} rad')
        self.get_logger().info(f'  roll_cam: {self.roll_cam} rad')
        self.get_logger().info(f'  pitch_cam: {self.pitch_cam} rad')
        self.get_logger().info(f'  yaw_cam: {self.yaw_cam} rad')
        self.get_logger().info(f'  apply_y180_correction: {self.apply_y180_correction}')

        # TF2 buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Latest transform from odometry fallback mode
        self.latest_odom_transform = None
        self.latest_odom_stamp = None

        # Subscribe to odometry fallback (always active, only used if use_tf=False)
        self.odom_sub = self.create_subscription(
            Odometry,
            self.input_odom_topic,
            self.odometry_callback,
            10
        )

        # Publish to MAVROS vision pose
        self.vision_pub = self.create_publisher(
            PoseStamped,
            '/mavros/vision_pose/pose',
            10
        )

        # Rate-limited timer for output
        self.timer_period = 1.0 / self.output_rate
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        # State tracking
        self.first_message = True
        self.message_count = 0
        self.last_transform_time = None

        # Pre-compute correction quaternions
        # 1) Camera-to-body corrections
        self.q_cam_to_body_x = quaternion_from_euler(self.roll_cam, 0.0, 0.0)
        self.q_cam_to_body_y = quaternion_from_euler(0.0, self.pitch_cam, 0.0)
        self.q_cam_to_body_z = quaternion_from_euler(0.0, 0.0, self.yaw_cam)

        # 2) World frame rotation (gamma_world)
        self.q_rot_z = quaternion_from_euler(0.0, 0.0, -self.gamma_world)

        # 3) 180° Y-axis correction (similarity transform: w, -x, y, -z)
        # This is q_180y * q * q_180y_inv which simplifies to component negation
        # We'll apply this as a final step if enabled

        self.get_logger().info('Vision bridge started')
        if self.use_tf:
            self.get_logger().info(f'  Mode: TF lookup ({self.target_frame_id} -> {self.source_frame_id})')
        else:
            self.get_logger().info(f'  Mode: Odometry subscription ({self.input_odom_topic})')
        self.get_logger().info('  Publishing to: /mavros/vision_pose/pose')

    def odometry_callback(self, msg):
        """
        Store latest odometry message for fallback mode.
        """
        # Convert Odometry to a transform-like structure
        self.latest_odom_stamp = msg.header.stamp
        self.latest_odom_transform = {
            'translation': {
                'x': msg.pose.pose.position.x,
                'y': msg.pose.pose.position.y,
                'z': msg.pose.pose.position.z
            },
            'rotation': {
                'x': msg.pose.pose.orientation.x,
                'y': msg.pose.pose.orientation.y,
                'z': msg.pose.pose.orientation.z,
                'w': msg.pose.pose.orientation.w
            }
        }

    def apply_position_rotation(self, x, y, z):
        """
        Apply gamma_world rotation to position (rotation around Z axis).

        Returns:
            tuple (x_new, y_new, z_new)
        """
        cos_g = math.cos(self.gamma_world)
        sin_g = math.sin(self.gamma_world)

        x_new = cos_g * x + sin_g * y
        y_new = -sin_g * x + cos_g * y
        z_new = z

        return (x_new, y_new, z_new)

    def apply_orientation_corrections(self, q_orig):
        """
        Apply all orientation corrections in the correct order:
        1. Camera-to-body corrections (roll, pitch, yaw)
        2. World frame rotation (-gamma_world)
        3. 180° Y-axis similarity transform (if enabled)

        Args:
            q_orig: tuple (w, x, y, z) original quaternion

        Returns:
            tuple (w, x, y, z) corrected quaternion
        """
        w, x, y, z = q_orig
        q_cam = (w, x, y, z)

        # Apply camera-to-body corrections
        q_body = quaternion_multiply(q_cam, self.q_cam_to_body_x)
        q_body = quaternion_multiply(q_body, self.q_cam_to_body_y)
        q_body = quaternion_multiply(q_body, self.q_cam_to_body_z)

        # Apply world frame rotation
        q_body = quaternion_multiply(self.q_rot_z, q_body)

        # Normalize
        q_body = normalize_quaternion(q_body)

        # Apply 180° Y-axis correction (similarity transform)
        # Mathematically: q_180y * q * q_180y_inv = (w, -x, y, -z)
        if self.apply_y180_correction:
            w, x, y, z = q_body
            q_body = (w, -x, y, -z)

        return q_body

    def timer_callback(self):
        """
        Rate-limited callback that publishes vision pose.
        Mirrors the ros::Rate loop in the C++ implementation.
        """
        try:
            if self.use_tf:
                # TF lookup mode
                transform = self.tf_buffer.lookup_transform(
                    self.target_frame_id,
                    self.source_frame_id,
                    rclpy.time.Time(),  # Time(0) means latest available
                    timeout=Duration(seconds=0.1)
                )

                # Check if this is a new transform
                transform_stamp = transform.header.stamp
                if self.last_transform_time is not None:
                    if transform_stamp.sec == self.last_transform_time.sec and \
                       transform_stamp.nanosec == self.last_transform_time.nanosec:
                        # Same transform as last time, skip
                        return

                self.last_transform_time = transform_stamp

                # Extract position
                pos_orig_x = transform.transform.translation.x
                pos_orig_y = transform.transform.translation.y
                pos_orig_z = transform.transform.translation.z

                # Extract orientation
                q_orig = (
                    transform.transform.rotation.w,
                    transform.transform.rotation.x,
                    transform.transform.rotation.y,
                    transform.transform.rotation.z
                )

                stamp = transform.header.stamp

            else:
                # Odometry subscription fallback mode
                if self.latest_odom_transform is None:
                    return

                # Check if this is a new message
                if self.last_transform_time is not None:
                    if self.latest_odom_stamp.sec == self.last_transform_time.sec and \
                       self.latest_odom_stamp.nanosec == self.last_transform_time.nanosec:
                        # Same message as last time, skip
                        return

                self.last_transform_time = self.latest_odom_stamp

                # Extract position
                pos_orig_x = self.latest_odom_transform['translation']['x']
                pos_orig_y = self.latest_odom_transform['translation']['y']
                pos_orig_z = self.latest_odom_transform['translation']['z']

                # Extract orientation
                q_orig = (
                    self.latest_odom_transform['rotation']['w'],
                    self.latest_odom_transform['rotation']['x'],
                    self.latest_odom_transform['rotation']['y'],
                    self.latest_odom_transform['rotation']['z']
                )

                stamp = self.latest_odom_stamp

            # Log first message arrival
            if self.first_message:
                self.get_logger().info('First transform received, publishing vision pose')
                self.first_message = False

            # Apply position rotation (gamma_world)
            pos_body_x, pos_body_y, pos_body_z = self.apply_position_rotation(
                pos_orig_x, pos_orig_y, pos_orig_z
            )

            # Apply orientation corrections
            q_body_w, q_body_x, q_body_y, q_body_z = self.apply_orientation_corrections(q_orig)

            # Create PoseStamped message
            pose_msg = PoseStamped()
            pose_msg.header.stamp = stamp
            pose_msg.header.frame_id = self.target_frame_id

            pose_msg.pose.position.x = pos_body_x
            pose_msg.pose.position.y = pos_body_y
            pose_msg.pose.position.z = pos_body_z

            pose_msg.pose.orientation.w = q_body_w
            pose_msg.pose.orientation.x = q_body_x
            pose_msg.pose.orientation.y = q_body_y
            pose_msg.pose.orientation.z = q_body_z

            # Publish to MAVROS
            self.vision_pub.publish(pose_msg)

            # Log every 100 messages for health monitoring
            self.message_count += 1
            if self.message_count % 100 == 0:
                self.get_logger().info(
                    f'Vision bridge active: {self.message_count} messages published '
                    f'(rate: {self.output_rate} Hz)'
                )

        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            if self.use_tf:
                # Only log TF errors if we're in TF mode
                self.get_logger().warn(
                    f'TF lookup failed ({self.target_frame_id} -> {self.source_frame_id}): {str(e)}',
                    throttle_duration_sec=10.0
                )


def main(args=None):
    rclpy.init(args=args)
    node = VisionBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info('Vision bridge shutting down')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
