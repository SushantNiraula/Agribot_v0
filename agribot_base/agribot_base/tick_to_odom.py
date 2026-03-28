#!/usr/bin/env python3
"""
tick_to_odom.py
Converts left/right encoder tick counts into:
  - /wheel/odometry  (nav_msgs/Odometry)  → fed into EKF
  - TF broadcast: odom → base_link        → direct transform for visualisation

This node does NOT own the canonical /odom topic — the EKF
(robot_localization) publishes /odometry/filtered remapped to /odom.
"""

import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
from tf2_ros import TransformBroadcaster


class TickToOdom(Node):

    # ------------------------------------------------------------------ #
    #  Robot parameters — edit these to match your physical hardware       #
    # ------------------------------------------------------------------ #
    WHEEL_RADIUS_M   = 0.055   # metres
    WHEEL_BASE_M     = 0.50    # metres (centre-to-centre of wheels)
    TICKS_PER_REV    = 280.0   # encoder ticks per full wheel revolution

    # Twist covariances (tuned empirically; increase if odometry is noisy)
    COV_VX   = 0.01   # variance on forward velocity  (m/s)²
    COV_VYAW = 0.05   # variance on yaw rate           (rad/s)²

    # Pose covariances (driven by accumulated dead-reckoning error)
    COV_X    = 0.01   # x position
    COV_Y    = 0.01   # y position — same scale as x for a symmetric robot
    COV_YAW  = 0.05   # heading — larger because yaw error grows fastest
    # ------------------------------------------------------------------ #

    TIMER_PERIOD_S = 0.05   # 20 Hz publish rate

    def __init__(self):
        super().__init__('tick_to_odom')

        # Derived constant
        self.meters_per_tick = (
            2.0 * math.pi * self.WHEEL_RADIUS_M / self.TICKS_PER_REV
        )

        # --- Subscribers --------------------------------------------------
        self.sub_l = self.create_subscription(
            Int32, '/left_ticks',  self._left_cb,  10)
        self.sub_r = self.create_subscription(
            Int32, '/right_ticks', self._right_cb, 10)

        # --- Publishers ---------------------------------------------------
        self.odom_pub = self.create_publisher(Odometry, '/wheel/odometry', 10)

        # TF broadcaster lets rviz2 draw the robot frame without the EKF
        self.tf_broadcaster = TransformBroadcaster(self)

        # --- State --------------------------------------------------------
        self.current_l = 0
        self.current_r = 0
        self.prev_l    = 0
        self.prev_r    = 0
        self.first_read = True

        self.x  = 0.0
        self.y  = 0.0
        self.th = 0.0

        self.last_time = self.get_clock().now()

        # --- Timer --------------------------------------------------------
        self.create_timer(self.TIMER_PERIOD_S, self._update_and_publish)

        self.get_logger().info(
            f'tick_to_odom ready — '
            f'{self.meters_per_tick*1000:.4f} mm/tick, '
            f'wheelbase={self.WHEEL_BASE_M*100:.1f} cm'
        )

    # ------------------------------------------------------------------ #
    #  Callbacks                                                           #
    # ------------------------------------------------------------------ #
    def _left_cb(self, msg: Int32):
        self.current_l = msg.data

    def _right_cb(self, msg: Int32):
        self.current_r = msg.data

    # ------------------------------------------------------------------ #
    #  Main update loop                                                    #
    # ------------------------------------------------------------------ #
    def _update_and_publish(self):
        # On the very first call just snapshot the tick counts so we don't
        # get a huge phantom jump if the encoder already has a non-zero value.
        if self.first_read:
            self.prev_l = self.current_l
            self.prev_r = self.current_r
            self.first_read = False
            return

        now = self.get_clock().now()
        dt  = (now - self.last_time).nanoseconds * 1e-9

        # Guard against zero or negative dt (shouldn't happen but be safe)
        if dt <= 0.0:
            return

        # ---- Delta ticks -----------------------------------------------
        delta_l = self.current_l - self.prev_l
        delta_r = self.current_r - self.prev_r
        self.prev_l = self.current_l
        self.prev_r = self.current_r

        # ---- Convert to distances --------------------------------------
        d_l = delta_l * self.meters_per_tick
        d_r = delta_r * self.meters_per_tick

        d_center = (d_l + d_r) / 2.0
        d_theta  = (d_r - d_l) / self.WHEEL_BASE_M

        # ---- Velocities ------------------------------------------------
        v_x   = d_center / dt
        v_yaw = d_theta  / dt

        # ---- Integrate pose --------------------------------------------
        self.th += d_theta
        self.x  += d_center * math.cos(self.th)
        self.y  += d_center * math.sin(self.th)

        # ---- Build Odometry message ------------------------------------
        odom = Odometry()
        odom.header.stamp    = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id  = 'base_link'

        odom.pose.pose.position.x    = self.x
        odom.pose.pose.position.y    = self.y
        odom.pose.pose.position.z    = 0.0
        odom.pose.pose.orientation   = _yaw_to_quaternion(self.th)

        odom.twist.twist.linear.x    = v_x
        odom.twist.twist.linear.y    = 0.0   # non-holonomic — no lateral slip
        odom.twist.twist.angular.z   = v_yaw

        # Pose covariance (6×6 row-major, indices: x=0, y=7, yaw=35)
        # All off-diagonals stay 0.0 — we assume independent errors.
        pc = [0.0] * 36
        pc[0]  = self.COV_X      # σ²_x
        pc[7]  = self.COV_Y      # σ²_y
        pc[14] = 1e6             # z  — constrained by two_d_mode in EKF
        pc[21] = 1e6             # roll
        pc[28] = 1e6             # pitch
        pc[35] = self.COV_YAW   # σ²_yaw
        odom.pose.covariance = pc

        # Twist covariance
        tc = [0.0] * 36
        tc[0]  = self.COV_VX    # σ²_vx
        tc[7]  = 1e6            # vy  — zero by constraint
        tc[14] = 1e6            # vz
        tc[21] = 1e6            # v_roll
        tc[28] = 1e6            # v_pitch
        tc[35] = self.COV_VYAW  # σ²_v_yaw
        odom.twist.covariance = tc

        self.odom_pub.publish(odom)

        # ---- Broadcast TF  odom → base_link ----------------------------
        # This is a raw dead-reckoning transform.  The EKF will publish its
        # own (filtered) version of the same transform — both can coexist
        # but you should set publish_tf=false in the EKF if you only want
        # one transform.  With publish_tf=true in ekf.yaml the EKF's TF
        # wins because it is published later (same stamp, different source).
        tf = TransformStamped()
        tf.header.stamp            = now.to_msg()
        tf.header.frame_id         = 'odom'
        tf.child_frame_id          = 'base_link'
        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        tf.transform.translation.z = 0.0
        tf.transform.rotation      = _yaw_to_quaternion(self.th)
        self.tf_broadcaster.sendTransform(tf)

        self.last_time = now


# ------------------------------------------------------------------ #
#  Helper                                                              #
# ------------------------------------------------------------------ #
def _yaw_to_quaternion(yaw: float) -> Quaternion:
    """Convert a yaw angle (radians) to a geometry_msgs/Quaternion."""
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


# ------------------------------------------------------------------ #
#  Entry point                                                         #
# ------------------------------------------------------------------ #
def main(args=None):
    rclpy.init(args=args)
    node = TickToOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()#!/usr/bin/env python3
"""
tick_to_odom.py
Converts left/right encoder tick counts into:
  - /wheel/odometry  (nav_msgs/Odometry)  → fed into EKF
  - TF broadcast: odom → base_link        → direct transform for visualisation

This node does NOT own the canonical /odom topic — the EKF
(robot_localization) publishes /odometry/filtered remapped to /odom.
"""

import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped
from tf2_ros import TransformBroadcaster


class TickToOdom(Node):

    # ------------------------------------------------------------------ #
    #  Robot parameters — edit these to match your physical hardware       #
    # ------------------------------------------------------------------ #
    WHEEL_RADIUS_M   = 0.055   # metres
    WHEEL_BASE_M     = 0.50    # metres (centre-to-centre of wheels)
    TICKS_PER_REV    = 280.0   # encoder ticks per full wheel revolution

    # Twist covariances (tuned empirically; increase if odometry is noisy)
    COV_VX   = 0.01   # variance on forward velocity  (m/s)²
    COV_VYAW = 0.05   # variance on yaw rate           (rad/s)²

    # Pose covariances (driven by accumulated dead-reckoning error)
    COV_X    = 0.01   # x position
    COV_Y    = 0.01   # y position — same scale as x for a symmetric robot
    COV_YAW  = 0.05   # heading — larger because yaw error grows fastest
    # ------------------------------------------------------------------ #

    TIMER_PERIOD_S = 0.05   # 20 Hz publish rate

    def __init__(self):
        super().__init__('tick_to_odom')

        # Derived constant
        self.meters_per_tick = (
            2.0 * math.pi * self.WHEEL_RADIUS_M / self.TICKS_PER_REV
        )

        # --- Subscribers --------------------------------------------------
        self.sub_l = self.create_subscription(
            Int32, '/left_ticks',  self._left_cb,  10)
        self.sub_r = self.create_subscription(
            Int32, '/right_ticks', self._right_cb, 10)

        # --- Publishers ---------------------------------------------------
        self.odom_pub = self.create_publisher(Odometry, '/wheel/odometry', 10)

        # TF broadcaster lets rviz2 draw the robot frame without the EKF
        self.tf_broadcaster = TransformBroadcaster(self)

        # --- State --------------------------------------------------------
        self.current_l = 0
        self.current_r = 0
        self.prev_l    = 0
        self.prev_r    = 0
        self.first_read = True

        self.x  = 0.0
        self.y  = 0.0
        self.th = 0.0

        self.last_time = self.get_clock().now()

        # --- Timer --------------------------------------------------------
        self.create_timer(self.TIMER_PERIOD_S, self._update_and_publish)

        self.get_logger().info(
            f'tick_to_odom ready — '
            f'{self.meters_per_tick*1000:.4f} mm/tick, '
            f'wheelbase={self.WHEEL_BASE_M*100:.1f} cm'
        )

    # ------------------------------------------------------------------ #
    #  Callbacks                                                           #
    # ------------------------------------------------------------------ #
    def _left_cb(self, msg: Int32):
        self.current_l = msg.data

    def _right_cb(self, msg: Int32):
        self.current_r = msg.data

    # ------------------------------------------------------------------ #
    #  Main update loop                                                    #
    # ------------------------------------------------------------------ #
    def _update_and_publish(self):
        # On the very first call just snapshot the tick counts so we don't
        # get a huge phantom jump if the encoder already has a non-zero value.
        if self.first_read:
            self.prev_l = self.current_l
            self.prev_r = self.current_r
            self.first_read = False
            return

        now = self.get_clock().now()
        dt  = (now - self.last_time).nanoseconds * 1e-9

        # Guard against zero or negative dt (shouldn't happen but be safe)
        if dt <= 0.0:
            return

        # ---- Delta ticks -----------------------------------------------
        delta_l = self.current_l - self.prev_l
        delta_r = self.current_r - self.prev_r
        self.prev_l = self.current_l
        self.prev_r = self.current_r

        # ---- Convert to distances --------------------------------------
        d_l = delta_l * self.meters_per_tick
        d_r = delta_r * self.meters_per_tick

        d_center = (d_l + d_r) / 2.0
        d_theta  = (d_r - d_l) / self.WHEEL_BASE_M

        # ---- Velocities ------------------------------------------------
        v_x   = d_center / dt
        v_yaw = d_theta  / dt

        # ---- Integrate pose --------------------------------------------
        self.th += d_theta
        self.x  += d_center * math.cos(self.th)
        self.y  += d_center * math.sin(self.th)

        # ---- Build Odometry message ------------------------------------
        odom = Odometry()
        odom.header.stamp    = now.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id  = 'base_link'

        odom.pose.pose.position.x    = self.x
        odom.pose.pose.position.y    = self.y
        odom.pose.pose.position.z    = 0.0
        odom.pose.pose.orientation   = _yaw_to_quaternion(self.th)

        odom.twist.twist.linear.x    = v_x
        odom.twist.twist.linear.y    = 0.0   # non-holonomic — no lateral slip
        odom.twist.twist.angular.z   = v_yaw

        # Pose covariance (6×6 row-major, indices: x=0, y=7, yaw=35)
        # All off-diagonals stay 0.0 — we assume independent errors.
        pc = [0.0] * 36
        pc[0]  = self.COV_X      # σ²_x
        pc[7]  = self.COV_Y      # σ²_y
        pc[14] = 1e6             # z  — constrained by two_d_mode in EKF
        pc[21] = 1e6             # roll
        pc[28] = 1e6             # pitch
        pc[35] = self.COV_YAW   # σ²_yaw
        odom.pose.covariance = pc

        # Twist covariance
        tc = [0.0] * 36
        tc[0]  = self.COV_VX    # σ²_vx
        tc[7]  = 1e6            # vy  — zero by constraint
        tc[14] = 1e6            # vz
        tc[21] = 1e6            # v_roll
        tc[28] = 1e6            # v_pitch
        tc[35] = self.COV_VYAW  # σ²_v_yaw
        odom.twist.covariance = tc

        self.odom_pub.publish(odom)

        # ---- Broadcast TF  odom → base_link ----------------------------
        # This is a raw dead-reckoning transform.  The EKF will publish its
        # own (filtered) version of the same transform — both can coexist
        # but you should set publish_tf=false in the EKF if you only want
        # one transform.  With publish_tf=true in ekf.yaml the EKF's TF
        # wins because it is published later (same stamp, different source).
        tf = TransformStamped()
        tf.header.stamp            = now.to_msg()
        tf.header.frame_id         = 'odom'
        tf.child_frame_id          = 'base_link'
        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        tf.transform.translation.z = 0.0
        tf.transform.rotation      = _yaw_to_quaternion(self.th)
        self.tf_broadcaster.sendTransform(tf)

        self.last_time = now


# ------------------------------------------------------------------ #
#  Helper                                                              #
# ------------------------------------------------------------------ #
def _yaw_to_quaternion(yaw: float) -> Quaternion:
    """Convert a yaw angle (radians) to a geometry_msgs/Quaternion."""
    q = Quaternion()
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


# ------------------------------------------------------------------ #
#  Entry point                                                         #
# ------------------------------------------------------------------ #
def main(args=None):
    rclpy.init(args=args)
    node = TickToOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()