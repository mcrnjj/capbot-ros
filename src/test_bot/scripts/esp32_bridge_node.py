#!/usr/bin/env python3
"""
esp32_bridge_node.py - puente ROS2 <-> servicio capbot-jetson-bridge.

El servicio capbot-jetson-bridge corre en OTRO proceso (asyncio, fuera de
este workspace) y posee el puerto serie hacia el ESP32. Ese servicio expone
su propio nodo ROS2 ('capbot_jetson_bridge', ver ros_bridge.py en ese repo)
con dos tópicos sin tipo propio (std_msgs/Float32MultiArray):

  from_bridge  (lo publica el servicio jetson-bridge, lo suscribimos aquí)
      [x, y, theta, v, w, setpoint_x, setpoint_y, setpoint_theta]
      - x, y, theta, v, w: odometría reportada por el ESP32 (pose absoluta +
        velocidad lineal/angular).
      - setpoint_x/y/theta: última meta de posición que el host (Nav2, vía
        UDP) le mandó al ESP32 para su controlador autónomo on-board; se
        republica sólo a modo de diagnóstico/visualización.

  to_bridge    (lo publicamos aquí, lo suscribe el servicio jetson-bridge)
      [left, right, stop]
      - left, right: comando de motor en unidades crudas del firmware
        (rango [-CMD_FULL_SCALE, +CMD_FULL_SCALE], ver MotorDriver.h en el
        firmware ESP32). NO son velocidades físicas; este nodo hace la
        cinemática diferencial + escalado a partir de /cmd_vel.
      - stop: != 0 frena ya e ignora left/right.

Este nodo es un nodo ROS2 normal (sin asyncio ni hilos propios): traduce
from_bridge a nav_msgs/Odometry (+ TF opcional) para que lo consuma el EKF
(ver config/ekf.yaml) y a un PoseStamped de diagnóstico para el setpoint de
Nav2, y traduce /cmd_vel a comandos de rueda para to_bridge, con un watchdog
que frena si /cmd_vel deja de llegar.
"""
import math

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist, PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster

# Rango de comando crudo que espera el firmware del ESP32 (ver
# capbot-ESP32/include/Config.h::CMD_FULL_SCALE e int16 del frame MOTOR_CMD).
_CMD_FULL_SCALE = 32767.0

# Cantidad de floats esperados en from_bridge.
_FROM_BRIDGE_LEN = 8


def _yaw_to_quat(yaw: float):
    half = yaw / 2.0
    return 0.0, 0.0, math.sin(half), math.cos(half)


class Esp32BridgeNode(Node):
    def __init__(self) -> None:
        super().__init__('esp32_bridge_node')

        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('publish_odom_tf', False)
        self.declare_parameter('wheel_separation', 0.226)
        self.declare_parameter('wheel_radius', 0.035)
        # rad/s de rueda que corresponde a comando full-scale (+-32767).
        # Placeholder: ajustar a la velocidad máxima real de los motores.
        self.declare_parameter('max_wheel_speed', 6.0)
        self.declare_parameter('cmd_vel_timeout', 0.5)
        self.declare_parameter('twist_linear_variance', 0.01)
        self.declare_parameter('twist_angular_variance', 0.02)

        self._last_cmd_vel_time = None
        self._cmd_vel_stopped = False

        self._pub_odom = self.create_publisher(Odometry, 'odom', 10)
        self._pub_to_bridge = self.create_publisher(Float32MultiArray, 'to_bridge', 10)
        self._pub_setpoint_echo = self.create_publisher(
            PoseStamped, 'esp32_bridge/setpoint_echo', 10)
        self._tf_broadcaster = TransformBroadcaster(self)

        self.create_subscription(Float32MultiArray, 'from_bridge', self._on_from_bridge, 10)
        self.create_subscription(Twist, 'cmd_vel', self._on_cmd_vel, 10)
        self.create_timer(0.1, self._check_cmd_vel_timeout)

        self.get_logger().info(
            "Listo. from_bridge -> /odom (tf %s) | /cmd_vel -> to_bridge "
            "(wheel_sep=%.3fm, wheel_r=%.3fm, max_wheel_speed=%.2f rad/s, "
            "cmd_vel_timeout=%.2fs)" % (
                "habilitado" if self.get_parameter('publish_odom_tf').value else "deshabilitado",
                self.get_parameter('wheel_separation').value,
                self.get_parameter('wheel_radius').value,
                self.get_parameter('max_wheel_speed').value,
                self.get_parameter('cmd_vel_timeout').value,
            )
        )

    # -------------------- from_bridge -> /odom --------------------
    def _on_from_bridge(self, msg: Float32MultiArray) -> None:
        if len(msg.data) < _FROM_BRIDGE_LEN:
            self.get_logger().warn(
                f"from_bridge: se esperaban {_FROM_BRIDGE_LEN} valores "
                "[x,y,theta,v,w,setpoint_x,setpoint_y,setpoint_theta], "
                f"llegaron {len(msg.data)}",
                throttle_duration_sec=2.0,
            )
            return

        x, y, theta, v, w, sx, sy, stheta = msg.data[:_FROM_BRIDGE_LEN]
        stamp = self.get_clock().now().to_msg()
        self._publish_odom(x, y, theta, v, w, stamp)
        self._publish_setpoint_echo(sx, sy, stheta, stamp)

    def _publish_odom(self, x, y, theta, v, w, stamp) -> None:
        qx, qy, qz, qw = _yaw_to_quat(theta)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.get_parameter('odom_frame').value
        odom.child_frame_id = self.get_parameter('base_frame').value
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w

        lin_var = self.get_parameter('twist_linear_variance').value
        ang_var = self.get_parameter('twist_angular_variance').value
        cov = list(odom.twist.covariance)
        cov[0] = lin_var    # vx
        cov[7] = lin_var    # vy (robot no-holonómico: ~0 con misma confianza)
        cov[35] = ang_var   # vyaw
        odom.twist.covariance = cov

        self._pub_odom.publish(odom)

        if self.get_parameter('publish_odom_tf').value:
            tf = TransformStamped()
            tf.header.stamp = stamp
            tf.header.frame_id = odom.header.frame_id
            tf.child_frame_id = odom.child_frame_id
            tf.transform.translation.x = x
            tf.transform.translation.y = y
            tf.transform.rotation.x = qx
            tf.transform.rotation.y = qy
            tf.transform.rotation.z = qz
            tf.transform.rotation.w = qw
            self._tf_broadcaster.sendTransform(tf)

    def _publish_setpoint_echo(self, x, y, theta, stamp) -> None:
        qx, qy, qz, qw = _yaw_to_quat(theta)

        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = self.get_parameter('map_frame').value
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.x = qx
        pose.pose.orientation.y = qy
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw
        self._pub_setpoint_echo.publish(pose)

    # -------------------- /cmd_vel -> to_bridge --------------------
    def _on_cmd_vel(self, msg: Twist) -> None:
        self._last_cmd_vel_time = self.get_clock().now()
        self._cmd_vel_stopped = False

        separation = self.get_parameter('wheel_separation').value
        radius = self.get_parameter('wheel_radius').value
        max_speed = self.get_parameter('max_wheel_speed').value

        v_left = (msg.linear.x - msg.angular.z * separation / 2.0) / radius
        v_right = (msg.linear.x + msg.angular.z * separation / 2.0) / radius

        self._publish_wheel_cmd(v_left, v_right, max_speed, stop=False)

    def _publish_wheel_cmd(self, v_left: float, v_right: float, max_speed: float,
                            stop: bool) -> None:
        if stop or max_speed <= 0.0:
            left_cmd = right_cmd = 0.0
        else:
            scale = _CMD_FULL_SCALE / max_speed
            left_cmd = max(-_CMD_FULL_SCALE, min(_CMD_FULL_SCALE, v_left * scale))
            right_cmd = max(-_CMD_FULL_SCALE, min(_CMD_FULL_SCALE, v_right * scale))

        out = Float32MultiArray()
        out.data = [left_cmd, right_cmd, 1.0 if stop else 0.0]
        self._pub_to_bridge.publish(out)

    # -------------------- watchdog /cmd_vel --------------------
    def _check_cmd_vel_timeout(self) -> None:
        if self._last_cmd_vel_time is None or self._cmd_vel_stopped:
            return

        timeout = self.get_parameter('cmd_vel_timeout').value
        elapsed = (self.get_clock().now() - self._last_cmd_vel_time).nanoseconds / 1e9
        if elapsed > timeout:
            self._cmd_vel_stopped = True
            self._publish_wheel_cmd(0.0, 0.0, 0.0, stop=True)
            self.get_logger().warn(
                f"Sin /cmd_vel hace {elapsed:.2f}s (timeout={timeout:.2f}s); frenando.",
                throttle_duration_sec=2.0,
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Esp32BridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
