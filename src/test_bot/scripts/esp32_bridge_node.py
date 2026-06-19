#!/usr/bin/env python3
"""
esp32_bridge_node.py
--------------------
Puente ROS2 <-> ESP32 para el Capbot (Jetson Nano, ROS2 Eloquent).

Reemplaza, para el modo autonomo, al servicio teleop `capbot-jetson-bridge`:
habla el MISMO protocolo serie (COBS + CRC16-CCITT, /dev/ttyTHS1 @115200) pero
expone interfaces ROS:

  Publica:
    /odom              nav_msgs/Odometry   (desde la telemetria 'odo' del ESP32)

  Suscribe:
    /cmd_vel           geometry_msgs/Twist (de NAV2 -> VEL_CMD al ESP32)

Arquitectura de control (decidida para este port):
  NAV2 controller -> /cmd_vel (v, w) -> este nodo -> VEL_CMD (0x16) -> ESP32,
  que alimenta sus PID de velocidad internos (linearVelPid / angularVelPid).

  >>> REQUIERE UN CAMBIO DE FIRMWARE EN capbot-ESP32 <<<
  Hoy el firmware NO tiene VEL_CMD (0x16). Hay que anadirlo: nuevo MsgType que
  reciba <ff> = (v_m_s, w_rad_s) y los use como setpoint de velocidad, en un modo
  de velocidad. Ver README (seccion "Cambio de firmware requerido"). Mientras no
  exista, el ESP32 ignorara el frame (su framing COBS+CRC lo valida y descarta el
  tipo desconocido sin corromper el stream), asi que el puente es seguro de correr.

Telemetria del ESP32 (JSON, MsgType 0x20):
  {mode, u:{pwm_left,pwm_right}, odo:{x,y,a(grados),v(m/s),w(grados/s)}, sp, error}
  Solo usamos 'odo'. La IMU cruda no se transmite (el ESP32 ya fusiona internamente).

TF: por defecto este nodo NO publica odom->base_link, porque de eso se encarga el
EKF local de robot_localization (ekf.yaml: publish_tf=true). Si corres SIN EKF,
pon publish_odom_tf:=true.
"""

import sys
import math
import json
import struct
import threading

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, TransformStamped
from tf2_ros import TransformBroadcaster

try:
    import serial
except ImportError:
    serial = None


# =============================================================================
# Framing COBS + CRC16 (embebido; byte-compatible con capbot-ESP32 / bridge)
# =============================================================================
DELIMITER = 0x00

# MsgType — mantener sincronizado con capbot-ESP32/include/Config.h
MOTOR_CMD     = 0x10
BRAKE_ON      = 0x11
HEARTBEAT     = 0x12
PID_PARAM     = 0x13
SETPOINT_COMP = 0x14
MODE_CMD      = 0x15
VEL_CMD       = 0x16   # NUEVO (requiere soporte en firmware)
TELEMETRY     = 0x20
ESP_HELLO     = 0x21


def crc16_ccitt(data, init=0xFFFF):
    crc = init
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def cobs_encode(data):
    out = bytearray([0])
    code_idx = 0
    code = 1
    for b in data:
        if b == 0:
            out[code_idx] = code
            code_idx = len(out)
            out.append(0)
            code = 1
        else:
            out.append(b)
            code += 1
            if code == 0xFF:
                out[code_idx] = code
                code_idx = len(out)
                out.append(0)
                code = 1
    out[code_idx] = code
    return bytes(out)


def cobs_decode(data):
    out = bytearray()
    i = 0
    n = len(data)
    while i < n:
        code = data[i]
        if code == 0:
            raise ValueError("cero inesperado en stream COBS")
        end = i + code
        if end > n:
            raise ValueError("codigo COBS se pasa del final")
        out.extend(data[i + 1:end])
        i = end
        if code < 0xFF and i < n:
            out.append(0)
    return bytes(out)


def pack_frame(msg_type, payload):
    raw = struct.pack("<BB", msg_type & 0xFF, len(payload)) + payload
    raw += struct.pack("<H", crc16_ccitt(raw))
    return cobs_encode(raw) + bytes([DELIMITER])


def unpack_frame(encoded):
    """encoded sin el delimitador final. Devuelve (msg_type, payload) o lanza."""
    raw = cobs_decode(encoded)
    if len(raw) < 4:
        raise ValueError("frame truncado")
    msg_type, length = raw[0], raw[1]
    if len(raw) != 2 + length + 2:
        raise ValueError("longitud inconsistente")
    payload = raw[2:2 + length]
    (crc_recv,) = struct.unpack("<H", raw[2 + length:])
    if crc_recv != crc16_ccitt(raw[:2 + length]):
        raise ValueError("CRC invalido")
    return msg_type, payload


class FrameBuffer:
    """Acumula bytes y entrega frames completos al ver 0x00."""

    def __init__(self, max_frame_bytes=512):
        self._buf = bytearray()
        self._max = max_frame_bytes

    def feed(self, data):
        frames = []
        for b in data:
            if b == DELIMITER:
                if self._buf:
                    try:
                        frames.append(unpack_frame(bytes(self._buf)))
                    except ValueError:
                        pass
                    self._buf.clear()
            else:
                self._buf.append(b)
                if len(self._buf) > self._max:
                    self._buf.clear()
        return frames


# Builders
def build_heartbeat():
    return pack_frame(HEARTBEAT, b"")


def build_brake():
    return pack_frame(BRAKE_ON, b"")


def build_mode(mode):
    return pack_frame(MODE_CMD, struct.pack("<B", mode & 0xFF))


def build_vel(v_mps, w_rps):
    # Contrato propuesto: <ff> = (velocidad lineal m/s, velocidad angular rad/s)
    return pack_frame(VEL_CMD, struct.pack("<ff", float(v_mps), float(w_rps)))


# =============================================================================
# Nodo
# =============================================================================
class Esp32BridgeNode(Node):
    def __init__(self):
        super().__init__("esp32_bridge_node")

        self.declare_parameter("serial_port", "/dev/ttyTHS1")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("publish_odom_tf", False)   # EKF lo publica
        self.declare_parameter("heartbeat_period", 0.05)   # 50 ms
        self.declare_parameter("cmd_vel_timeout", 0.5)     # s sin cmd_vel -> frena
        self.declare_parameter("esp32_mode", 1)            # 1 = autonomo/velocidad
        self.declare_parameter("send_mode_on_start", True)

        gp = self.get_parameter
        self.port = gp("serial_port").value
        self.baud = gp("baudrate").value
        self.odom_frame = gp("odom_frame").value
        self.base_frame = gp("base_frame").value
        self.publish_tf = gp("publish_odom_tf").value
        self.hb_period = float(gp("heartbeat_period").value)
        self.cmd_timeout = float(gp("cmd_vel_timeout").value)
        self.esp32_mode = int(gp("esp32_mode").value)
        self.send_mode = gp("send_mode_on_start").value

        if serial is None:
            self.get_logger().fatal("pyserial no instalado: pip3 install pyserial")
            sys.exit(1)

        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.tf_broadcaster = TransformBroadcaster(self) if self.publish_tf else None
        self.create_subscription(Twist, "/cmd_vel", self.on_cmd_vel, 10)

        self._buffer = FrameBuffer()
        self._ser = None
        self._tx_lock = threading.Lock()
        self._last_cmd_time = self.get_clock().now()
        self._running = True

        # Hilo de lectura serie (pyserial es bloqueante).
        self._reader = threading.Thread(target=self._reader_loop, daemon=True)
        self._reader.start()

        # Timers ROS para heartbeat y watchdog de cmd_vel.
        self.create_timer(self.hb_period, self._send_heartbeat)
        self.create_timer(0.1, self._cmd_watchdog)

        if self.send_mode:
            # Se reintenta en _open_port tambien, por si el puerto aun no abre.
            self._pending_mode = True
        else:
            self._pending_mode = False

        self.get_logger().info(
            "esp32_bridge: %s @ %d | odom_frame=%s base_frame=%s | tf=%s | mode=%d"
            % (self.port, self.baud, self.odom_frame, self.base_frame,
               self.publish_tf, self.esp32_mode))

    # ---------------- Serie ----------------
    def _open_port(self):
        try:
            self._ser = serial.Serial(
                port=self.port, baudrate=self.baud,
                timeout=0.05, write_timeout=0.2)
            self._ser.reset_input_buffer()
            self._ser.reset_output_buffer()
            self.get_logger().info("Serial ESP32 abierto: %s" % self.port)
            if self._pending_mode:
                self._write(build_mode(self.esp32_mode))
            return True
        except Exception as exc:
            self.get_logger().warn("No se pudo abrir %s: %s" % (self.port, exc),
                                   throttle_duration_sec=2.0)
            self._ser = None
            return False

    def _write(self, data):
        with self._tx_lock:
            if self._ser is not None and self._ser.is_open:
                try:
                    self._ser.write(data)
                except Exception as exc:
                    self.get_logger().warn("Error escribiendo serial: %s" % exc,
                                           throttle_duration_sec=2.0)

    def _reader_loop(self):
        while self._running:
            if self._ser is None or not self._ser.is_open:
                if not self._open_port():
                    self._sleep(1.0)
                    continue
            try:
                waiting = self._ser.in_waiting
                data = self._ser.read(waiting if waiting > 0 else 1)
                if data:
                    for msg_type, payload in self._buffer.feed(data):
                        self._dispatch(msg_type, payload)
            except Exception as exc:
                self.get_logger().warn("Error leyendo serial: %s" % exc,
                                       throttle_duration_sec=2.0)
                try:
                    self._ser.close()
                except Exception:
                    pass
                self._ser = None
                self._sleep(0.5)

    def _sleep(self, s):
        # sleep simple sin bloquear el spin (corre en hilo aparte).
        import time
        time.sleep(s)

    def _dispatch(self, msg_type, payload):
        if msg_type == TELEMETRY:
            self._handle_telemetry(payload)
        elif msg_type == ESP_HELLO:
            self.get_logger().info("ESP32 HELLO")
            if self._pending_mode:
                self._write(build_mode(self.esp32_mode))

    def _handle_telemetry(self, payload):
        try:
            data = json.loads(payload.decode("utf-8"))
        except (UnicodeDecodeError, ValueError):
            return
        if not isinstance(data, dict):
            return
        odo = data.get("odo")
        if not isinstance(odo, dict):
            return
        self._publish_odom(odo)

    def _publish_odom(self, odo):
        x = float(odo.get("x", 0.0))
        y = float(odo.get("y", 0.0))
        yaw = math.radians(float(odo.get("a", 0.0)))    # 'a' viene en grados
        v = float(odo.get("v", 0.0))                    # m/s
        w = math.radians(float(odo.get("w", 0.0)))      # 'w' viene en grados/s

        stamp = self.get_clock().now().to_msg()
        qz = math.sin(yaw / 2.0)
        qw = math.cos(yaw / 2.0)

        msg = Odometry()
        msg.header.stamp = stamp
        msg.header.frame_id = self.odom_frame
        msg.child_frame_id = self.base_frame
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        msg.twist.twist.linear.x = v
        msg.twist.twist.angular.z = w

        # Covarianzas diagonales razonables (x, y, yaw / vx, vyaw).
        msg.pose.covariance[0] = 0.02
        msg.pose.covariance[7] = 0.02
        msg.pose.covariance[35] = 0.05
        msg.twist.covariance[0] = 0.02
        msg.twist.covariance[35] = 0.05
        self.odom_pub.publish(msg)

        if self.tf_broadcaster is not None:
            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = x
            t.transform.translation.y = y
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(t)

    # ---------------- Comandos ----------------
    def on_cmd_vel(self, msg):
        self._last_cmd_time = self.get_clock().now()
        self._write(build_vel(msg.linear.x, msg.angular.z))

    def _cmd_watchdog(self):
        # Si NAV2 deja de mandar cmd_vel, mandar velocidad cero (frenar suave).
        dt = (self.get_clock().now() - self._last_cmd_time).nanoseconds * 1e-9
        if dt > self.cmd_timeout:
            self._write(build_vel(0.0, 0.0))

    def _send_heartbeat(self):
        self._write(build_heartbeat())

    def destroy_node(self):
        self._running = False
        try:
            self._write(build_brake())
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = Esp32BridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
