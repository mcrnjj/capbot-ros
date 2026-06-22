#!/usr/bin/env python3
"""
gui_bridge_node.py  -  Puente WebSocket entre la GUI (capbot-host) y ROS2/NAV2.

Emula el flujo de rviz2 "2D Goal Pose" pero desde el dashboard del host:
  * Difunde la pose REAL del robot (TF map -> base_link) a los clientes WS.
  * Recibe objetivos de navegacion {x, y, yaw} y los reenvia a la accion
    NavigateToPose de NAV2 (bt_navigator). En Eloquent NO existe el topic
    /goal_pose, por eso se usa la ACCION via ActionClient.

Arquitectura (un solo proceso, dos hilos):
  - Hilo principal: rclpy. TF listener, timers y ActionClient. Es quien habla
    con ROS (la API de rclpy NO es thread-safe, todo lo de ROS vive aqui).
  - Hilo daemon: servidor WebSocket con su propio loop asyncio (websockets 9.x,
    compatible Python 3.6 igual que capbot-jetson-bridge).

Comunicacion entre hilos (sin tocar rclpy desde el hilo WS):
  - pose:        el timer rclpy guarda el ultimo snapshot bajo un Lock; la
                 tarea de publicacion del hilo WS lo lee y lo difunde.
  - goal/cancel: el handler WS encola en una queue.Queue (thread-safe); un
                 timer rclpy la drena y dispara la accion.
  - nav_status:  los callbacks de la accion (hilo rclpy) inyectan el mensaje en
                 el loop del hilo WS con loop.call_soon_threadsafe(...).

Protocolo JSON:
  ROS -> GUI:  {"type":"pose","x":..,"y":..,"yaw":..,"valid":bool,"stamp":..}
               {"type":"nav_status","state":"accepted|rejected|active|
                  succeeded|aborted|canceled","distance_remaining":..}
  GUI -> ROS:  {"type":"goal","x":..,"y":..,"yaw":..}
               {"type":"cancel"}

Parametros:
  map_frame   (str,   def "map")
  base_frame  (str,   def "base_link")
  ws_host     (str,   def "0.0.0.0")
  ws_port     (int,   def 8766)
  publish_hz  (float, def 10.0)
  action_name (str,   def "navigate_to_pose")   # nombre de la accion NAV2
"""
# PY36: sin `from __future__`; sin anotaciones `dict[...]`; sin asyncio.create_task.
import asyncio
import json
import math
import threading

try:
    import queue
except ImportError:  # pragma: no cover
    import Queue as queue  # type: ignore

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.time import Time

import tf2_ros
from tf2_ros import Buffer, TransformListener

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus

# PY36: websockets 9.x es la ultima compatible con Python 3.6 (ver jetson-bridge).
try:
    import websockets
    from websockets.exceptions import ConnectionClosed
except ImportError:  # pragma: no cover
    websockets = None
    ConnectionClosed = Exception  # type: ignore


def yaw_from_quaternion(qx, qy, qz, qw):
    """Yaw (rad) desde cuaternion. Solo necesitamos la rotacion en Z (plano)."""
    return math.atan2(2.0 * (qw * qz + qx * qy),
                      1.0 - 2.0 * (qy * qy + qz * qz))


class GuiBridgeNode(Node):
    def __init__(self):
        super().__init__('gui_bridge_node')

        # ---------------- Parametros ----------------
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('ws_host', '0.0.0.0')
        self.declare_parameter('ws_port', 8766)
        self.declare_parameter('publish_hz', 10.0)
        self.declare_parameter('action_name', 'navigate_to_pose')

        self._map_frame = self.get_parameter('map_frame').value
        self._base_frame = self.get_parameter('base_frame').value
        self._ws_host = self.get_parameter('ws_host').value
        self._ws_port = int(self.get_parameter('ws_port').value)
        self._publish_hz = float(self.get_parameter('publish_hz').value)
        self._action_name = self.get_parameter('action_name').value

        # ---------------- Estado compartido entre hilos ----------------
        self._pose_lock = threading.Lock()
        self._pose_snapshot = None          # ultimo dict de pose listo para enviar
        self._goal_queue = queue.Queue()    # GUI -> ROS: ('goal', x, y, yaw) | ('cancel',)

        self._clients = set()               # websockets conectados (solo hilo WS)
        self._ws_loop = None                # loop asyncio del hilo WS

        # ActionClient + handle del goal activo (solo hilo rclpy)
        self._action = ActionClient(self, NavigateToPose, self._action_name)
        self._goal_handle = None

        # ---------------- TF ----------------
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # ---------------- Timers (hilo rclpy) ----------------
        period = 1.0 / self._publish_hz if self._publish_hz > 0 else 0.1
        self._pose_timer = self.create_timer(period, self._update_pose)
        self._goal_timer = self.create_timer(0.05, self._drain_goals)

        # ---------------- Hilo WebSocket ----------------
        self._ws_thread = threading.Thread(target=self._run_ws, name='gui-ws', daemon=True)
        self._ws_thread.start()

        self.get_logger().info(
            "gui_bridge_node listo: WS ws://%s:%d  (map=%s base=%s action=%s)"
            % (self._ws_host, self._ws_port, self._map_frame,
               self._base_frame, self._action_name))

    # =============================================================
    # Hilo rclpy: TF -> snapshot de pose
    # =============================================================
    def _update_pose(self):
        try:
            tf = self._tf_buffer.lookup_transform(
                self._map_frame, self._base_frame, Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            # Aun no hay TF map->base_link (localizacion no lista). Avisamos invalido.
            with self._pose_lock:
                self._pose_snapshot = {"type": "pose", "valid": False}
            return

        t = tf.transform.translation
        q = tf.transform.rotation
        yaw = yaw_from_quaternion(q.x, q.y, q.z, q.w)
        stamp = tf.header.stamp.sec + tf.header.stamp.nanosec * 1e-9
        with self._pose_lock:
            self._pose_snapshot = {
                "type": "pose",
                "x": round(t.x, 4),
                "y": round(t.y, 4),
                "yaw": round(yaw, 4),
                "valid": True,
                "stamp": stamp,
            }

    def _read_pose_snapshot(self):
        with self._pose_lock:
            return self._pose_snapshot

    # =============================================================
    # Hilo rclpy: goals/cancel -> accion NavigateToPose
    # =============================================================
    def _drain_goals(self):
        while True:
            try:
                item = self._goal_queue.get_nowait()
            except queue.Empty:
                return
            if item[0] == 'goal':
                self._send_goal(item[1], item[2], item[3])
            elif item[0] == 'cancel':
                self._cancel_goal()

    def _send_goal(self, x, y, yaw):
        if not self._action.server_is_ready():
            # No bloqueamos el hilo de timers; un wait corto basta para el primer goal.
            if not self._action.wait_for_server(timeout_sec=1.0):
                self.get_logger().warn("NavigateToPose no disponible; goal descartado")
                self._emit_status("rejected")
                return

        goal = NavigateToPose.Goal()
        ps = PoseStamped()
        ps.header.frame_id = self._map_frame
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = float(x)
        ps.pose.position.y = float(y)
        ps.pose.position.z = 0.0
        ps.pose.orientation.z = math.sin(float(yaw) / 2.0)
        ps.pose.orientation.w = math.cos(float(yaw) / 2.0)
        goal.pose = ps

        self.get_logger().info("Goal -> x=%.3f y=%.3f yaw=%.3f" % (x, y, yaw))
        send_future = self._action.send_goal_async(
            goal, feedback_callback=self._on_feedback)
        send_future.add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Goal RECHAZADO por NAV2")
            self._emit_status("rejected")
            return
        self._goal_handle = goal_handle
        self._emit_status("accepted")
        self._emit_status("active")
        goal_handle.get_result_async().add_done_callback(self._on_result)

    def _on_feedback(self, feedback_msg):
        try:
            dist = float(feedback_msg.feedback.distance_remaining)
        except AttributeError:
            dist = None
        self._emit_status("active", dist)

    def _on_result(self, future):
        status = future.result().status
        mapping = {
            GoalStatus.STATUS_SUCCEEDED: "succeeded",
            GoalStatus.STATUS_ABORTED: "aborted",
            GoalStatus.STATUS_CANCELED: "canceled",
        }
        state = mapping.get(status, "aborted")
        self.get_logger().info("Resultado navegacion: %s (status=%d)" % (state, status))
        self._goal_handle = None
        self._emit_status(state)

    def _cancel_goal(self):
        if self._goal_handle is not None:
            self.get_logger().info("Cancelando goal activo")
            self._goal_handle.cancel_goal_async()

    # =============================================================
    # Puente hilo rclpy -> hilo WS: difundir nav_status
    # =============================================================
    def _emit_status(self, state, distance_remaining=None):
        loop = self._ws_loop
        if loop is None:
            return
        msg = {"type": "nav_status", "state": state}
        if distance_remaining is not None:
            msg["distance_remaining"] = round(distance_remaining, 3)
        payload = json.dumps(msg)
        loop.call_soon_threadsafe(self._schedule_broadcast, payload)

    # =============================================================
    # Hilo WebSocket
    # =============================================================
    def _run_ws(self):
        if websockets is None:
            self.get_logger().error("paquete 'websockets' no instalado; WS no arranca")
            return
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        self._ws_loop = loop
        try:
            server = loop.run_until_complete(
                websockets.serve(self._ws_handler, self._ws_host, self._ws_port,
                                 ping_interval=5, ping_timeout=5))
            asyncio.ensure_future(self._publish_loop(), loop=loop)
            loop.run_forever()
        except Exception as exc:  # pragma: no cover
            self.get_logger().error("Servidor WS cayo: %s" % exc)
        finally:
            try:
                server.close()
            except Exception:
                pass
            loop.close()

    async def _ws_handler(self, ws, path=None):
        # PY36/websockets 9.x: handler (ws, path).
        self._clients.add(ws)
        self.get_logger().info("Cliente GUI conectado (total=%d)" % len(self._clients))
        try:
            async for msg in ws:
                self._on_ws_message(msg)
        except ConnectionClosed:
            pass
        finally:
            self._clients.discard(ws)
            self.get_logger().info("Cliente GUI desconectado (total=%d)" % len(self._clients))

    def _on_ws_message(self, msg):
        try:
            if isinstance(msg, bytes):
                msg = msg.decode('utf-8')
            data = json.loads(msg)
        except (UnicodeDecodeError, ValueError):
            return
        if not isinstance(data, dict):
            return
        mtype = data.get("type")
        if mtype == "goal":
            try:
                x = float(data["x"])
                y = float(data["y"])
                yaw = float(data.get("yaw", 0.0))
            except (KeyError, TypeError, ValueError):
                return
            self._goal_queue.put(('goal', x, y, yaw))
        elif mtype == "cancel":
            self._goal_queue.put(('cancel',))

    async def _publish_loop(self):
        period = 1.0 / self._publish_hz if self._publish_hz > 0 else 0.1
        while True:
            await asyncio.sleep(period)
            pose = self._read_pose_snapshot()
            if pose is not None and self._clients:
                await self._broadcast(json.dumps(pose))

    def _schedule_broadcast(self, payload):
        # Corre en el loop del hilo WS (via call_soon_threadsafe).
        asyncio.ensure_future(self._broadcast(payload), loop=self._ws_loop)

    async def _broadcast(self, payload):
        if not self._clients:
            return
        coros = [c.send(payload) for c in list(self._clients)]
        await asyncio.gather(*coros, return_exceptions=True)


def main(args=None):
    rclpy.init(args=args)
    node = GuiBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
