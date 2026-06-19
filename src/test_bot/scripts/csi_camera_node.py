#!/usr/bin/env python3
"""
csi_camera_node.py
------------------
Nodo de camara CSI (IMX219 / Raspberry Cam v2) para Jetson Nano en ROS2 Eloquent.

Por que existe:
  En la Jetson, `v4l2_camera` NO captura camaras CSI: hay que pasar por el ISP de
  NVIDIA (Argus) via GStreamer (`nvarguscamerasrc`). Este nodo abre ese pipeline
  con OpenCV (CAP_GSTREAMER, appsink BGR) y republica como ROS:

      /camera/image_raw      sensor_msgs/Image      (bgr8)
      /camera/camera_info    sensor_msgs/CameraInfo (de un yaml estilo camera_calibration)

  Son exactamente los topics/encoding que espera aruco_localizer.py.

Las caps del pipeline son las mismas que ya funcionan en capbot-jetson-bridge
(net/video_pipeline.py), pero terminando en appsink en vez de H264/UDP.

Parametros (todos con default):
  sensor_id          : id del sensor CSI (0 o 1). Def 0
  capture_width      : ancho de captura del sensor. Def 1280
  capture_height     : alto de captura del sensor. Def 720
  output_width       : ancho de salida (nvvidconv reescala). Def 640
  output_height      : alto de salida. Def 480
  framerate          : fps. Def 30
  flip_method        : nvvidconv flip-method 0..7 (2 = 180deg). Def 0
  frame_id           : frame del CameraInfo/Image. Def 'camera_link_optical'
  camera_info_url    : ruta a yaml de calibracion (file://... o ruta directa). Def ''
  publish_rate       : Hz a los que se publica (<= framerate). Def 30.0

IMPORTANTE sobre calibracion: los intrinsecos del yaml deben corresponder a la
resolucion de SALIDA (output_width x output_height). Si recalibras, hazlo a esa
misma resolucion o reescala K en consecuencia.
"""

import sys
import os

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Image, CameraInfo

import cv2
from cv_bridge import CvBridge


def build_gst_pipeline(sensor_id, cap_w, cap_h, out_w, out_h, fps, flip):
    """Pipeline Argus -> nvvidconv (reescala/flip) -> BGR -> appsink."""
    return (
        "nvarguscamerasrc sensor-id={sid} ! "
        "video/x-raw(memory:NVMM),width={cw},height={ch},"
        "framerate={fps}/1,format=NV12 ! "
        "nvvidconv flip-method={flip} ! "
        "video/x-raw,width={ow},height={oh},format=BGRx ! "
        "videoconvert ! "
        "video/x-raw,format=BGR ! "
        "appsink drop=true max-buffers=1 sync=false"
    ).format(sid=sensor_id, cw=cap_w, ch=cap_h, fps=fps,
             flip=flip, ow=out_w, oh=out_h)


def load_camera_info(path, default_w, default_h, frame_id):
    """Carga un yaml estilo camera_calibration en un CameraInfo.

    Acepta tanto rutas directas como 'file://...'. Si no hay archivo o falla,
    devuelve un CameraInfo con K identidad-aproximada (sin distorsion) para no
    tumbar el nodo; aruco_localizer igual necesita una K razonable, asi que se
    avisa por log."""
    info = CameraInfo()
    info.header.frame_id = frame_id
    info.width = default_w
    info.height = default_h

    if path.startswith("file://"):
        path = path[len("file://"):]

    if not path or not os.path.isfile(path):
        return info, False

    import yaml
    with open(path, "r") as f:
        data = yaml.safe_load(f)

    info.width = int(data.get("image_width", default_w))
    info.height = int(data.get("image_height", default_h))
    info.distortion_model = data.get("distortion_model", "plumb_bob")

    def _mat(key):
        node = data.get(key, {})
        return [float(x) for x in node.get("data", [])]

    k = _mat("camera_matrix")
    d = _mat("distortion_coefficients")
    r = _mat("rectification_matrix")
    p = _mat("projection_matrix")

    if len(k) == 9:
        info.k = k
    if d:
        info.d = d
    if len(r) == 9:
        info.r = r
    if len(p) == 12:
        info.p = p
    return info, True


class CsiCameraNode(Node):
    def __init__(self):
        super().__init__("csi_camera_node")

        self.declare_parameter("sensor_id", 0)
        self.declare_parameter("capture_width", 1280)
        self.declare_parameter("capture_height", 720)
        self.declare_parameter("output_width", 640)
        self.declare_parameter("output_height", 480)
        self.declare_parameter("framerate", 30)
        self.declare_parameter("flip_method", 0)
        self.declare_parameter("frame_id", "camera_link_optical")
        self.declare_parameter("camera_info_url", "")
        self.declare_parameter("publish_rate", 30.0)

        gp = self.get_parameter
        sensor_id = gp("sensor_id").value
        cap_w = gp("capture_width").value
        cap_h = gp("capture_height").value
        out_w = gp("output_width").value
        out_h = gp("output_height").value
        fps = gp("framerate").value
        flip = gp("flip_method").value
        self.frame_id = gp("frame_id").value
        info_url = gp("camera_info_url").value
        pub_rate = float(gp("publish_rate").value)

        self.bridge = CvBridge()

        # CameraInfo (los intrinsecos deben matchear out_w x out_h).
        self.camera_info, ok = load_camera_info(info_url, out_w, out_h, self.frame_id)
        if not ok:
            self.get_logger().warn(
                "Sin calibracion valida en '%s'. aruco necesita K real; "
                "publicando CameraInfo vacio." % info_url)

        pipeline = build_gst_pipeline(sensor_id, cap_w, cap_h, out_w, out_h, fps, flip)
        self.get_logger().info("GStreamer: %s" % pipeline)

        self.cap = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
        if not self.cap.isOpened():
            self.get_logger().fatal(
                "No se pudo abrir la camara CSI. Revisa: nvargus-daemon activo, "
                "OpenCV compilado con GStreamer (cv2.getBuildInformation), y sensor-id.")
            sys.exit(1)

        self.image_pub = self.create_publisher(Image, "/camera/image_raw",
                                               qos_profile_sensor_data)
        self.info_pub = self.create_publisher(CameraInfo, "/camera/camera_info", 10)

        period = 1.0 / max(1.0, pub_rate)
        self.timer = self.create_timer(period, self.grab_and_publish)
        self.get_logger().info(
            "CSI camera lista: %dx%d @ %.1f Hz, frame=%s"
            % (out_w, out_h, pub_rate, self.frame_id))

    def grab_and_publish(self):
        ok, frame = self.cap.read()
        if not ok or frame is None:
            self.get_logger().warn("Frame CSI vacio", throttle_duration_sec=2.0)
            return

        stamp = self.get_clock().now().to_msg()

        msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        msg.header.stamp = stamp
        msg.header.frame_id = self.frame_id
        self.image_pub.publish(msg)

        self.camera_info.header.stamp = stamp
        self.info_pub.publish(self.camera_info)

    def destroy_node(self):
        try:
            if self.cap is not None:
                self.cap.release()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = CsiCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
