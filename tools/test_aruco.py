#!/usr/bin/env python3
"""
test_aruco.py  -  Test de deteccion ArUco AISLADO (sin ROS).
============================================================

Objetivo: validar en la Jetson Nano, de la forma mas independiente posible,
las DOS cosas que de verdad pueden fallar antes de meter ROS de por medio:

  1. Que la camara CSI (IMX219) abre via nvarguscamerasrc/GStreamer.
  2. Que el cv2.aruco del OpenCV de JetPack (4.1.1) detecta marcadores.

NO necesita: colcon build, TF, CameraInfo/calibracion, ni el resto del stack.
La calibracion NO afecta a la DETECCION (encontrar el marcador y su id); solo
afectaria a la estimacion de POSE, que aqui no se hace.

El pipeline GStreamer es el mismo que usa scripts/csi_camera_node.py, para que
si esto funciona, el nodo de ROS tambien deberia abrir la camara igual.

Diccionario por defecto: DICT_5X5_250 (el mismo de config/markers_db_*.yaml).

Uso:
  python3 test_aruco.py                  # CSI sensor 0, 640x480, imprime ids
  python3 test_aruco.py --device 1       # /dev/video1 (camara USB, no CSI)
  python3 test_aruco.py --dict DICT_4X4_50
  python3 test_aruco.py --save vista.jpg # guarda un frame con los marcadores dibujados
  python3 test_aruco.py --frames 0       # corre indefinidamente (Ctrl-C para salir)

Sin pantalla en la Jetson (headless): no se usa cv2.imshow; los ids se imprimen
por consola y opcionalmente se guarda un JPG con --save.
"""

import argparse
import sys
import time

import cv2


def build_csi_pipeline(sensor_id, cap_w, cap_h, out_w, out_h, fps, flip):
    """Mismo pipeline Argus->nvvidconv->BGR->appsink que csi_camera_node.py."""
    return (
        "nvarguscamerasrc sensor-id={sid} ! "
        "video/x-raw(memory:NVMM),width={cw},height={ch},"
        "framerate={fps}/1,format=NV12 ! "
        "nvvidconv flip-method={flip} ! "
        "video/x-raw,width={ow},height={oh},format=BGRx ! "
        "videoconvert ! video/x-raw,format=BGR ! "
        "appsink drop=true max-buffers=1 sync=false"
    ).format(sid=sensor_id, cw=cap_w, ch=cap_h, fps=fps,
             flip=flip, ow=out_w, oh=out_h)


def open_capture(args):
    """Abre la captura: CSI por GStreamer, o /dev/videoN si --device se da."""
    if args.device is not None:
        print("[i] Abriendo camara V4L2 /dev/video%d (NO CSI)" % args.device)
        return cv2.VideoCapture(args.device)

    pipeline = build_csi_pipeline(
        args.sensor_id, args.capture_width, args.capture_height,
        args.width, args.height, args.framerate, args.flip)
    print("[i] GStreamer: %s" % pipeline)
    return cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)


def make_detector(dict_name):
    if not hasattr(cv2.aruco, dict_name):
        print("[X] Diccionario desconocido: %s" % dict_name)
        sys.exit(2)
    aruco_dict = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, dict_name))
    # API nueva (OpenCV >=4.7) vs antigua (4.1.1 de JetPack)
    if hasattr(cv2.aruco, "DetectorParameters"):
        params = cv2.aruco.DetectorParameters()
    else:
        params = cv2.aruco.DetectorParameters_create()

    if hasattr(cv2.aruco, "ArucoDetector"):
        detector = cv2.aruco.ArucoDetector(aruco_dict, params)
        return lambda gray: detector.detectMarkers(gray)
    return lambda gray: cv2.aruco.detectMarkers(gray, aruco_dict, parameters=params)


def main():
    ap = argparse.ArgumentParser(description="Test de deteccion ArUco aislado (sin ROS)")
    ap.add_argument("--dict", default="DICT_5X5_250",
                    help="diccionario ArUco (def DICT_5X5_250, el de markers_db_*.yaml)")
    ap.add_argument("--sensor-id", type=int, default=0, help="id del sensor CSI (def 0)")
    ap.add_argument("--device", type=int, default=None,
                    help="usar /dev/videoN (camara USB) en vez de CSI")
    ap.add_argument("--capture-width", type=int, default=1280)
    ap.add_argument("--capture-height", type=int, default=720)
    ap.add_argument("--width", type=int, default=640, help="ancho de salida (def 640)")
    ap.add_argument("--height", type=int, default=480, help="alto de salida (def 480)")
    ap.add_argument("--framerate", type=int, default=30)
    ap.add_argument("--flip", type=int, default=0, help="nvvidconv flip-method 0..7")
    ap.add_argument("--frames", type=int, default=300,
                    help="cuantos frames procesar (0 = indefinido, Ctrl-C para salir)")
    ap.add_argument("--save", default="",
                    help="guarda el primer frame CON marcadores detectados a esta ruta")
    args = ap.parse_args()

    print("[i] OpenCV %s" % cv2.__version__)
    detect = make_detector(args.dict)

    cap = open_capture(args)
    if not cap.isOpened():
        print("[X] No se pudo abrir la camara. Revisa:")
        print("    - nvargus-daemon activo (sudo systemctl restart nvargus-daemon)")
        print("    - OpenCV con GStreamer: python3 -c \"import cv2;"
              " print(cv2.getBuildInformation())\" | grep -i gstreamer")
        print("    - sensor-id correcto / cable CSI bien conectado")
        sys.exit(1)

    print("[i] Camara abierta. Buscando marcadores '%s'... (Ctrl-C para salir)"
          % args.dict)

    seen = set()
    saved = False
    n = 0
    last_log = 0.0
    try:
        while args.frames == 0 or n < args.frames:
            ok, frame = cap.read()
            if not ok or frame is None:
                print("[!] Frame vacio (camara devolvio None)")
                time.sleep(0.05)
                continue
            n += 1

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, _ = detect(gray)

            now = time.time()
            if ids is not None and len(ids) > 0:
                flat = ids.flatten().tolist()
                seen.update(flat)
                if now - last_log > 0.3:  # no saturar la consola
                    print("[OK] frame %d  ids=%s" % (n, flat))
                    last_log = now
                if args.save and not saved:
                    cv2.aruco.drawDetectedMarkers(frame, corners, ids)
                    cv2.imwrite(args.save, frame)
                    print("[i] Frame con marcadores guardado en: %s" % args.save)
                    saved = True
            else:
                if now - last_log > 1.0:
                    print("[..] frame %d  sin marcadores" % n)
                    last_log = now
    except KeyboardInterrupt:
        print("\n[i] Interrumpido por usuario.")
    finally:
        cap.release()

    print("\n==== RESUMEN ====")
    print("Frames procesados : %d" % n)
    if seen:
        print("IDs detectados    : %s" % sorted(seen))
        print("[OK] Deteccion ArUco FUNCIONA en la Jetson.")
    else:
        print("IDs detectados    : (ninguno)")
        print("[!] No se detecto ningun marcador. Verifica:")
        print("    - que el marcador impreso sea del diccionario %s" % args.dict)
        print("    - iluminacion y que el marcador este completo en el encuadre")
        print("    - foco de la camara (los IMX219 traen foco fijo)")


if __name__ == "__main__":
    main()
