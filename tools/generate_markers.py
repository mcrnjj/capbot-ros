#!/usr/bin/env python3
"""
generate_markers.py  -  Genera PNGs de marcadores ArUco para imprimir.
=====================================================================

El repo NO incluye imagenes de marcadores. Este script las crea para que las
imprimas y puedas probar la deteccion (test_aruco.py) y, mas adelante, la
localizacion del robot.

Por defecto genera los IDs y el diccionario que usa config/markers_db_small.yaml:
  diccionario = DICT_5X5_250
  ids         = 0, 1, 2
  tamano      = 0.10 m (100 mm)  <- 'marker_size' del yaml

IMPORTANTE al imprimir:
  - Imprime SIN "ajustar a pagina" / "fit to page" (eso reescala y rompe el
    tamano fisico). Usa "tamano real" / "100%".
  - Mide con regla el lado NEGRO del marcador ya impreso: debe coincidir con
    --size_mm. Si no, ajusta la escala de impresion o el 'marker_size' del yaml.
  - Pega cada marcador plano sobre carton/superficie rigida; si se comba, la
    pose sale mal.

Uso:
  python3 generate_markers.py                       # ids 0,1,2 DICT_5X5_250 a 100mm
  python3 generate_markers.py --ids 0 1 2 3 4
  python3 generate_markers.py --dict DICT_4X4_50 --size_mm 150
  python3 generate_markers.py --out_dir markers_print

Requiere: opencv-contrib-python (el modulo cv2.aruco). En la Jetson el OpenCV
de JetPack ya lo trae; en un PC: pip3 install opencv-contrib-python
"""

import argparse
import os
import sys

import cv2
import numpy as np


def generate_marker_image(aruco_dict, marker_id, size_mm, dpi, border_mm):
    """Devuelve una imagen (numpy) del marcador con borde blanco (quiet zone)."""
    px_per_mm = dpi / 25.4
    side_px = int(round(size_mm * px_per_mm))
    border_px = int(round(border_mm * px_per_mm))

    # API nueva vs antigua para dibujar el marcador
    if hasattr(cv2.aruco, "generateImageMarker"):
        marker = cv2.aruco.generateImageMarker(aruco_dict, marker_id, side_px)
    else:
        marker = cv2.aruco.drawMarker(aruco_dict, marker_id, side_px)

    canvas = np.full(
        (side_px + 2 * border_px, side_px + 2 * border_px), 255, dtype=np.uint8)
    canvas[border_px:border_px + side_px, border_px:border_px + side_px] = marker
    return canvas


def main():
    ap = argparse.ArgumentParser(description="Genera PNGs de marcadores ArUco para imprimir")
    ap.add_argument("--dict", default="DICT_5X5_250",
                    help="diccionario (def DICT_5X5_250, el de markers_db_small.yaml)")
    ap.add_argument("--ids", type=int, nargs="+", default=[0, 1, 2],
                    help="ids a generar (def 0 1 2)")
    ap.add_argument("--size_mm", type=float, default=100.0,
                    help="lado del marcador en mm (def 100 = marker_size 0.10 m)")
    ap.add_argument("--border_mm", type=float, default=10.0,
                    help="ancho del borde blanco/quiet zone en mm (def 10)")
    ap.add_argument("--dpi", type=int, default=300, help="resolucion de impresion (def 300)")
    ap.add_argument("--out_dir", default="markers_print", help="carpeta de salida")
    args = ap.parse_args()

    if not hasattr(cv2.aruco, args.dict):
        print("[X] Diccionario desconocido: %s" % args.dict)
        sys.exit(2)
    aruco_dict = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, args.dict))

    os.makedirs(args.out_dir, exist_ok=True)

    for mid in args.ids:
        img = generate_marker_image(aruco_dict, mid, args.size_mm, args.dpi, args.border_mm)
        fname = os.path.join(
            args.out_dir,
            "%s_id%d_%dmm.png" % (args.dict, mid, int(args.size_mm)))
        cv2.imwrite(fname, img)
        print("[OK] %s  (%dx%d px @ %d dpi)" % (fname, img.shape[1], img.shape[0], args.dpi))

    print("\nListo. Imprime a TAMANO REAL (sin 'fit to page') y verifica con regla")
    print("que el lado negro mida %.0f mm." % args.size_mm)


if __name__ == "__main__":
    main()
