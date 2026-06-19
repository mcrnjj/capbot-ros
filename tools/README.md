# tools/ — utilidades de prueba (fuera del workspace ROS)

Scripts standalone para validar hardware/visión **sin** ROS ni `colcon build`.
No son nodos ni se instalan con el paquete.

## `test_aruco.py` — Nivel 1: detección ArUco aislada
Abre la cámara CSI (mismo pipeline GStreamer que `csi_camera_node.py`) y reporta
los IDs ArUco detectados. Valida cámara + `cv2.aruco` antes de meter ROS.
La calibración NO afecta a la detección, así que no hace falta `camera.yaml`.

```bash
pip3 install numpy          # si falta
python3 test_aruco.py                  # CSI sensor 0, DICT_5X5_250
python3 test_aruco.py --save vista.jpg # guarda un frame con los marcadores dibujados
python3 test_aruco.py --device 1       # cámara USB /dev/video1 en vez de CSI
```

## `generate_markers.py` — crea los PNGs para imprimir
El repo no incluye imágenes de marcadores. Genera los IDs/diccionario de
`config/markers_db_small.yaml` (DICT_5X5_250, ids 0,1,2, 100 mm) listos para imprimir.

```bash
python3 generate_markers.py            # -> markers_print/DICT_5X5_250_id{0,1,2}_100mm.png
```

Imprime a **tamaño real** (sin "fit to page") y verifica con regla que el lado
negro mida lo indicado, o la pose saldrá mal.
