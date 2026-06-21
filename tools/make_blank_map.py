#!/usr/bin/env python3
"""
make_blank_map.py  -  genera un mapa nav2 (map_server) vacio para pruebas.
===========================================================================

Crea una imagen PGM (formato P5, escrita a mano, sin PIL/numpy) toda libre
con un borde ocupado opcional, mas el .yaml que espera nav2_map_server.
Pensado para probar el bringup completo (robot.launch.py, esp32_bridge_node,
EKF, NAV2) sin necesitar un mapa real del entorno.

Tambien genera un markers_db_<name>.yaml vacio (markers: []): aruco_localizer
lo exige al iniciar (sys.exit fatal si falta) y robot.launch.py SIEMPRE lo
lanza (no esta condicionado a enable_nav), asi que sin ese archivo el
bringup no arranca aunque el mapa este bien. Con la lista vacia el nodo
arranca igual, solo que /aruco_pose nunca publica (sin correccion de deriva
del EKF global).

Uso:
  python3 make_blank_map.py                            # 6x6 m, res 0.05, borde 2 px
  python3 make_blank_map.py --name blank --width-m 8 --height-m 8
  python3 make_blank_map.py --border-cells 0            # sin paredes (todo libre)
  python3 make_blank_map.py --out-dir /ruta/a/config --force

Luego:
  ros2 launch test_bot robot.launch.py map_name:=blank
  ros2 launch test_bot robot.launch.py map_name:=blank enable_nav:=false   # solo localizacion
"""
import argparse
import os
import sys

# Convencion map_server con negate=0: blanco=libre, negro=ocupado, gris 205=desconocido.
_OCCUPIED_THRESH = 0.65
_FREE_THRESH = 0.196
_FREE_VALUE = 255
_OCCUPIED_VALUE = 0


def build_grid(width_px: int, height_px: int, border_cells: int) -> bytes:
    """Grilla de bytes (1 byte/pixel, fila por fila) con borde ocupado opcional."""
    data = bytearray(width_px * height_px)
    border = max(0, border_cells)
    for y in range(height_px):
        row_off = y * width_px
        is_h_wall = y < border or y >= height_px - border
        for x in range(width_px):
            if is_h_wall or x < border or x >= width_px - border:
                data[row_off + x] = _OCCUPIED_VALUE
            else:
                data[row_off + x] = _FREE_VALUE
    return bytes(data)


def write_pgm(path: str, width_px: int, height_px: int, data: bytes) -> None:
    with open(path, "wb") as f:
        f.write(b"P5\n%d %d\n255\n" % (width_px, height_px))
        f.write(data)


def write_map_yaml(path: str, image_name: str, resolution: float,
                    origin_x: float, origin_y: float) -> None:
    with open(path, "w") as f:
        f.write(
            "image: {image}\n"
            "resolution: {res}\n"
            "origin: [{ox}, {oy}, 0.0]\n"
            "negate: 0\n"
            "occupied_thresh: {occ}\n"
            "free_thresh: {free}\n".format(
                image=image_name, res=resolution, ox=origin_x, oy=origin_y,
                occ=_OCCUPIED_THRESH, free=_FREE_THRESH,
            )
        )


def write_empty_markers_db(path: str) -> None:
    with open(path, "w") as f:
        f.write("aruco_dict: DICT_5X5_250\nmarker_size: 0.1\nmarkers: []\n")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                  formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--name", default="blank",
                     help="nombre del mapa -> map_name:=<name> (def 'blank')")
    ap.add_argument("--width-m", type=float, default=6.0, help="ancho en metros (def 6.0)")
    ap.add_argument("--height-m", type=float, default=6.0, help="alto en metros (def 6.0)")
    ap.add_argument("--resolution", type=float, default=0.05, help="metros/pixel (def 0.05)")
    ap.add_argument("--border-cells", type=int, default=2,
                     help="ancho del borde ocupado en pixeles, 0 = sin paredes (def 2)")
    ap.add_argument("--out-dir", default=None,
                     help="carpeta destino (def src/test_bot/config de este repo)")
    ap.add_argument("--no-markers-db", action="store_true",
                     help="no generar markers_db_<name>.yaml")
    ap.add_argument("--force", action="store_true", help="sobrescribir archivos existentes")
    args = ap.parse_args()

    if args.width_m <= 0 or args.height_m <= 0 or args.resolution <= 0:
        print("[X] --width-m/--height-m/--resolution deben ser > 0")
        sys.exit(1)

    out_dir = args.out_dir or os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "..", "src", "test_bot", "config")
    out_dir = os.path.normpath(out_dir)
    os.makedirs(out_dir, exist_ok=True)

    width_px = max(1, round(args.width_m / args.resolution))
    height_px = max(1, round(args.height_m / args.resolution))

    pgm_name = "test_map_%s.pgm" % args.name
    pgm_path = os.path.join(out_dir, pgm_name)
    yaml_path = os.path.join(out_dir, "test_map_%s.yaml" % args.name)

    for p in (pgm_path, yaml_path):
        if os.path.exists(p) and not args.force:
            print("[X] %s ya existe (usa --force para sobrescribir)" % p)
            sys.exit(1)

    print("[i] Generando grilla %dx%d px (%.1fx%.1f m @ %.3f m/px, borde=%d px)..."
          % (width_px, height_px, args.width_m, args.height_m, args.resolution,
             args.border_cells))
    data = build_grid(width_px, height_px, args.border_cells)
    write_pgm(pgm_path, width_px, height_px, data)
    print("[OK] Imagen escrita : %s" % pgm_path)

    origin_x = -args.width_m / 2.0
    origin_y = -args.height_m / 2.0
    write_map_yaml(yaml_path, pgm_name, args.resolution, origin_x, origin_y)
    print("[OK] Yaml escrito   : %s  (origin=[%.3f, %.3f, 0.0])"
          % (yaml_path, origin_x, origin_y))

    if not args.no_markers_db:
        markers_path = os.path.join(out_dir, "markers_db_%s.yaml" % args.name)
        if os.path.exists(markers_path) and not args.force:
            print("[!] %s ya existe, no se toca (usa --force para sobrescribir)" % markers_path)
        else:
            write_empty_markers_db(markers_path)
            print("[OK] markers_db vacio: %s" % markers_path)
            print("     aruco_localizer arrancara, pero /aruco_pose nunca publica")
            print("     (sin marcadores no hay correccion de deriva del EKF global).")

    print("\n==== LISTO ====")
    print("Probar con:")
    print("  ros2 launch test_bot robot.launch.py map_name:=%s" % args.name)
    print("  ros2 launch test_bot robot.launch.py map_name:=%s enable_nav:=false"
          % args.name)


if __name__ == "__main__":
    main()
