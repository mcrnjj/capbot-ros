# capbot-ros — Stack autónomo ROS2 para Jetson Nano (Eloquent)

Port del paquete `test_bot` (originalmente **ROS2 Foxy / Ubuntu 20.04**, en el
directorio hermano `test_bot_ROS`) a la **Jetson Nano con JetPack 4.x
(Ubuntu 18.04) usando ROS2 Eloquent**.

Es un **workspace colcon** (`src/test_bot/`) pensado para `colcon build` directo
en la Jetson. Cubre **solo el robot real** (sin Gazebo/simulación).

> Robot del Capstone. Ecosistema relacionado: `capbot-ESP32` (firmware),
> `capbot-jetson-bridge` (teleop) y `capbot-host` (app PC).

---

## Por qué Eloquent (y no Dashing ni Foxy nativo)

- La Jetson Nano con JetPack 4.x trae **Ubuntu 18.04 + Python 3.6**.
- **Foxy** exige Python 3.8 / Ubuntu 20.04 → no es nativo en JetPack 4.x.
- **Eloquent** usa Python 3.6 (coincide con JetPack y con `capbot-jetson-bridge`,
  que ya está escrito para 3.6) y, a diferencia de **Dashing**, **sí incluye
  `nav2_waypoint_follower`** y una arquitectura NAV2 más cercana a Foxy.

> Ambos (Dashing/Eloquent) están EOL. Para un robot offline fijo es aceptable.
> Alternativa no elegida: imagen comunitaria Ubuntu 20.04 + Foxy nativo en la Nano.

---

## Matriz de compatibilidad (Foxy → Eloquent)

| Componente | Estado | Notas |
|---|---|---|
| `aruco_localizer.py` | ✅ Portado tal cual | Ya detecta APIs de OpenCV en runtime. OpenCV 4.1.1 de JetPack OK. Requiere `pip3 install numpy scipy`. |
| `robot_localization` (EKF) | ✅ Adaptado | `ekf.yaml` simplificado: fusiona `/odom` + `/aruco_pose`. Sin `/imu/data` (ver abajo). |
| Cámara CSI (IMX219) | ✅ Nodo nuevo | `csi_camera_node.py` usa `nvarguscamerasrc` vía OpenCV/GStreamer. `v4l2_camera` NO sirve para CSI. |
| Puente ESP32 | ✅ Nodo nuevo | `esp32_bridge_node.py`: COBS+CRC16 embebido. Publica `/odom`, manda `/cmd_vel`→`VEL_CMD`. |
| Launch | ✅ Adaptado | Sintaxis **Eloquent**: `node_executable` / `node_name` (no `executable`/`name`). |
| **NAV2** | ⚠️ **Validar** | `nav2_params.yaml` conserva el tuning de Foxy; la *estructura* de plugins puede diferir en Eloquent. Ver "Validación pendiente". |

---

## Arquitectura

```
            CSI (IMX219)                 Serie COBS+CRC16 /dev/ttyTHS1
   ┌──────────────────────┐      ┌──────────────────────────────┐
   │ csi_camera_node       │      │ esp32_bridge_node             │
   │  /camera/image_raw    │      │  TELEMETRY(JSON) -> /odom     │
   │  /camera/camera_info  │      │  /cmd_vel -> VEL_CMD (0x16)   │
   └──────────┬───────────┘      └───────────┬──────────────────┘
              │                               │
        ┌─────▼─────┐                  ┌──────▼───────┐
        │ aruco_     │ /aruco_pose     │ ekf_odom     │  TF odom->base_link
        │ localizer  ├────────────┐    │ (local)      │
        └────────────┘            │    └──────────────┘
                                  │    ┌──────────────┐
                                  └───►│ ekf_map      │  TF map->odom
                              /odom ──►│ (global)     │  (corrige deriva c/ ArUco)
                                       └──────┬───────┘
                                              │ map->odom->base_link
                                       ┌──────▼───────────────────────────┐
                                       │ NAV2: planner -> controller(DWB)  │
                                       │   -> /cmd_vel ────────────────────┘
                                       └───────────────────────────────────┘
```

**Control:** NAV2 produce `/cmd_vel` (v, w). El ESP32 originalmente solo acepta
PWM crudo (manual) o setpoints de **posición** (autónomo). Para cerrar el lazo con
NAV2 se añade un comando de **velocidad** al firmware (ver siguiente sección).

---

## ⚠️ Cambio de firmware requerido en `capbot-ESP32` (VEL_CMD 0x16)

`esp32_bridge_node.py` ya envía un frame nuevo que **el firmware actual no maneja**:

```
MsgType VEL_CMD = 0x16
payload = <ff>  (little-endian): float32 v [m/s], float32 w [rad/s]
```

Mientras el firmware no lo implemente, el ESP32 **descarta** el frame de forma
segura (el framing COBS+CRC valida y el `default:` de `dispatchFrame()` ignora
tipos desconocidos sin corromper el stream). Es decir, **el puente es seguro de
correr ya**, pero el robot no se moverá en autónomo hasta el cambio.

**Qué hay que hacer en el firmware** (yo NO lo he tocado; requiere tu OK y lo haría
en una branch aparte):
1. `include/Config.h`: añadir `constexpr uint8_t VEL_CMD = 0x16;` en `MsgType`.
2. `lib/Link/JetsonLink.*`: callback `onVelCmd(float v, float w)`; en
   `dispatchFrame()` un `case Cfg::MsgType::VEL_CMD` que haga
   `memcpy` de 2 float y dispare el callback (n>=8).
3. `src/main.cpp`: en `onVelCmd`, fijar setpoint de velocidad y hacer que el
   control use los PID de velocidad ya existentes (`linearVelPid`, `angularVelPid`)
   en vez del lazo de posición. Lo más limpio: un modo "velocidad" (p.ej.
   `MODE_CMD` con un valor nuevo, o autoseleccionar al recibir VEL_CMD) que llame
   a la rama de velocidad de `Controlador` saltándose el PID de posición.

> Conversión de unidades: el firmware trabaja `theta` en grados y `omega` en
> grados/s; el puente manda `w` en **rad/s**. Decide en qué capa conviertes
> (recomendado: convertir a grados/s dentro del firmware al recibir VEL_CMD).

---

## EKF: por qué no hay `/imu/data`

La telemetría del ESP32 (`SensorHub::buildPayload`) **muestrea la IMU pero solo
transmite odometría ya fusionada** (`odo: {x, y, a, v, w}`). Por eso el puente
publica únicamente `/odom` y el `ekf.yaml` fusiona `/odom` + `/aruco_pose`.

Si en el futuro quieres fusión IMU+encoders en la Jetson, hay que **añadir los
campos crudos de IMU al `buildPayload` del firmware** (otro cambio en
`capbot-ESP32`) y reactivar `imu0` en `ekf.yaml`.

---

## Estructura

```
capbot-ros/
├── README.md                      <- este archivo
└── src/test_bot/
    ├── package.xml                (deps Eloquent)
    ├── CMakeLists.txt             (instala scripts + assets)
    ├── scripts/
    │   ├── aruco_localizer.py     (portado; ya portable)
    │   ├── csi_camera_node.py     (NUEVO; nvarguscamerasrc)
    │   └── esp32_bridge_node.py   (NUEVO; COBS <-> /odom, /cmd_vel)
    ├── config/
    │   ├── ekf.yaml               (simplificado)
    │   ├── nav2_params.yaml       (Eloquent; VALIDAR)
    │   ├── camera.yaml            (placeholder; RECALIBRAR)
    │   ├── markers_db_small.yaml  / markers_db_large.yaml
    │   └── test_map_*.{yaml,pgm,png}
    ├── description/
    │   ├── robot_real.urdf.xacro  (NUEVO; sin lidar/gazebo)
    │   └── *.xacro                (de test_bot_ROS)
    ├── launch/robot.launch.py     (Eloquent)
    └── maps/
```

---

## Build (en la Jetson)

```bash
# Dependencias Python no cubiertas por rosdep:
pip3 install numpy scipy pyserial

# OpenCV debe tener GStreamer (el de JetPack lo trae):
python3 -c "import cv2; print(cv2.getBuildInformation())" | grep -i gstreamer

cd capbot-ros
rosdep install --from-paths src --ignore-src -r -y   # opcional/parcial en Eloquent
colcon build --symlink-install
source install/setup.bash
```

## Run

```bash
# Stack completo (localización + NAV2):
ros2 launch test_bot robot.launch.py

# Solo localización (recomendado para primer bringup):
ros2 launch test_bot robot.launch.py enable_nav:=false

# Otro mapa / puerto serie:
ros2 launch test_bot robot.launch.py map_name:=large serial_port:=/dev/ttyUSB0
```

Permisos del puerto serie: `sudo usermod -aG dialout $USER` (y re-login). En la
Jetson `/dev/ttyTHS1` puede requerir liberar la consola serie (`nvgetty`).

---

## Validación pendiente (antes de confiar en autónomo)

1. **NAV2 / Eloquent**: `diff` de `config/nav2_params.yaml` contra
   `/opt/ros/eloquent/share/nav2_bringup/params/nav2_params.yaml`. Revisar
   `progress_checker_plugin` / `goal_checker_plugin`, `planner_plugins`,
   `recoveries_server` y la ruta/nombre del BT XML del `bt_navigator`.
2. **Calibración de cámara**: `config/camera.yaml` es nominal. Recalibrar a 640×480
   con `camera_calibration` o ArUco fallará en precisión.
3. **Firmware VEL_CMD** (sección arriba).
4. **Coexistencia con `capbot-jetson-bridge`**: ambos abren el **mismo serie**
   `/dev/ttyTHS1` y la **misma cámara CSI**. No correr los dos a la vez sobre los
   mismos recursos: en modo autónomo usa este stack ROS; en teleop, el bridge.
5. **TF única**: el EKF local publica `odom->base_link`; por eso el puente lleva
   `publish_odom_tf:=false`. No actives ambos.

---

## Estado

Hecho: estructura, package/build, nodos (aruco, cámara CSI, puente ESP32), EKF,
nav2_params (a validar), URDF real, launch.

Pendiente: cambio de firmware VEL_CMD (en `capbot-ESP32`, con tu OK y en branch),
validación NAV2 en Eloquent, calibración de cámara, pruebas en hardware.
