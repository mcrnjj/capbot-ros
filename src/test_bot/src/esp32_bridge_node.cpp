// esp32_bridge_node.cpp - puente ROS2 <-> servicio capbot-jetson-bridge.
//
// Port a C++ del nodo equivalente en Python (ver git history de
// scripts/esp32_bridge_node.py). El servicio capbot-jetson-bridge corre en
// OTRO proceso (asyncio, fuera de este workspace) y posee el puerto serie
// hacia el ESP32. Ese servicio expone su propio nodo ROS2
// ('capbot_jetson_bridge', ver ros_bridge.py en ese repo) con dos topicos
// sin tipo propio (std_msgs/Float32MultiArray):
//
//   from_bridge  (lo publica el servicio jetson-bridge, lo suscribimos aqui)
//       [x, y, theta, v, w, setpoint_x, setpoint_y, setpoint_theta]
//       - x, y: metros. v: m/s.
//       - theta, w: ¡EN GRADOS Y GRADOS/S, no radianes! El firmware
//         (capbot-ESP32/lib/Sensors/Odometry.h::StateEstimate) trabaja toda
//         la parte angular en grados; este nodo convierte a radianes antes
//         de publicar a ROS (que exige radianes).
//       - setpoint_x/y/theta: ultima meta de posicion que el host (Nav2, via
//         UDP) le mando al ESP32 para su controlador autonomo on-board; se
//         republica solo a modo de diagnostico/visualizacion. setpoint_theta
//         tambien viene en grados (mismo motivo).
//
//   to_bridge    (lo publicamos aqui, lo suscribe el servicio jetson-bridge)
//       [left, right, stop]
//       - left, right: SOLO tienen significado fisico si el ESP32 esta en
//         modo AUTONOMOUS_NAV (Cfg::MsgType::MODE_CMD = 1). En ese modo
//         (ver capbot-ESP32/src/main.cpp::onMotorCmd) son el setpoint de
//         velocidad para el PID on-board: left = v en mm/s, right = w en
//         decimas de grado/s (deg/s * 10). En MANUAL (modo 0, el default al
//         arrancar) estos mismos campos se interpretan como PWM crudo en
//         vez de velocidad — este nodo NO controla el modo del ESP32 (eso
//         viaja por UDP desde el host/Nav2-bridge, fuera de este topico), asi
//         que /cmd_vel solo mueve el robot si algo externo ya puso al ESP32
//         en AUTONOMOUS_NAV.
//       - stop: != 0 frena ya e ignora left/right (y el firmware vuelve a
//         modo MANUAL, ver onBrake() en el firmware).
//
// Nodo ROS2 normal (un solo hilo, sin asyncio): traduce from_bridge a
// nav_msgs/Odometry (+ TF opcional) para que lo consuma el EKF (ver
// config/ekf.yaml) y a un PoseStamped de diagnostico para el setpoint de
// Nav2, y traduce /cmd_vel (m/s, rad/s) a setpoint de velocidad del ESP32
// (mm/s, decideg/s) para to_bridge, con un watchdog que frena si /cmd_vel
// deja de llegar.

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/transform_broadcaster.h"

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace
{

// Rango crudo del frame MOTOR_CMD (int16 por canal, ver
// capbot-ESP32/include/Config.h::CMD_FULL_SCALE). Sirve como backstop tanto
// si el ESP32 esta en MANUAL (PWM) como en AUTONOMOUS_NAV (mm/s, decideg/s):
// en ambos casos left/right viajan en ese mismo rango de wire.
constexpr double kCmdFullScale = 32767.0;

// Cantidad de floats esperados en from_bridge.
constexpr std::size_t kFromBridgeLen = 8;

// El firmware trabaja la parte angular en grados (Odometry.h::StateEstimate);
// ROS exige radianes.
constexpr double kDegToRad = 0.017453292519943295;
constexpr double kRadToDeg = 57.29577951308232;

// AUTONOMOUS_NAV en el ESP32 espera v en mm/s y w en decimas de grado/s
// (ver Config.h::MsgType::MOTOR_CMD y main.cpp::onMotorCmd).
constexpr double kMetersToMillimeters = 1000.0;
constexpr double kRadPerSecToDeciDegPerSec = kRadToDeg * 10.0;

struct Quat
{
  double x;
  double y;
  double z;
  double w;
};

// yaw en RADIANES.
Quat yawToQuat(double yaw)
{
  const double half = yaw / 2.0;
  return Quat{0.0, 0.0, std::sin(half), std::cos(half)};
}

}  // namespace

class Esp32BridgeNode : public rclcpp::Node
{
public:
  Esp32BridgeNode()
  : Node("esp32_bridge_node")
  {
    declare_parameter<std::string>("odom_frame", "odom");
    declare_parameter<std::string>("base_frame", "base_link");
    declare_parameter<std::string>("map_frame", "map");
    declare_parameter<bool>("publish_odom_tf", false);
    // Clamps de seguridad sobre /cmd_vel antes de mandarlo como setpoint de
    // velocidad al ESP32. Placeholder: ajustar a la velocidad maxima real
    // del robot.
    declare_parameter<double>("max_linear_speed", 0.3);
    declare_parameter<double>("max_angular_speed", 2.0);
    declare_parameter<double>("cmd_vel_timeout", 0.5);
    declare_parameter<double>("twist_linear_variance", 0.01);
    declare_parameter<double>("twist_angular_variance", 0.02);

    pub_odom_ = create_publisher<nav_msgs::msg::Odometry>("odom", 10);
    pub_to_bridge_ = create_publisher<std_msgs::msg::Float32MultiArray>("to_bridge", 10);
    pub_setpoint_echo_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "esp32_bridge/setpoint_echo", 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    sub_from_bridge_ = create_subscription<std_msgs::msg::Float32MultiArray>(
      "from_bridge", 10, std::bind(&Esp32BridgeNode::onFromBridge, this, _1));
    sub_cmd_vel_ = create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10, std::bind(&Esp32BridgeNode::onCmdVel, this, _1));

    watchdog_timer_ = create_wall_timer(
      100ms, std::bind(&Esp32BridgeNode::checkCmdVelTimeout, this));

    bool publish_tf = false;
    double max_linear_speed = 0.0;
    double max_angular_speed = 0.0;
    double cmd_vel_timeout = 0.0;
    get_parameter("publish_odom_tf", publish_tf);
    get_parameter("max_linear_speed", max_linear_speed);
    get_parameter("max_angular_speed", max_angular_speed);
    get_parameter("cmd_vel_timeout", cmd_vel_timeout);
    RCLCPP_INFO(
      get_logger(),
      "Listo. from_bridge -> /odom (tf %s) | /cmd_vel -> to_bridge "
      "(max_linear_speed=%.2f m/s, max_angular_speed=%.2f rad/s, "
      "cmd_vel_timeout=%.2fs). OJO: /cmd_vel solo mueve el robot si el ESP32 "
      "ya esta en modo AUTONOMOUS_NAV (este nodo no cambia el modo).",
      publish_tf ? "habilitado" : "deshabilitado",
      max_linear_speed, max_angular_speed, cmd_vel_timeout);
  }

private:
  // -------------------- from_bridge -> /odom --------------------
  void onFromBridge(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    if (msg->data.size() < kFromBridgeLen) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "from_bridge: se esperaban %zu valores "
        "[x,y,theta,v,w,setpoint_x,setpoint_y,setpoint_theta], llegaron %zu",
        kFromBridgeLen, msg->data.size());
      return;
    }

    const double x = msg->data[0];
    const double y = msg->data[1];
    const double theta_deg = msg->data[2];
    const double v = msg->data[3];
    const double w_deg_s = msg->data[4];
    const double sx = msg->data[5];
    const double sy = msg->data[6];
    const double stheta_deg = msg->data[7];

    const rclcpp::Time stamp = get_clock()->now();
    publishOdom(x, y, theta_deg * kDegToRad, v, w_deg_s * kDegToRad, stamp);
    publishSetpointEcho(sx, sy, stheta_deg * kDegToRad, stamp);
  }

  // theta, w en RADIANES (ya convertidos desde los grados/grados-s del ESP32).
  void publishOdom(
    double x, double y, double theta, double v, double w, const rclcpp::Time & stamp)
  {
    const Quat q = yawToQuat(theta);

    std::string odom_frame;
    std::string base_frame;
    get_parameter("odom_frame", odom_frame);
    get_parameter("base_frame", base_frame);

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = odom_frame;
    odom.child_frame_id = base_frame;
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.orientation.x = q.x;
    odom.pose.pose.orientation.y = q.y;
    odom.pose.pose.orientation.z = q.z;
    odom.pose.pose.orientation.w = q.w;
    odom.twist.twist.linear.x = v;
    odom.twist.twist.angular.z = w;

    double lin_var = 0.01;
    double ang_var = 0.02;
    get_parameter("twist_linear_variance", lin_var);
    get_parameter("twist_angular_variance", ang_var);
    odom.twist.covariance[0] = lin_var;    // vx
    odom.twist.covariance[7] = lin_var;    // vy (robot no-holonomico: ~0 con misma confianza)
    odom.twist.covariance[35] = ang_var;   // vyaw

    pub_odom_->publish(odom);

    bool publish_tf = false;
    get_parameter("publish_odom_tf", publish_tf);
    if (publish_tf) {
      geometry_msgs::msg::TransformStamped tf;
      tf.header.stamp = stamp;
      tf.header.frame_id = odom_frame;
      tf.child_frame_id = base_frame;
      tf.transform.translation.x = x;
      tf.transform.translation.y = y;
      tf.transform.rotation.x = q.x;
      tf.transform.rotation.y = q.y;
      tf.transform.rotation.z = q.z;
      tf.transform.rotation.w = q.w;
      tf_broadcaster_->sendTransform(tf);
    }
  }

  void publishSetpointEcho(double x, double y, double theta, const rclcpp::Time & stamp)
  {
    const Quat q = yawToQuat(theta);

    std::string map_frame;
    get_parameter("map_frame", map_frame);

    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = stamp;
    pose.header.frame_id = map_frame;
    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.orientation.x = q.x;
    pose.pose.orientation.y = q.y;
    pose.pose.orientation.z = q.z;
    pose.pose.orientation.w = q.w;
    pub_setpoint_echo_->publish(pose);
  }

  // -------------------- /cmd_vel -> to_bridge --------------------
  // Pasa derecho a unidades del ESP32 (mm/s, decideg/s): el PID de velocidad
  // y la mezcla diferencial ahora corren on-board (Controlador::computeVelocity),
  // este nodo ya NO hace cinematica de ruedas.
  void onCmdVel(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    last_cmd_vel_time_sec_ = get_clock()->now().seconds();
    has_last_cmd_vel_ = true;
    cmd_vel_stopped_ = false;

    double max_linear = 0.3;
    double max_angular = 2.0;
    get_parameter("max_linear_speed", max_linear);
    get_parameter("max_angular_speed", max_angular);

    const double linear = std::max(-max_linear, std::min(max_linear, msg->linear.x));
    const double angular = std::max(-max_angular, std::min(max_angular, msg->angular.z));

    const double v_mm_s = linear * kMetersToMillimeters;
    const double w_decideg_s = angular * kRadPerSecToDeciDegPerSec;

    publishVelocityCmd(v_mm_s, w_decideg_s, false);
  }

  void publishVelocityCmd(double v_mm_s, double w_decideg_s, bool stop)
  {
    float left_cmd = 0.0f;
    float right_cmd = 0.0f;
    if (!stop) {
      left_cmd = static_cast<float>(
        std::max(-kCmdFullScale, std::min(kCmdFullScale, v_mm_s)));
      right_cmd = static_cast<float>(
        std::max(-kCmdFullScale, std::min(kCmdFullScale, w_decideg_s)));
    }

    std_msgs::msg::Float32MultiArray out;
    out.data = {left_cmd, right_cmd, stop ? 1.0f : 0.0f};
    pub_to_bridge_->publish(out);
  }

  // -------------------- watchdog /cmd_vel --------------------
  void checkCmdVelTimeout()
  {
    if (!has_last_cmd_vel_ || cmd_vel_stopped_) {
      return;
    }

    double timeout = 0.5;
    get_parameter("cmd_vel_timeout", timeout);
    const double elapsed = get_clock()->now().seconds() - last_cmd_vel_time_sec_;
    if (elapsed > timeout) {
      cmd_vel_stopped_ = true;
      publishVelocityCmd(0.0, 0.0, true);
      RCLCPP_WARN(
        get_logger(), "Sin /cmd_vel hace %.2fs (timeout=%.2fs); frenando.", elapsed, timeout);
    }
  }

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_to_bridge_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_setpoint_echo_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_from_bridge_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr sub_cmd_vel_;
  rclcpp::TimerBase::SharedPtr watchdog_timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  double last_cmd_vel_time_sec_{0.0};
  bool has_last_cmd_vel_{false};
  bool cmd_vel_stopped_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Esp32BridgeNode>());
  rclcpp::shutdown();
  return 0;
}
