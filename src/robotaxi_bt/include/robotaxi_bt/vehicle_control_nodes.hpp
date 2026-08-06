// ============================================================================
// vehicle_control_nodes.hpp — Araç Kontrol Node'ları
//
// Aracın fiziksel kontrolüne dair temel node'lar:
//   - SetMaxSpeed     : hız limiti ayarla (SyncAction)
//   - StopVehicle     : tam durdur (SyncAction)
//   - TurnHeadlights  : far aç/kapat (SyncAction)
//   - Dwell           : belirli süre bekle (StatefulAction)
//
// BAĞIMLILIK: std_msgs (ROS2 publisher için)
// DURUM: ✅ Tam implemente — ROS topic publish ile gerçek çalışır.
//
// BT.CPP v3 uyumlu.
// ============================================================================
#ifndef VEHICLE_CONTROL_NODES_HPP
#define VEHICLE_CONTROL_NODES_HPP

#include "robotaxi_bt/bt_node_base.hpp"
#include "behaviortree_cpp_v3/action_node.h"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/bool.hpp"
#include <string>
#include <chrono>

namespace robotaxi_bt {

// ─────────────────────────────────────────────
// SetMaxSpeed (SyncAction)
//
// Aracın maksimum hız limitini ayarlar.
// Blackboard'a kaydeder + /vehicle/max_speed topic'ine publish eder.
// ─────────────────────────────────────────────
class SetMaxSpeed : public BT::SyncActionNode {
public:
  SetMaxSpeed(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<double>("speed", "Hedef hız m/s cinsinden")
    };
  }

  BT::NodeStatus tick() override;
private:
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_;
};

// ─────────────────────────────────────────────
// StopVehicle (SyncAction)
//
// Aracı tamamen durdurur — /cmd_vel'e sıfır Twist yayınlar.
// Duruşu DOĞRULAMAZ — bunu CheckStopAccuracy yapar.
// ─────────────────────────────────────────────
class StopVehicle : public BT::SyncActionNode {
public:
  StopVehicle(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override;
private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
};

// ─────────────────────────────────────────────
// TurnHeadlights (SyncAction)
//
// Far aç/kapat. Tünel segmentinde kullanılır.
// Şartname: tünelde far açılmazsa -50 puan.
// ─────────────────────────────────────────────
class TurnHeadlights : public BT::SyncActionNode {
public:
  TurnHeadlights(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<std::string>("state", "on veya off")
    };
  }

  BT::NodeStatus tick() override;
private:
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_;
};

// ─────────────────────────────────────────────
// Dwell (StatefulAction)
//
// min_sec ile max_sec arasında bekleme yapar.
// Yolcu durakları için 15-20 saniye bekleme gerekli (şartname).
//
// StatefulAction yaşam döngüsü:
//   tick #1 → onStart() → RUNNING (zamanlayıcı başlar)
//   tick #2..N → onRunning() → RUNNING (süre dolmadı)
//   tick #N+1 → onRunning() → SUCCESS (süre doldu)
//   (iptal) → onHalted() → temizlik
// ─────────────────────────────────────────────
class Dwell : public BT::StatefulActionNode {
public:
  Dwell(const std::string& name, const BT::NodeConfiguration& config)
    : BT::StatefulActionNode(name, config) {}

  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<double>("min_sec", "Minimum bekleme süresi (saniye)"),
      BT::InputPort<double>("max_sec", "Maksimum bekleme süresi (saniye)")
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  std::chrono::steady_clock::time_point start_time_;
  double wait_duration_sec_ = 15.0;
};

}  // namespace robotaxi_bt

#endif  // VEHICLE_CONTROL_NODES_HPP
