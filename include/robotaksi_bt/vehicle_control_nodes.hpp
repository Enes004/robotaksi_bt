// ============================================================================
// vehicle_control_nodes.hpp — Araç Kontrol Node'ları
//
// Aracın fiziksel kontrolüne dair temel node'lar:
//   - SetMaxSpeed     : hız limiti ayarla (SyncAction)
//   - StopVehicle     : tam durdur (StatefulAction, opsiyonel hold_sec portu)
//   - TurnHeadlights  : far aç/kapat (SyncAction)
//   - Dwell           : belirli süre bekle (StatefulAction)
//
// BAĞIMLILIK: std_msgs (ROS2 publisher için)
// DURUM: ✅ Tam implemente — ROS topic publish ile gerçek çalışırfi
//
// BT.CPP v3 uyumlu.
// ============================================================================
#ifndef VEHICLE_CONTROL_NODES_HPP
#define VEHICLE_CONTROL_NODES_HPP

#include "robotaksi_bt/bt_node_base.hpp"
#include "behaviortree_cpp_v3/action_node.h"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/bool.hpp"
#include <string>
#include <chrono>
#include <optional>

namespace robotaksi_bt {

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
// StopVehicle (StatefulAction)
//
// Aracı tamamen durdurur — /cmd_vel_bt'ye sıfır Twist yayınlar.
// (twist_mux mimarisi: BT → /cmd_vel_bt, Nav2 → /cmd_vel_nav, mux → /cmd_vel)
// Duruşu DOĞRULAMAZ — bunu CheckStopAccuracy yapar.
//
// hold_sec portu:
//   hold_sec <= 0 (varsayılan) → bir kez Twist yayınla, hemen SUCCESS dön
//                                 (eski SyncAction davranışı — Sequence'ler bozulmaz)
//   hold_sec  > 0              → o kadar saniye boyunca her tick'te tekrar
//                                 Twist yayınla (RUNNING), süre dolunca SUCCESS
//                                 XML: <StopVehicle hold_sec="20.0" />
// ─────────────────────────────────────────────
class StopVehicle : public BT::StatefulActionNode {
public:
  StopVehicle(const std::string& name, const BT::NodeConfiguration& config)
    : BT::StatefulActionNode(name, config) {}

  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<double>("hold_sec")
    };
  }

  BT::NodeStatus onStart()   override;
  BT::NodeStatus onRunning() override;
  void           onHalted()  override;

private:
  /// Lazy-init publisher — ilk onStart'ta oluşturulur.
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;

  /// hold_sec > 0 durumunda tutulur; onStart'ta okunur.
  double hold_sec_ = 0.0;

  /// Zamanlayıcı — onStart'ta başlatılır.
  std::chrono::steady_clock::time_point start_time_;

  /// Twist publish yardımcısı — hem onStart hem onRunning kullanır.
  void publishZeroVel();
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

}  // namespace robotaksi_bt

#endif  // VEHICLE_CONTROL_NODES_HPP
