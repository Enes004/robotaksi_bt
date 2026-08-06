// ============================================================================
// mission_utility_nodes.cpp — Görev Yardımcı Node İmplementasyonları
// ============================================================================

#include "robotaxi_bt/mission_utility_nodes.hpp"

namespace robotaxi_bt {

// ═══════════════════════════════════════════════════════════════
// RecordMissionPoint::tick()
//
// SafeDrive içinde çağrılır → TÜM segment tiplerinde çalışır.
// Şartname: her görev noktasına ulaşmak +30 puan.
//
// Blackboard'da "mission_points_reached" sayacını artırır.
// İleride: /odom'dan mevcut pozisyonu alıp point ile karşılaştırarak
// tolerans kontrolü yapılacak.
// ═══════════════════════════════════════════════════════════════
BT::NodeStatus RecordMissionPoint::tick()
{
  std::string point;
  double tolerance = 1.0;
  getInput("point", point);
  getInput("tolerance", tolerance);

  // Sayaç artır
  int count = 0;
  config().blackboard->get("mission_points_reached", count);
  config().blackboard->set("mission_points_reached", count + 1);

  RCLCPP_INFO(btLogger(), "RecordMissionPoint: nokta=%s, toplam=%d",
              point.c_str(), count + 1);

  return BT::NodeStatus::SUCCESS;
}

// ═══════════════════════════════════════════════════════════════
// RecordParkEntryReached::tick()
//
// Park alanı girişine ulaşıldığında çağrılır.
// Şartname: park girişi +20 puan.
// ═══════════════════════════════════════════════════════════════
BT::NodeStatus RecordParkEntryReached::tick()
{
  config().blackboard->set("park_entry_reached", true);
  RCLCPP_INFO(btLogger(), "RecordParkEntryReached: park girişi kaydedildi (+20)");
  return BT::NodeStatus::SUCCESS;
}

// ═══════════════════════════════════════════════════════════════
// SignalPassengerEvent::tick()
//
// Yolcu al/bırak olayını hem Blackboard'a hem ROS topic'e yazar.
// seg_meta'dan gelen event_type: "pickup" veya "dropoff"
// ═══════════════════════════════════════════════════════════════
BT::NodeStatus SignalPassengerEvent::tick()
{
  std::string event_type;
  if (!getInput("event_type", event_type)) {
    RCLCPP_ERROR(btLogger(), "SignalPassengerEvent: 'event_type' okunamadı!");
    return BT::NodeStatus::FAILURE;
  }

  config().blackboard->set("last_passenger_event", event_type);

  // Lazy-init: publisher'ı ilk tick'te oluştur
  if (!pub_) {
    auto ros = getRosNode(config());
    if (ros) {
      pub_ = ros->create_publisher<std_msgs::msg::String>("/mission/passenger_event", 10);
    }
  }
  if (pub_) {
    std_msgs::msg::String msg;
    msg.data = event_type;
    pub_->publish(msg);
  }

  RCLCPP_INFO(btLogger(), "SignalPassengerEvent: '%s' olayı yayınlandı", event_type.c_str());
  return BT::NodeStatus::SUCCESS;
}

}  // namespace robotaxi_bt
