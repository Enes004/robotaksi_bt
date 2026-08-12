// ============================================================================
// mission_utility_nodes.hpp — Görev Yardımcı Node'ları
//
//   - RecordMissionPoint    : görev noktası kaydı (SyncAction)
//   - RecordParkEntryReached: park girişi kaydı (SyncAction)
//   - SignalPassengerEvent  : yolcu olayı bildirimi (SyncAction)
//
// BAĞIMLILIK: std_msgs (ROS publisher)
// DURUM: ✅ Tam implemente
// ============================================================================
#ifndef MISSION_UTILITY_NODES_HPP
#define MISSION_UTILITY_NODES_HPP

#include "robotaksi_bt/bt_node_base.hpp"
#include "behaviortree_cpp_v3/action_node.h"
#include "std_msgs/msg/string.hpp"
#include <string>

namespace robotaksi_bt {

// ─── RecordMissionPoint (SyncAction) ───
class RecordMissionPoint : public BT::SyncActionNode {
public:
  RecordMissionPoint(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<std::string>("point", "Görev noktası koordinatı"),
      BT::InputPort<double>("tolerance", 1.0, "Tolerans (metre)")
    };
  }

  BT::NodeStatus tick() override;
};

// ─── RecordParkEntryReached (SyncAction) ───
class RecordParkEntryReached : public BT::SyncActionNode {
public:
  RecordParkEntryReached(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override;
};

// ─── SignalPassengerEvent (SyncAction) ───
class SignalPassengerEvent : public BT::SyncActionNode {
public:
  SignalPassengerEvent(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<std::string>("event_type", "pickup veya dropoff")
    };
  }

  BT::NodeStatus tick() override;
private:
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
};

}  // namespace robotaksi_bt

#endif  // MISSION_UTILITY_NODES_HPP
