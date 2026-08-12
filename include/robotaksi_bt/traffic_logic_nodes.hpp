// ============================================================================
// traffic_logic_nodes.hpp — Trafik Mantık Node'ları (Saf Mantık)
//
// Trafik ışığı/tabela ile ilgili SENSOR GEREKTIRMEYEN saf mantık node'ları:
//   - IsLightRed       : ışık kırmızı mı? (Condition)
//   - IsRoadSignType   : levha tipi kontrolü (Condition)
//   - StopAndProceed   : DUR tabelası → dur + devam (SyncAction)
//   - LogUnknownSign   : bilinmeyen levhayı logla (SyncAction)
//
// BAĞIMLILIK: YOK — Saf mantık + Blackboard
// DURUM: ✅ Tam implemente
//
// NOT: TrafficLightAhead, StopSignAhead, GlobalRoadSignAhead gibi
//      ALGILAMA node'ları burada DEĞİL — onlar sensör bağımlı (stub_nodes).
// ============================================================================
#ifndef TRAFFIC_LOGIC_NODES_HPP
#define TRAFFIC_LOGIC_NODES_HPP

#include "robotaksi_bt/bt_node_base.hpp"
#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/condition_node.h"
#include <string>

namespace robotaksi_bt {

// ─── IsLightRed (Condition) ───
class IsLightRed : public BT::ConditionNode {
public:
  IsLightRed(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config) {}

  static BT::PortsList providedPorts() {
    return { BT::InputPort<std::string>("color", "Işık rengi stringi") };
  }

  BT::NodeStatus tick() override;
};

// ─── IsRoadSignType (Condition) ───
class IsRoadSignType : public BT::ConditionNode {
public:
  IsRoadSignType(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config) {}

  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<std::string>("sign_type", "Algılanan levha tipi"),
      BT::InputPort<std::string>("expected", "Beklenen tip (LANE_MERGE, NO_ENTRY, ...)")
    };
  }

  BT::NodeStatus tick() override;
};

// ─── StopAndProceed (SyncAction) ───
class StopAndProceed : public BT::SyncActionNode {
public:
  StopAndProceed(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override;
};

// ─── LogUnknownSign (SyncAction) ───
class LogUnknownSign : public BT::SyncActionNode {
public:
  LogUnknownSign(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts() {
    return { BT::InputPort<std::string>("sign_type", "Tanımsız levha tipi") };
  }

  BT::NodeStatus tick() override;
};

}  // namespace robotaksi_bt

#endif  // TRAFFIC_LOGIC_NODES_HPP
