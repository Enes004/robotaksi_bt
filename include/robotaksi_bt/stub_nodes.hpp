// ============================================================================
// stub_nodes.hpp — Henüz İmplemente Edilmemiş Node'lar (Stub)
//
// Bu dosyadaki node'lar ŞU AN çalışan ama GERÇEK OLMAYAN stub'lardır.
// Her biri "TODO" ile işaretlenmiş gerçek implementasyon gereksinimleri içerir.
//
// STUB = derlenir, çalışır, ama gerçek sensör/harita verisi KULLANMAZ.
//
// KATEGORİLER:
//   🔧 SENSÖR BAĞIMLI — Algılama takımından topic gelecek
//   🗺️ HARİTA BAĞIMLI — Segment haritası gelince implemente edilecek
//   ⚡ NAV2 BAĞIMLI   — Nav2 action server entegrasyonu gerekli
//
// BT.CPP v3 uyumlu.
// ============================================================================
#ifndef STUB_NODES_HPP
#define STUB_NODES_HPP

#include "robotaksi_bt/bt_node_base.hpp"
#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/condition_node.h"
#include "robotaksi_bt/segment_graph.hpp"
#include <string>
#include <chrono>

namespace robotaksi_bt {

// ═══════════════ 🗺️ HARİTA BAĞIMLI ═══════════════

class LoadMission : public BT::SyncActionNode {
public:
  LoadMission(const std::string& n, const BT::NodeConfiguration& c) : BT::SyncActionNode(n,c) {}
  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<std::string>("geojson_file"), BT::InputPort<int>("tour"),
      BT::OutputPort<std::string>("route"), BT::OutputPort<int>("route_size"),
      BT::OutputPort<int>("seg_index")
    };
  }
  BT::NodeStatus tick() override;
private:
  bool graph_loaded_ = false;  // dosyalar sadece ilk tick'te yüklensin (globalSegmentGraph)
};

class GetCurrentSegment : public BT::SyncActionNode {
public:
  GetCurrentSegment(const std::string& n, const BT::NodeConfiguration& c) : BT::SyncActionNode(n,c) {}
  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<std::string>("route"), BT::InputPort<int>("seg_index"),
      BT::OutputPort<std::string>("seg_type"), BT::OutputPort<std::string>("seg_goal"),
      BT::OutputPort<std::string>("seg_meta")
    };
  }
  BT::NodeStatus tick() override;
};

class ReplanRoute : public BT::SyncActionNode {
public:
  ReplanRoute(const std::string& n, const BT::NodeConfiguration& c) : BT::SyncActionNode(n,c) {}
  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<std::string>("reason"),
      BT::OutputPort<std::string>("route"), BT::OutputPort<int>("route_size"),
      BT::OutputPort<int>("seg_index")
    };
  }
  BT::NodeStatus tick() override;
};

class CalculateLaneChange : public BT::SyncActionNode {
public:
  CalculateLaneChange(const std::string& n, const BT::NodeConfiguration& c) : BT::SyncActionNode(n,c) {}
  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<std::string>("current_pose"), BT::InputPort<std::string>("target_lane"),
      BT::InputPort<std::string>("detected_signs"), BT::OutputPort<std::string>("seg_goal")
    };
  }
  BT::NodeStatus tick() override;
};

class FindParkingSlot : public BT::SyncActionNode {
public:
  FindParkingSlot(const std::string& n, const BT::NodeConfiguration& c) : BT::SyncActionNode(n,c) {}
  static BT::PortsList providedPorts() {
    return { BT::OutputPort<std::string>("slot_pose") };
  }
  BT::NodeStatus tick() override;
};

// ═══════════════ 🔧 SENSÖR BAĞIMLI ═══════════════

class PedestrianAhead : public BT::ConditionNode {
public:
  PedestrianAhead(const std::string& n, const BT::NodeConfiguration& c) : BT::ConditionNode(n,c) {}
  static BT::PortsList providedPorts() { return { BT::InputPort<bool>("detected") }; }
  BT::NodeStatus tick() override;
};

class DynamicObstacleAhead : public BT::ConditionNode {
public:
  DynamicObstacleAhead(const std::string& n, const BT::NodeConfiguration& c) : BT::ConditionNode(n,c) {}
  static BT::PortsList providedPorts() { return { BT::InputPort<bool>("detected") }; }
  BT::NodeStatus tick() override;
};

class StaticObstacleInLane : public BT::ConditionNode {
public:
  StaticObstacleInLane(const std::string& n, const BT::NodeConfiguration& c) : BT::ConditionNode(n,c) {}
  static BT::PortsList providedPorts() { return { BT::InputPort<bool>("detected") }; }
  BT::NodeStatus tick() override;
};

class AvoidanceSpaceAvailable : public BT::ConditionNode {
public:
  AvoidanceSpaceAvailable(const std::string& n, const BT::NodeConfiguration& c) : BT::ConditionNode(n,c) {}
  static BT::PortsList providedPorts() { return { BT::InputPort<std::string>("free_lane") }; }
  BT::NodeStatus tick() override;
};

class IsTwoWayRoad : public BT::ConditionNode {
public:
  IsTwoWayRoad(const std::string& n, const BT::NodeConfiguration& c) : BT::ConditionNode(n,c) {}
  static BT::PortsList providedPorts() { return { BT::InputPort<bool>("is_two_way") }; }
  BT::NodeStatus tick() override;
};

class TrafficLightAhead : public BT::ConditionNode {
public:
  TrafficLightAhead(const std::string& n, const BT::NodeConfiguration& c) : BT::ConditionNode(n,c) {}
  static BT::PortsList providedPorts() { return { BT::OutputPort<std::string>("light_color") }; }
  BT::NodeStatus tick() override;
};

class StopSignAhead : public BT::ConditionNode {
public:
  StopSignAhead(const std::string& n, const BT::NodeConfiguration& c) : BT::ConditionNode(n,c) {}
  static BT::PortsList providedPorts() { return { BT::InputPort<std::string>("detected") }; }
  BT::NodeStatus tick() override;
};

class GlobalRoadSignAhead : public BT::ConditionNode {
public:
  GlobalRoadSignAhead(const std::string& n, const BT::NodeConfiguration& c) : BT::ConditionNode(n,c) {}
  static BT::PortsList providedPorts() { return { BT::OutputPort<std::string>("sign_type") }; }
  BT::NodeStatus tick() override;
};

class TurnConflictsWithSigns : public BT::ConditionNode {
public:
  TurnConflictsWithSigns(const std::string& n, const BT::NodeConfiguration& c) : BT::ConditionNode(n,c) {}
  static BT::PortsList providedPorts() {
    return { BT::InputPort<std::string>("planned_turn"), BT::InputPort<std::string>("detected_signs") };
  }
  BT::NodeStatus tick() override;
};

class CheckStopAccuracy : public BT::ConditionNode {
public:
  CheckStopAccuracy(const std::string& n, const BT::NodeConfiguration& c) : BT::ConditionNode(n,c) {}
  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<std::string>("point"), BT::InputPort<double>("tolerance"),
      BT::InputPort<double>("heading_tolerance")
    };
  }
  BT::NodeStatus tick() override;
};

class IsStuck : public BT::ConditionNode {
public:
  IsStuck(const std::string& n, const BT::NodeConfiguration& c) : BT::ConditionNode(n,c) {}
  static BT::PortsList providedPorts() { return {}; }
  BT::NodeStatus tick() override;
private:
  bool stuck_active_ = false;
  std::chrono::steady_clock::time_point stuck_since_;
};

// ═══════════════ ⚡ NAV2 / HAREKET BAĞIMLI ═══════════════

class FollowLaneSegment : public BT::StatefulActionNode {
public:
  FollowLaneSegment(const std::string& n, const BT::NodeConfiguration& c) : BT::StatefulActionNode(n,c) {}
  static BT::PortsList providedPorts() {
    return { BT::InputPort<std::string>("goal"), BT::InputPort<double>("max_speed", "0") };
  }
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;
private:
  std::chrono::steady_clock::time_point start_time_;
};

class WaitForClear : public BT::StatefulActionNode {
public:
  WaitForClear(const std::string& n, const BT::NodeConfiguration& c) : BT::StatefulActionNode(n,c) {}
  static BT::PortsList providedPorts() {
    return { BT::InputPort<std::string>("hazard") };
  }
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;
private:
  std::chrono::steady_clock::time_point start_time_;
};

class WaitForGreenLight : public BT::StatefulActionNode {
public:
  WaitForGreenLight(const std::string& n, const BT::NodeConfiguration& c) : BT::StatefulActionNode(n,c) {}
  static BT::PortsList providedPorts() {
    return { BT::InputPort<std::string>("light_color") };
  }
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;
};

class ProceedOnGreen : public BT::SyncActionNode {
public:
  ProceedOnGreen(const std::string& n, const BT::NodeConfiguration& c) : BT::SyncActionNode(n,c) {}
  static BT::PortsList providedPorts() {
    return { BT::InputPort<std::string>("light_color"), BT::InputPort<double>("max_react_sec") };
  }
  BT::NodeStatus tick() override;
};

class StopAtStopLine : public BT::SyncActionNode {
public:
  StopAtStopLine(const std::string& n, const BT::NodeConfiguration& c) : BT::SyncActionNode(n,c) {}
  static BT::PortsList providedPorts() {
    return { BT::InputPort<double>("tolerance") };
  }
  BT::NodeStatus tick() override;
};

class YieldAtRoundabout : public BT::SyncActionNode {
public:
  YieldAtRoundabout(const std::string& n, const BT::NodeConfiguration& c) : BT::SyncActionNode(n,c) {}
  static BT::PortsList providedPorts() { return {}; }
  BT::NodeStatus tick() override;
};

class ExecuteParking : public BT::StatefulActionNode {
public:
  ExecuteParking(const std::string& n, const BT::NodeConfiguration& c) : BT::StatefulActionNode(n,c) {}
  static BT::PortsList providedPorts() {
    return { BT::InputPort<std::string>("slot"), BT::InputPort<double>("time_limit_sec") };
  }
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;
private:
  std::chrono::steady_clock::time_point start_time_;
};

class BackUpAction : public BT::SyncActionNode {
public:
  BackUpAction(const std::string& n, const BT::NodeConfiguration& c) : BT::SyncActionNode(n,c) {}
  static BT::PortsList providedPorts() {
    return { BT::InputPort<double>("backup_dist"), BT::InputPort<double>("backup_speed"), BT::InputPort<double>("time_allowance") };
  }
  BT::NodeStatus tick() override;
};

class SpinAction : public BT::SyncActionNode {
public:
  SpinAction(const std::string& n, const BT::NodeConfiguration& c) : BT::SyncActionNode(n,c) {}
  static BT::PortsList providedPorts() {
    return { BT::InputPort<double>("spin_dist"), BT::InputPort<double>("time_allowance") };
  }
  BT::NodeStatus tick() override;
};

}  // namespace robotaksi_bt

#endif  // STUB_NODES_HPP
