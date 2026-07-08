// ============================================================================
// Teknofest 2026 Robotaksi — Segment BT Custom Node Tanımları
//
// segment_bt.xml'deki TÜM custom Action ve Condition node'larının
// sınıf tanımları. Her node:
//   - Doğru portları tanımlar (XML TreeNodesModel ile birebir eşleşir)
//   - tick() override eder (implementasyon segment_bt_nodes.cpp'de)
//
// BT.CPP v3 (behaviortree_cpp_v3) uyumlu.
// ============================================================================
#ifndef SEGMENT_BT_NODES_HPP
#define SEGMENT_BT_NODES_HPP

#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/condition_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "robotaxi_bt/segment_graph.hpp"
#include "rclcpp/rclcpp.hpp"
#include <string>
#include <vector>
#include <chrono>
#include <cmath>

namespace robotaxi_bt {

// ═════════════════════════════════════════════════════════════
// ACTION NODE'LARI
// ═════════════════════════════════════════════════════════════

// ─── LoadMission: GeoJSON + graf → rota ───
class LoadMission : public BT::SyncActionNode {
public:
  LoadMission(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<std::string>("geojson_file", "GeoJSON dosya yolu"),
      BT::InputPort<int>("tour", "Tur numarasi 1/2/3"),
      BT::OutputPort<std::string>("route", "Segment dizisi (JSON)"),
      BT::OutputPort<int>("route_size", "Segment sayisi"),
      BT::OutputPort<int>("seg_index", "Baslangic indeksi (0)")
    };
  }

  BT::NodeStatus tick() override;

private:
  SegmentGraph graph_;
};

// ─── GetCurrentSegment: aktif segmenti çöz ───
class GetCurrentSegment : public BT::SyncActionNode {
public:
  GetCurrentSegment(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<std::string>("route"),
      BT::InputPort<int>("seg_index"),
      BT::OutputPort<std::string>("seg_type", "LANE_FOLLOW/INTERSECTION/..."),
      BT::OutputPort<std::string>("seg_goal", "Segment bitis hedefi"),
      BT::OutputPort<std::string>("seg_meta", "Tipe ozel veri")
    };
  }

  BT::NodeStatus tick() override;
};

// ─── AdvanceSegment: seg_index++ ───
class AdvanceSegment : public BT::SyncActionNode {
public:
  AdvanceSegment(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts() {
    return {
      BT::BidirectionalPort<int>("seg_index")
    };
  }

  BT::NodeStatus tick() override;
};

// ─── SetMaxSpeed: hız limiti ayarla ───
class SetMaxSpeed : public BT::SyncActionNode {
public:
  SetMaxSpeed(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<double>("speed", "m/s")
    };
  }

  BT::NodeStatus tick() override;
};

// ─── FollowLaneSegment: şeritte segment sonuna sür ───
class FollowLaneSegment : public BT::StatefulActionNode {
public:
  FollowLaneSegment(const std::string& name, const BT::NodeConfiguration& config)
    : BT::StatefulActionNode(name, config) {}

  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<std::string>("goal", "Segment sonu (Pose2D string)"),
      BT::InputPort<double>("max_speed", "0 = mevcut limiti kullan")
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  std::chrono::steady_clock::time_point start_time_;
};

// ─── StopVehicle: kontrollü tam dur ───
class StopVehicle : public BT::SyncActionNode {
public:
  StopVehicle(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override;
};

// ─── WaitForClear: tehlike geçene dek RUNNING ───
class WaitForClear : public BT::StatefulActionNode {
public:
  WaitForClear(const std::string& name, const BT::NodeConfiguration& config)
    : BT::StatefulActionNode(name, config) {}

  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<std::string>("hazard", "pedestrian/dynamic/static")
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  std::chrono::steady_clock::time_point start_time_;
};

// ─── StopAtStopLine: çizgide toleransta dur ───
class StopAtStopLine : public BT::SyncActionNode {
public:
  StopAtStopLine(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<double>("tolerance", "metre")
    };
  }

  BT::NodeStatus tick() override;
};

// ─── WaitForGreenLight: yeşil olana dek RUNNING ───
class WaitForGreenLight : public BT::StatefulActionNode {
public:
  WaitForGreenLight(const std::string& name, const BT::NodeConfiguration& config)
    : BT::StatefulActionNode(name, config) {}

  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<std::string>("light_color")
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;
};

// ─── ProceedOnGreen: yeşilde hızlı kalkış ───
class ProceedOnGreen : public BT::SyncActionNode {
public:
  ProceedOnGreen(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<std::string>("light_color"),
      BT::InputPort<double>("max_react_sec")
    };
  }

  BT::NodeStatus tick() override;
};

// ─── StopAndProceed: DUR tabelası → tam dur → devam ───
class StopAndProceed : public BT::SyncActionNode {
public:
  StopAndProceed(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override;
};

// ─── CalculateLaneChange: şerit değişim hedefi üret ───
class CalculateLaneChange : public BT::SyncActionNode {
public:
  CalculateLaneChange(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<std::string>("current_pose"),
      BT::InputPort<std::string>("target_lane"),
      BT::InputPort<std::string>("detected_signs", "Aktif tabelalar — yasak yön kontrolü"),
      BT::OutputPort<std::string>("seg_goal", "Guncellenen hedef")
    };
  }

  BT::NodeStatus tick() override;
};

// ─── ReplanRoute: rotayı yeniden çıkar ───
class ReplanRoute : public BT::SyncActionNode {
public:
  ReplanRoute(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<std::string>("reason"),
      BT::OutputPort<std::string>("route"),
      BT::OutputPort<int>("route_size"),
      BT::OutputPort<int>("seg_index")
    };
  }

  BT::NodeStatus tick() override;
};

// ─── TurnHeadlights: far aç/kapat ───
class TurnHeadlights : public BT::SyncActionNode {
public:
  TurnHeadlights(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<std::string>("state", "on/off")
    };
  }

  BT::NodeStatus tick() override;
};

// ─── YieldAtRoundabout: dönelde boşluk bekle ───
class YieldAtRoundabout : public BT::SyncActionNode {
public:
  YieldAtRoundabout(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override;
};

// ─── ExecuteRoundabout: doğru çıkıştan ayrıl ───
class ExecuteRoundabout : public BT::SyncActionNode {
public:
  ExecuteRoundabout(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<std::string>("exit_node")
    };
  }

  BT::NodeStatus tick() override;
};

// ─── Dwell: belirtilen süre bekle ───
class Dwell : public BT::StatefulActionNode {
public:
  Dwell(const std::string& name, const BT::NodeConfiguration& config)
    : BT::StatefulActionNode(name, config) {}

  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<double>("min_sec"),
      BT::InputPort<double>("max_sec")
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  std::chrono::steady_clock::time_point start_time_;
  double wait_duration_sec_ = 15.0;
};

// ─── SignalPassengerEvent: yolcu al/bırak işaretle ───
class SignalPassengerEvent : public BT::SyncActionNode {
public:
  SignalPassengerEvent(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<std::string>("event_type", "pickup/dropoff")
    };
  }

  BT::NodeStatus tick() override;
};

// ─── RecordMissionPoint: görev noktası geçiş kaydı ───
class RecordMissionPoint : public BT::SyncActionNode {
public:
  RecordMissionPoint(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<std::string>("point"),
      BT::InputPort<double>("tolerance")
    };
  }

  BT::NodeStatus tick() override;
};

// ─── RecordParkEntryReached: park girişi geçildi ───
class RecordParkEntryReached : public BT::SyncActionNode {
public:
  RecordParkEntryReached(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override;
};

// ─── FindParkingSlot: uygun slot bul ───
class FindParkingSlot : public BT::SyncActionNode {
public:
  FindParkingSlot(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts() {
    return {
      BT::OutputPort<std::string>("slot_pose")
    };
  }

  BT::NodeStatus tick() override;
};

// ─── ExecuteParking: dik park manevrası ───
class ExecuteParking : public BT::StatefulActionNode {
public:
  ExecuteParking(const std::string& name, const BT::NodeConfiguration& config)
    : BT::StatefulActionNode(name, config) {}

  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<std::string>("slot"),
      BT::InputPort<double>("time_limit_sec")
    };
  }

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  std::chrono::steady_clock::time_point start_time_;
};

// ─── BackUp: Nav2 kurtarma — geri git ───
class BackUpAction : public BT::SyncActionNode {
public:
  BackUpAction(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<double>("backup_dist"),
      BT::InputPort<double>("backup_speed"),
      BT::InputPort<double>("time_allowance")
    };
  }

  BT::NodeStatus tick() override;
};

// ─── Spin: Nav2 kurtarma — dön ───
class SpinAction : public BT::SyncActionNode {
public:
  SpinAction(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<double>("spin_dist"),
      BT::InputPort<double>("time_allowance")
    };
  }

  BT::NodeStatus tick() override;
};


// ═════════════════════════════════════════════════════════════
// CONDITION NODE'LARI
// ═════════════════════════════════════════════════════════════

// ─── HasMoreSegments ───
class HasMoreSegments : public BT::ConditionNode {
public:
  HasMoreSegments(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config) {}

  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<int>("seg_index"),
      BT::InputPort<int>("route_size")
    };
  }

  BT::NodeStatus tick() override;
};

// ─── IsSegmentType ───
class IsSegmentType : public BT::ConditionNode {
public:
  IsSegmentType(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config) {}

  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<std::string>("seg_type"),
      BT::InputPort<std::string>("expected")
    };
  }

  BT::NodeStatus tick() override;
};

// ─── IsTrafficControlActive ───
class IsTrafficControlActive : public BT::ConditionNode {
public:
  IsTrafficControlActive(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config) {}

  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<int>("tour")
    };
  }

  BT::NodeStatus tick() override;
};

// ─── EmergencyStopRequested ───
class EmergencyStopRequested : public BT::ConditionNode {
public:
  EmergencyStopRequested(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config) {}

  static BT::PortsList providedPorts() { return {}; }
  BT::NodeStatus tick() override;
};

// ─── PedestrianAhead ───
class PedestrianAhead : public BT::ConditionNode {
public:
  PedestrianAhead(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config) {}

  static BT::PortsList providedPorts() {
    return { BT::InputPort<bool>("detected") };
  }

  BT::NodeStatus tick() override;
};

// ─── DynamicObstacleAhead ───
class DynamicObstacleAhead : public BT::ConditionNode {
public:
  DynamicObstacleAhead(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config) {}

  static BT::PortsList providedPorts() {
    return { BT::InputPort<bool>("detected") };
  }

  BT::NodeStatus tick() override;
};

// ─── StaticObstacleInLane ───
class StaticObstacleInLane : public BT::ConditionNode {
public:
  StaticObstacleInLane(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config) {}

  static BT::PortsList providedPorts() {
    return { BT::InputPort<bool>("detected") };
  }

  BT::NodeStatus tick() override;
};

// ─── AvoidanceSpaceAvailable ───
class AvoidanceSpaceAvailable : public BT::ConditionNode {
public:
  AvoidanceSpaceAvailable(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config) {}

  static BT::PortsList providedPorts() {
    return { BT::InputPort<std::string>("free_lane") };
  }

  BT::NodeStatus tick() override;
};

// ─── TrafficLightAhead ───
class TrafficLightAhead : public BT::ConditionNode {
public:
  TrafficLightAhead(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config) {}

  static BT::PortsList providedPorts() {
    return { BT::OutputPort<std::string>("light_color") };
  }

  BT::NodeStatus tick() override;
};

// ─── IsLightRed ───
class IsLightRed : public BT::ConditionNode {
public:
  IsLightRed(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config) {}

  static BT::PortsList providedPorts() {
    return { BT::InputPort<std::string>("color") };
  }

  BT::NodeStatus tick() override;
};

// ─── StopSignAhead ───
class StopSignAhead : public BT::ConditionNode {
public:
  StopSignAhead(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config) {}

  static BT::PortsList providedPorts() {
    return { BT::InputPort<std::string>("detected") };
  }

  BT::NodeStatus tick() override;
};

// ─── TurnConflictsWithSigns ───
class TurnConflictsWithSigns : public BT::ConditionNode {
public:
  TurnConflictsWithSigns(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config) {}

  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<std::string>("planned_turn"),
      BT::InputPort<std::string>("detected_signs")
    };
  }

  BT::NodeStatus tick() override;
};

// ─── CheckStopAccuracy ───
class CheckStopAccuracy : public BT::ConditionNode {
public:
  CheckStopAccuracy(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config) {}

  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<std::string>("point"),
      BT::InputPort<double>("tolerance"),
      BT::InputPort<double>("heading_tolerance", "Derece cinsinden kafa açısı toleransı")
    };
  }

  BT::NodeStatus tick() override;
};

// ─── IsStuck ───
class IsStuck : public BT::ConditionNode {
public:
  IsStuck(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config) {}

  static BT::PortsList providedPorts() { return {}; }
  BT::NodeStatus tick() override;
};

// ─── GlobalRoadSignAhead: Her segmentte çalışır (LANE_FOLLOW dahil!) ───
// B-50h/i, B-14a, TT-36a/b, TT-4 (hız limiti kaldırıldı — şartnamede yok)
class GlobalRoadSignAhead : public BT::ConditionNode {
public:
  GlobalRoadSignAhead(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config) {}

  static BT::PortsList providedPorts() {
    return {
      BT::OutputPort<std::string>("sign_type",
        "LANE_MERGE / PEDESTRIAN_CROSS / PASS_DIRECTION / NO_ENTRY / NONE")
    };
  }

  BT::NodeStatus tick() override;
};

// ─── IsRoadSignType: global levha tipini karşılaştır ───
class IsRoadSignType : public BT::ConditionNode {
public:
  IsRoadSignType(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config) {}

  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<std::string>("sign_type"),
      BT::InputPort<std::string>("expected")
    };
  }

  BT::NodeStatus tick() override;
};

// ─── ApplySpeedLimit: hız limiti levhasını uygula ───
class ApplySpeedLimit : public BT::SyncActionNode {
public:
  ApplySpeedLimit(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<std::string>("sign_data", "Levha verisi (orn: SPEED_LIMIT_30)")
    };
  }

  BT::NodeStatus tick() override;
};

// ─── LogUnknownSign: bilinmeyen levhayı logla ───
class LogUnknownSign : public BT::SyncActionNode {
public:
  LogUnknownSign(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts() {
    return { BT::InputPort<std::string>("sign_type") };
  }

  BT::NodeStatus tick() override;
};

// ─── ClearHandledFlags: segment başında latch bayraklarını sıfırla ───
class ClearHandledFlags : public BT::SyncActionNode {
public:
  ClearHandledFlags(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config) {}
  static BT::PortsList providedPorts() { return {}; }
  BT::NodeStatus tick() override;
};

// ─── WaitForGoSignal: UMS-2 start sinyali bekle ───
class WaitForGoSignal : public BT::StatefulActionNode {
public:
  WaitForGoSignal(const std::string& name, const BT::NodeConfiguration& config)
    : BT::StatefulActionNode(name, config) {}
  static BT::PortsList providedPorts() { return {}; }
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;
};

// ─── IsTwoWayRoad: iki yönlü yol mu? ───
class IsTwoWayRoad : public BT::ConditionNode {
public:
  IsTwoWayRoad(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config) {}
  static BT::PortsList providedPorts() {
    return { BT::InputPort<bool>("is_two_way") };
  }
  BT::NodeStatus tick() override;
};


// ═════════════════════════════════════════════════════════════
// NODE KAYIT FONKSİYONU
// ═════════════════════════════════════════════════════════════

inline void registerSegmentBTNodes(BT::BehaviorTreeFactory& factory) {
  factory.registerNodeType<LoadMission>("LoadMission");
  factory.registerNodeType<GetCurrentSegment>("GetCurrentSegment");
  factory.registerNodeType<AdvanceSegment>("AdvanceSegment");
  factory.registerNodeType<ClearHandledFlags>("ClearHandledFlags");
  factory.registerNodeType<WaitForGoSignal>("WaitForGoSignal");
  factory.registerNodeType<SetMaxSpeed>("SetMaxSpeed");
  factory.registerNodeType<FollowLaneSegment>("FollowLaneSegment");
  factory.registerNodeType<StopVehicle>("StopVehicle");
  factory.registerNodeType<WaitForClear>("WaitForClear");
  factory.registerNodeType<StopAtStopLine>("StopAtStopLine");
  factory.registerNodeType<WaitForGreenLight>("WaitForGreenLight");
  factory.registerNodeType<ProceedOnGreen>("ProceedOnGreen");
  factory.registerNodeType<StopAndProceed>("StopAndProceed");
  factory.registerNodeType<CalculateLaneChange>("CalculateLaneChange");
  factory.registerNodeType<ReplanRoute>("ReplanRoute");
  factory.registerNodeType<TurnHeadlights>("TurnHeadlights");
  factory.registerNodeType<YieldAtRoundabout>("YieldAtRoundabout");
  factory.registerNodeType<ExecuteRoundabout>("ExecuteRoundabout");
  factory.registerNodeType<Dwell>("Dwell");
  factory.registerNodeType<SignalPassengerEvent>("SignalPassengerEvent");
  factory.registerNodeType<RecordMissionPoint>("RecordMissionPoint");
  factory.registerNodeType<RecordParkEntryReached>("RecordParkEntryReached");
  factory.registerNodeType<FindParkingSlot>("FindParkingSlot");
  factory.registerNodeType<ExecuteParking>("ExecuteParking");
  factory.registerNodeType<BackUpAction>("BackUp");
  factory.registerNodeType<SpinAction>("Spin");
  factory.registerNodeType<LogUnknownSign>("LogUnknownSign");

  factory.registerNodeType<HasMoreSegments>("HasMoreSegments");
  factory.registerNodeType<IsSegmentType>("IsSegmentType");
  factory.registerNodeType<IsTrafficControlActive>("IsTrafficControlActive");
  factory.registerNodeType<EmergencyStopRequested>("EmergencyStopRequested");
  factory.registerNodeType<PedestrianAhead>("PedestrianAhead");
  factory.registerNodeType<DynamicObstacleAhead>("DynamicObstacleAhead");
  factory.registerNodeType<StaticObstacleInLane>("StaticObstacleInLane");
  factory.registerNodeType<AvoidanceSpaceAvailable>("AvoidanceSpaceAvailable");
  factory.registerNodeType<IsTwoWayRoad>("IsTwoWayRoad");
  factory.registerNodeType<TrafficLightAhead>("TrafficLightAhead");
  factory.registerNodeType<IsLightRed>("IsLightRed");
  factory.registerNodeType<StopSignAhead>("StopSignAhead");
  factory.registerNodeType<TurnConflictsWithSigns>("TurnConflictsWithSigns");
  factory.registerNodeType<CheckStopAccuracy>("CheckStopAccuracy");
  factory.registerNodeType<GlobalRoadSignAhead>("GlobalRoadSignAhead");
  factory.registerNodeType<IsRoadSignType>("IsRoadSignType");
  factory.registerNodeType<IsStuck>("IsStuck");
}

}  // namespace robotaxi_bt

#endif  // SEGMENT_BT_NODES_HPP
