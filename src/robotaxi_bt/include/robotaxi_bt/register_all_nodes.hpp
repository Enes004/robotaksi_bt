
// ============================================================================
// register_all_nodes.hpp — Tüm BT Node'larını Toplu Kayıt
//
// Kullanım:
//   BT::BehaviorTreeFactory factory;
//   robotaxi_bt::registerAllNodes(factory);
// ============================================================================
#ifndef REGISTER_ALL_NODES_HPP
#define REGISTER_ALL_NODES_HPP

#include "behaviortree_cpp_v3/bt_factory.h"

// Modüler node grupları
#include "robotaxi_bt/segment_loop_nodes.hpp"
#include "robotaxi_bt/vehicle_control_nodes.hpp"
#include "robotaxi_bt/traffic_logic_nodes.hpp"
#include "robotaxi_bt/mission_utility_nodes.hpp"
#include "robotaxi_bt/stub_nodes.hpp"

namespace robotaxi_bt {

inline void registerAllNodes(BT::BehaviorTreeFactory& factory)
{
  // ── 👁️ Perception: Sensör/Algılama Bağımlı ──
  factory.registerNodeType<EmergencyStopRequested>("EmergencyStopRequested");
  factory.registerNodeType<PedestrianAhead>("PedestrianAhead");
  factory.registerNodeType<DynamicObstacleAhead>("DynamicObstacleAhead");
  factory.registerNodeType<StaticObstacleInLane>("StaticObstacleInLane");
  factory.registerNodeType<AvoidanceSpaceAvailable>("AvoidanceSpaceAvailable");
  factory.registerNodeType<IsTwoWayRoad>("IsTwoWayRoad");
  factory.registerNodeType<TrafficLightAhead>("TrafficLightAhead");
  factory.registerNodeType<StopSignAhead>("StopSignAhead");
  factory.registerNodeType<GlobalRoadSignAhead>("GlobalRoadSignAhead");
  factory.registerNodeType<TurnConflictsWithSigns>("TurnConflictsWithSigns");
  factory.registerNodeType<IsLightRed>("IsLightRed");
  factory.registerNodeType<IsRoadSignType>("IsRoadSignType");
  factory.registerNodeType<LogUnknownSign>("LogUnknownSign");

  // ── ⚡ Nav2: Hareket/Navigasyon Bağımlı ──
  factory.registerNodeType<FollowLaneSegment>("FollowLaneSegment");
  factory.registerNodeType<WaitForClear>("WaitForClear");
  factory.registerNodeType<WaitForGreenLight>("WaitForGreenLight");
  factory.registerNodeType<WaitForGoSignal>("WaitForGoSignal");
  factory.registerNodeType<ProceedOnGreen>("ProceedOnGreen");
  factory.registerNodeType<StopAtStopLine>("StopAtStopLine");
  factory.registerNodeType<YieldAtRoundabout>("YieldAtRoundabout");
  factory.registerNodeType<ExecuteParking>("ExecuteParking");
  factory.registerNodeType<BackUpAction>("BackUp");
  factory.registerNodeType<SpinAction>("Spin");
  factory.registerNodeType<ReplanRoute>("ReplanRoute");
  factory.registerNodeType<CalculateLaneChange>("CalculateLaneChange");

  // ── 🚗 Control: Araç Kontrol/Aktüasyon ──
  factory.registerNodeType<SetMaxSpeed>("SetMaxSpeed");
  factory.registerNodeType<StopVehicle>("StopVehicle");
  factory.registerNodeType<TurnHeadlights>("TurnHeadlights");
  factory.registerNodeType<Dwell>("Dwell");
  factory.registerNodeType<StopAndProceed>("StopAndProceed");

  // ── 📍 Localization: Konum/Segment/Harita Takibi ──
  factory.registerNodeType<HasMoreSegments>("HasMoreSegments");
  factory.registerNodeType<IsSegmentType>("IsSegmentType");
  factory.registerNodeType<IsTrafficControlActive>("IsTrafficControlActive");
  factory.registerNodeType<AdvanceSegment>("AdvanceSegment");
  factory.registerNodeType<ClearHandledFlags>("ClearHandledFlags");
  factory.registerNodeType<LoadMission>("LoadMission");
  factory.registerNodeType<GetCurrentSegment>("GetCurrentSegment");
  factory.registerNodeType<FindParkingSlot>("FindParkingSlot");
  factory.registerNodeType<CheckStopAccuracy>("CheckStopAccuracy");
  factory.registerNodeType<IsStuck>("IsStuck");

  // ── 🎯 Görev Yardımcı (Perception/Nav2/Control/Localization dışı) ──
  factory.registerNodeType<RecordMissionPoint>("RecordMissionPoint");
  factory.registerNodeType<RecordParkEntryReached>("RecordParkEntryReached");
  factory.registerNodeType<SignalPassengerEvent>("SignalPassengerEvent");
}

}  // namespace robotaxi_bt

#endif  // REGISTER_ALL_NODES_HPP
