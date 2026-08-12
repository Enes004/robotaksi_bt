// ============================================================================
// stub_nodes.cpp — Stub Node İmplementasyonları
//
// Bu node'lar derlenir ve çalışır ama GERÇEK sensör/harita verisi KULLANMAZ.
// Her birinin TODO yorumu gerçek implementasyon için rehberdir.
// ============================================================================

#include "robotaksi_bt/stub_nodes.hpp"

namespace robotaksi_bt {

// ═══════════════ 🗺️ HARİTA BAĞIMLI STUB'LAR ═══════════════

BT::NodeStatus LoadMission::tick()
{
  std::string geojson_file; int tour = 1;
  getInput("geojson_file", geojson_file);
  getInput("tour", tour);
  RCLCPP_WARN(btLogger(), "LoadMission: STUB — geojson=%s, tur=%d", geojson_file.c_str(), tour);

  // Segment graf'ı yalnızca ilk tick'te yükle — her tick'te dosyaları
  // yeniden parse etmek gereksiz I/O'ya yol açar.
  if (!graph_loaded_) {
    // TODO: gerçek dosya yollarını gir (harita ekibinden gelen dosyalar)
    const std::string routing_graph_yaml = "TODO: routing_graph.yaml yolu";
    const std::string lanelet_layer_geojson = "TODO: lanelet_layer.geojson yolu";

    if (!graph_.loadFromYAML(routing_graph_yaml)) {
      RCLCPP_ERROR(btLogger(), "LoadMission: routing_graph.yaml yüklenemedi: %s",
                   routing_graph_yaml.c_str());
    } else {
      // loadFromYAML'dan SONRA ayrıca çağrılmalı — iki dosya harita
      // ekibinden ayrı zamanlarda gelir (bkz. segment_graph.hpp).
      if (!graph_.loadTypesFromGeoJSON(lanelet_layer_geojson)) {
        RCLCPP_WARN(btLogger(), "LoadMission: lanelet_layer.geojson yüklenemedi: %s",
                    lanelet_layer_geojson.c_str());
      }
      graph_loaded_ = true;
    }
  }

  // TODO: 1) GeoJSON oku  2) segment_map.yaml yükle  3) Dijkstra rota planla
  setOutput("route", std::string("[]"));
  setOutput("route_size", 0);
  setOutput("seg_index", 0);
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus GetCurrentSegment::tick()
{
  int seg_index = 0; getInput("seg_index", seg_index);
  RCLCPP_INFO(btLogger(), "GetCurrentSegment: STUB index=%d", seg_index);
  // TODO: route JSON parse et, seg_index'teki segmenti çöz
  setOutput("seg_type", std::string("LANE_FOLLOW"));
  setOutput("seg_goal", std::string("0.0;0.0;0.0"));
  setOutput("seg_meta", std::string(""));
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus ReplanRoute::tick()
{
  std::string reason; getInput("reason", reason);
  RCLCPP_WARN(btLogger(), "ReplanRoute: STUB sebep=%s", reason.c_str());
  // TODO: Mevcut pozisyondan yeni Dijkstra rotası hesapla
  setOutput("route", std::string("[]"));
  setOutput("route_size", 0);
  setOutput("seg_index", 0);
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus CalculateLaneChange::tick()
{
  std::string target_lane; getInput("target_lane", target_lane);
  RCLCPP_INFO(btLogger(), "CalculateLaneChange: STUB hedef=%s", target_lane.c_str());
  // TODO: Yanal offset hesapla (±3.5m şerit genişliği)
  setOutput("seg_goal", std::string("0.0;0.0;0.0"));
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus FindParkingSlot::tick()
{
  RCLCPP_INFO(btLogger(), "FindParkingSlot: STUB");
  // TODO: Kamera/Lidar ile P-3a tabelası olan boş slot bul
  setOutput("slot_pose", std::string("0.0;0.0;0.0"));
  return BT::NodeStatus::SUCCESS;
}

// ═══════════════ 🔧 SENSÖR BAĞIMLI STUB'LAR ═══════════════
// Condition stub'ları FAILURE döner = "tehlike yok"
// SafetyReflexes'te Inverter ile SUCCESS'a çevrilir → devam et

BT::NodeStatus EmergencyStopRequested::tick()
{
  // TODO: /emergency_stop (Bool) topic'ini subscribe et
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus PedestrianAhead::tick()
{
  // TODO: /perception/pedestrian (Bool) topic'ini subscribe et
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus DynamicObstacleAhead::tick()
{
  // TODO: /perception/dynamic_obstacle (Bool) topic'ini subscribe et
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus StaticObstacleInLane::tick()
{
  // TODO: /perception/static_obstacle (Bool) topic'ini subscribe et
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus AvoidanceSpaceAvailable::tick()
{
  // TODO: Costmap'ten yan şerit boşluk kontrolü
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus IsTwoWayRoad::tick()
{
  // TODO: segment_map.yaml'dan veya B-52a tabelasından kontrol
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus TrafficLightAhead::tick()
{
  // TODO: /perception/traffic_light (String) subscribe et
  setOutput("light_color", std::string("NONE"));
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus StopSignAhead::tick()
{
  // TODO: /perception/stop_sign (Bool) subscribe et
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus GlobalRoadSignAhead::tick()
{
  // TODO: /perception/road_sign (String) subscribe et
  setOutput("sign_type", std::string("NONE"));
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus TurnConflictsWithSigns::tick()
{
  std::string planned_turn, detected_signs;
  getInput("planned_turn", planned_turn);
  getInput("detected_signs", detected_signs);
  // TODO: Planlanan dönüş ile tabelalar çelişiyor mu kontrol et
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus CheckStopAccuracy::tick()
{
  // TODO: /amcl_pose veya /odom'dan mevcut pozisyonu al, point ile karşılaştır
  return BT::NodeStatus::SUCCESS;  // Stub: her zaman doğru
}

BT::NodeStatus IsStuck::tick()
{
  // TODO: /odom'dan son N saniyedeki mesafeyi hesapla, eşik altında mı?
  return BT::NodeStatus::FAILURE;
}

// ═══════════════ ⚡ NAV2/HAREKET BAĞIMLI STUB'LAR ═══════════════

BT::NodeStatus FollowLaneSegment::onStart()
{
  std::string goal; getInput("goal", goal);
  RCLCPP_INFO(btLogger(), "FollowLaneSegment: STUB START goal=%s", goal.c_str());
  // TODO: Nav2 NavigateToPose action client başlat
  start_time_ = std::chrono::steady_clock::now();
  return BT::NodeStatus::RUNNING;
}
BT::NodeStatus FollowLaneSegment::onRunning()
{
  auto elapsed = std::chrono::steady_clock::now() - start_time_;
  if (elapsed > std::chrono::seconds(2)) {
    RCLCPP_INFO(btLogger(), "FollowLaneSegment: STUB tamamlandı");
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::RUNNING;
}
void FollowLaneSegment::onHalted()
{
  RCLCPP_WARN(btLogger(), "FollowLaneSegment: HALTED");
}

BT::NodeStatus WaitForClear::onStart()
{
  std::string hazard; getInput("hazard", hazard);
  RCLCPP_INFO(btLogger(), "WaitForClear: STUB %s bekleniyor", hazard.c_str());
  start_time_ = std::chrono::steady_clock::now();
  return BT::NodeStatus::RUNNING;
}
BT::NodeStatus WaitForClear::onRunning()
{
  if (std::chrono::steady_clock::now() - start_time_ > std::chrono::seconds(3)) {
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::RUNNING;
}
void WaitForClear::onHalted() { RCLCPP_WARN(btLogger(), "WaitForClear: HALTED"); }

BT::NodeStatus WaitForGreenLight::onStart()
{
  RCLCPP_INFO(btLogger(), "WaitForGreenLight: STUB bekleniyor");
  return BT::NodeStatus::RUNNING;
}
BT::NodeStatus WaitForGreenLight::onRunning()
{
  // TODO: /perception/traffic_light topic'inden renk oku
  return BT::NodeStatus::SUCCESS;  // Stub: hemen yeşil
}
void WaitForGreenLight::onHalted() { RCLCPP_WARN(btLogger(), "WaitForGreenLight: HALTED"); }

BT::NodeStatus WaitForGoSignal::onStart()
{
  RCLCPP_INFO(btLogger(), "WaitForGoSignal: STUB UMS-2 bekleniyor");
  return BT::NodeStatus::RUNNING;
}
BT::NodeStatus WaitForGoSignal::onRunning()
{
  // TODO: /mission/go_signal (Bool) subscribe et
  return BT::NodeStatus::SUCCESS;  // Stub: hemen başlat
}
void WaitForGoSignal::onHalted() { RCLCPP_WARN(btLogger(), "WaitForGoSignal: HALTED"); }

BT::NodeStatus ProceedOnGreen::tick()
{
  RCLCPP_INFO(btLogger(), "ProceedOnGreen: STUB");
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus StopAtStopLine::tick()
{
  RCLCPP_INFO(btLogger(), "StopAtStopLine: STUB");
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus YieldAtRoundabout::tick()
{
  RCLCPP_INFO(btLogger(), "YieldAtRoundabout: STUB");
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus ExecuteParking::onStart()
{
  RCLCPP_INFO(btLogger(), "ExecuteParking: STUB START");
  start_time_ = std::chrono::steady_clock::now();
  return BT::NodeStatus::RUNNING;
}
BT::NodeStatus ExecuteParking::onRunning()
{
  double limit = 180.0; getInput("time_limit_sec", limit);
  auto sec = std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time_).count();
  if (sec >= limit) return BT::NodeStatus::FAILURE;
  if (sec >= 5.0) return BT::NodeStatus::SUCCESS;
  return BT::NodeStatus::RUNNING;
}
void ExecuteParking::onHalted() { RCLCPP_WARN(btLogger(), "ExecuteParking: HALTED"); }

BT::NodeStatus BackUpAction::tick()
{
  double dist=0.3, speed=0.05; getInput("backup_dist", dist); getInput("backup_speed", speed);
  RCLCPP_INFO(btLogger(), "BackUp: STUB dist=%.2f speed=%.2f", dist, speed);
  // TODO: /cmd_vel'e negatif linear.x publish
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus SpinAction::tick()
{
  double dist=1.57; getInput("spin_dist", dist);
  RCLCPP_INFO(btLogger(), "Spin: STUB dist=%.2f rad", dist);
  // TODO: /cmd_vel'e angular.z publish
  return BT::NodeStatus::SUCCESS;
}

}  // namespace robotaksi_bt
