// ============================================================================
// stub_nodes.cpp — Stub Node İmplementasyonları
//
// Bu node'lar derlenir ve çalışır ama GERÇEK sensör/harita verisi KULLANMAZ.
// Her birinin TODO yorumu gerçek implementasyon için rehberdir.
// ============================================================================

#include "robotaksi_bt/stub_nodes.hpp"
#include <sstream>
#include <cmath>

namespace robotaksi_bt {

// ═══════════════ 🗺️ HARİTA BAĞIMLI STUB'LAR ═══════════════

BT::NodeStatus LoadMission::tick()
{
  if (!graph_loaded_) {
    const std::string lanelet_file = "config/lanelet_layer.geojson";
    const std::string linestring_file = "config/linestring_layer.geojson";

    if (!graph_.loadFromGeoJSON(lanelet_file, linestring_file)) {
      RCLCPP_ERROR(btLogger(), "LoadMission: GeoJSON harita yüklenemedi!");
      return BT::NodeStatus::FAILURE;
    }

    graph_loaded_ = true;
  }

  // ── GeoJSON'dan waypoint okuma ──────────────────────────────────────────
  // geojson_file portu: mission JSON dosya yolu (opsiyonel).
  // Boşsa veya dosya açılamazsa eski sabit test rotasına fallback yapılır.
  std::vector<std::string> waypoint_ids;
  std::string mission_json_path;
  getInput("geojson_file", mission_json_path);

  if (mission_json_path.empty()) {
    // Fallback: port verilmemiş
    RCLCPP_WARN(btLogger(),
      "LoadMission: 'geojson_file' portu boş — mission_json verilmedi, test rotası kullanılıyor");
  } else {
    bool parse_ok = false;
    std::ifstream mf(mission_json_path);
    if (!mf.is_open()) {
      RCLCPP_WARN(btLogger(),
        "LoadMission: Mission JSON açılamadı: %s — test rotası kullanılıyor",
        mission_json_path.c_str());
    } else {
      try {
        nlohmann::json mj;
        mf >> mj;
        if (mj.contains("features") && mj["features"].is_array() &&
            !mj["features"].empty())
        {
          for (const auto& feat : mj["features"]) {
            // geometry.coordinates: [lon, lat]
            if (!feat.contains("geometry")) continue;
            const auto& geom = feat["geometry"];
            if (!geom.contains("coordinates") ||
                !geom["coordinates"].is_array() ||
                geom["coordinates"].size() < 2) continue;

            double lon = geom["coordinates"][0].get<double>();
            double lat = geom["coordinates"][1].get<double>();

            // Equirectangular projeksiyon (namespace-level yardımcılar)
            double x = lonToMeters(lon);
            double y = latToMeters(lat);

            // En yakın graf düğümünü bul
            std::string nid = graph_.findNearestNode(x, y);
            if (!nid.empty()) {
              waypoint_ids.push_back(nid);
            }
          }
          if (!waypoint_ids.empty()) {
            parse_ok = true;
            RCLCPP_INFO(btLogger(),
              "LoadMission: Mission JSON'dan %zu waypoint yüklendi (%s)",
              waypoint_ids.size(), mission_json_path.c_str());
          } else {
            RCLCPP_WARN(btLogger(),
              "LoadMission: Mission JSON features boş veya eşleşme yok — test rotası kullanılıyor");
          }
        } else {
          RCLCPP_WARN(btLogger(),
            "LoadMission: Mission JSON 'features' listesi boş/eksik — test rotası kullanılıyor");
        }
      } catch (const nlohmann::json::exception& e) {
        RCLCPP_WARN(btLogger(),
          "LoadMission: Mission JSON parse hatası: %s — test rotası kullanılıyor", e.what());
      }
    }

    if (!parse_ok) {
      waypoint_ids.clear();  // fallback'e düşülecek
    }
  }

  // Fallback: geojson yoksa ya da parse başarısızsa sabit test rotası
  if (waypoint_ids.empty()) {
    waypoint_ids = {"4", "10", "16"};
  }

  // GÖREV 1 — Teşhis: bulunan waypoint id'lerini logla
  {
    std::string ids_str;
    for (auto& id : waypoint_ids) ids_str += id + " ";
    RCLCPP_INFO(btLogger(), "LoadMission: bulunan waypoint id'leri: %s", ids_str.c_str());
  }

  // GÖREV 2 — İki kademeli rota denemesi
  auto route = graph_.planRoute(waypoint_ids);
  if (route.empty()) {
    RCLCPP_WARN(btLogger(), "LoadMission: mission_json rotası boş, sabit test rotasına düşülüyor");
    route = graph_.planRoute({"4", "10", "16"});
  }
  if (route.empty()) {
    RCLCPP_ERROR(btLogger(), "LoadMission: Rota planlama başarısız veya rota boş!");
    return BT::NodeStatus::FAILURE;
  }

  auto route_ptr = std::make_shared<Route>(route);
  // Kök blackboard'a doğrudan yaz — SubTree autoremap sınırını aşar
  globalRootBlackboard()->set("route_obj", route_ptr);
  globalRootBlackboard()->set("route_size", static_cast<int>(route.size()));
  globalRootBlackboard()->set("seg_index", 0);

  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus GetCurrentSegment::tick()
{
  // Kök blackboard'dan oku — SubTree autoremap sınırını aşar
  std::shared_ptr<Route> route_ptr;
  if (!globalRootBlackboard()->get("route_obj", route_ptr) || !route_ptr) {
    RCLCPP_ERROR(btLogger(), "GetCurrentSegment: Blackboard'da 'route_obj' bulunamadı veya null!");
    return BT::NodeStatus::FAILURE;
  }

  int seg_index = 0;
  if (!globalRootBlackboard()->get("seg_index", seg_index)) {
    RCLCPP_ERROR(btLogger(), "GetCurrentSegment: 'seg_index' okunamadı!");
    return BT::NodeStatus::FAILURE;
  }

  if (seg_index < 0 || static_cast<size_t>(seg_index) >= route_ptr->size()) {
    RCLCPP_ERROR(btLogger(), "GetCurrentSegment: seg_index (%d) sınır dışında! (route_size: %zu)",
                 seg_index, route_ptr->size());
    return BT::NodeStatus::FAILURE;
  }

  const Segment& segment = route_ptr->at(static_cast<size_t>(seg_index));

  // Kök blackboard'a yaz — SubTree autoremap sınırını aşar
  globalRootBlackboard()->set("seg_type", segment.type);

  std::ostringstream oss;
  oss << segment.goal_x << ";" << segment.goal_y << ";" << segment.goal_yaw;
  globalRootBlackboard()->set("seg_goal", oss.str());

  globalRootBlackboard()->set("seg_meta", segment.meta);

  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus ReplanRoute::tick()
{
  std::string reason; getInput("reason", reason);
  RCLCPP_WARN(btLogger(), "ReplanRoute: STUB sebep=%s", reason.c_str());
  // TODO: Mevcut pozisyondan yeni Dijkstra rotası hesapla
  // Kök blackboard'a yaz — SubTree autoremap sınırını aşar
  globalRootBlackboard()->set("route_size", 0);
  globalRootBlackboard()->set("seg_index", 0);
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus CalculateLaneChange::tick()
{
  std::string target_lane; getInput("target_lane", target_lane);
  RCLCPP_INFO(btLogger(), "CalculateLaneChange: STUB hedef=%s", target_lane.c_str());
  // TODO: Yanal offset hesapla (±3.5m şerit genişliği)
  globalRootBlackboard()->set("seg_goal", std::string("0.0;0.0;0.0"));  // kök BB
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
  auto& odom = OdometryProvider::instance();

  // Veri yoksa veya bayatsa → FAILURE (güvenli taraf: "doğru durdum" diyemeyiz)
  if (!odom.isFresh()) {
    RCLCPP_WARN_THROTTLE(btLogger(), *getRosNode(config()), 2000,
      "CheckStopAccuracy: odometry verisi yok/bayat — FAILURE");
    return BT::NodeStatus::FAILURE;
  }

  // Mevcut pozisyonu al
  double cur_x, cur_y, cur_yaw;
  if (!odom.getPose(cur_x, cur_y, cur_yaw)) {
    return BT::NodeStatus::FAILURE;
  }

  // Hedef noktayı "x;y;yaw" formatından parse et
  std::string point_str;
  if (!globalRootBlackboard()->get("seg_goal", point_str) || point_str.empty()) {
    RCLCPP_ERROR(btLogger(), "CheckStopAccuracy: 'point' portu boş!");
    return BT::NodeStatus::FAILURE;
  }

  double tgt_x = 0.0, tgt_y = 0.0, tgt_yaw = 0.0;
  {
    std::istringstream iss(point_str);
    char delim;
    if (!(iss >> tgt_x >> delim >> tgt_y >> delim >> tgt_yaw)) {
      RCLCPP_ERROR(btLogger(), "CheckStopAccuracy: 'point' parse hatası: '%s'", point_str.c_str());
      return BT::NodeStatus::FAILURE;
    }
  }

  // Toleransları oku
  double tolerance = 1.0;
  double heading_tolerance_deg = 15.0;
  getInput("tolerance", tolerance);
  getInput("heading_tolerance", heading_tolerance_deg);
  double heading_tolerance_rad = heading_tolerance_deg * M_PI / 180.0;

  // Öklid mesafesi
  double dx = cur_x - tgt_x;
  double dy = cur_y - tgt_y;
  double dist = std::sqrt(dx * dx + dy * dy);

  // Yaw farkı ([-π, π] aralığına normalize et)
  double tgt_yaw_rad = tgt_yaw * M_PI / 180.0;  // XML'de derece olarak gelir
  double yaw_err = cur_yaw - tgt_yaw_rad;
  // Normalize to [-π, π]
  while (yaw_err > M_PI)  yaw_err -= 2.0 * M_PI;
  while (yaw_err < -M_PI) yaw_err += 2.0 * M_PI;

  bool pos_ok = dist <= tolerance;
  bool yaw_ok = std::fabs(yaw_err) <= heading_tolerance_rad;

  RCLCPP_INFO_THROTTLE(btLogger(), *getRosNode(config()), 1000,
    "CheckStopAccuracy: dist=%.3f (tol=%.2f) yaw_err=%.1f° (tol=%.1f°) → %s",
    dist, tolerance,
    yaw_err * 180.0 / M_PI, heading_tolerance_deg,
    (pos_ok && yaw_ok) ? "SUCCESS" : "FAILURE");

  return (pos_ok && yaw_ok) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::NodeStatus IsStuck::tick()
{
  auto& odom = OdometryProvider::instance();

  // Veri yoksa veya bayatsa → FAILURE (güvenli taraf: GPS kesildiyse
  // yanlışlıkla recovery moduna girmemeli)
  if (!odom.isFresh()) {
    stuck_active_ = false;  // timer'ı sıfırla
    RCLCPP_WARN_THROTTLE(btLogger(), *getRosNode(config()), 2000,
      "IsStuck: odometry verisi yok/bayat — FAILURE");
    return BT::NodeStatus::FAILURE;
  }

  double speed = 0.0;
  if (!odom.getLinearSpeed(speed)) {
    stuck_active_ = false;
    return BT::NodeStatus::FAILURE;
  }

  constexpr double kSpeedThreshold = 0.05;  // m/s
  constexpr double kStuckDurationSec = 5.0; // saniye

  auto now = std::chrono::steady_clock::now();

  if (speed < kSpeedThreshold) {
    if (!stuck_active_) {
      // Düşük hız başladı, zamanlayıcıyı başlat
      stuck_active_ = true;
      stuck_since_ = now;
    }
    double elapsed = std::chrono::duration<double>(now - stuck_since_).count();
    if (elapsed >= kStuckDurationSec) {
      RCLCPP_WARN(btLogger(),
        "IsStuck: hız=%.3f m/s, %.1f sn boyunca düşük → STUCK (SUCCESS)",
        speed, elapsed);
      return BT::NodeStatus::SUCCESS;  // takılmış!
    }
  } else {
    // Hız eşiğin üstünde, timer'ı sıfırla
    stuck_active_ = false;
  }

  return BT::NodeStatus::FAILURE;
}

// ═══════════════ ⚡ NAV2/HAREKET BAĞIMLI STUB'LAR ═══════════════

BT::NodeStatus FollowLaneSegment::onStart()
{
  std::string goal; globalRootBlackboard()->get("seg_goal", goal);
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
