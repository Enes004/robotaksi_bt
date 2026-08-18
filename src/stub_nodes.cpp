// ============================================================================
// stub_nodes.cpp — Stub Node İmplementasyonları
//
// Bu node'lar derlenir ve çalışır ama GERÇEK sensör/harita verisi KULLANMAZ.
// Her birinin TODO yorumu gerçek implementasyon için rehberdir.
// ============================================================================

#include "robotaksi_bt/stub_nodes.hpp"
#include <cmath>
#include <sstream>

namespace robotaksi_bt {

// ═══════════════ 🗺️ HARİTA BAĞIMLI STUB'LAR ═══════════════

BT::NodeStatus LoadMission::tick() {
  if (!graph_loaded_) {
    const std::string lanelet_file = "config/lanelet_layer.geojson";
    const std::string linestring_file = "config/linestring_layer.geojson";

    if (!globalSegmentGraph().loadFromGeoJSON(lanelet_file, linestring_file)) {
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
    // BİLİNÇLİ TEST MODU: geojson_file parametresi verilmemiş.
    // Bu durum normal geliştirme akışında beklenen bir seçimdir;
    // using_fallback_route flag'i burada set edilmez (ayrı izleme gereksiz).
    RCLCPP_WARN(btLogger(),
                "LoadMission: 'geojson_file' portu boş — mission_json "
                "parametresi verilmedi,"
                " sabit test rotası kullanılacak (bilinçli test modu)");
  } else {
    bool parse_ok = false;
    std::ifstream mf(mission_json_path);
    if (!mf.is_open()) {
      RCLCPP_WARN(
          btLogger(),
          "LoadMission: Mission JSON açılamadı: %s — test rotası kullanılıyor",
          mission_json_path.c_str());
    } else {
      try {
        nlohmann::json mj;
        mf >> mj;
        if (mj.contains("features") && mj["features"].is_array() &&
            !mj["features"].empty()) {
          for (const auto &feat : mj["features"]) {
            // geometry.coordinates: [lon, lat]
            if (!feat.contains("geometry"))
              continue;
            const auto &geom = feat["geometry"];
            if (!geom.contains("coordinates") ||
                !geom["coordinates"].is_array() ||
                geom["coordinates"].size() < 2)
              continue;

            double lon = geom["coordinates"][0].get<double>();
            double lat = geom["coordinates"][1].get<double>();

            // Equirectangular projeksiyon (namespace-level yardımcılar)
            double x = lonToMeters(lon);
            double y = latToMeters(lat);

            // En yakın graf düğümünü bul
            std::string nid = globalSegmentGraph().findNearestNode(x, y);
            if (!nid.empty()) {
              waypoint_ids.push_back(nid);
            }
          }
          if (!waypoint_ids.empty()) {
            parse_ok = true;
            // Gerçek rota başarıyla yüklendi — fallback flag'ini temizle
            globalRootBlackboard()->set("using_fallback_route", false);
            RCLCPP_INFO(
                btLogger(),
                "LoadMission: Mission JSON'dan %zu waypoint yüklendi (%s)",
                waypoint_ids.size(), mission_json_path.c_str());
          } else {
            RCLCPP_ERROR(btLogger(), "LoadMission: Mission JSON features boş "
                                     "veya haritayla eşleşme yok!"
                                     " Koordinat/harita uyuşmazlığı olabilir. "
                                     "Sabit test rotasına düşülüyor.");
          }
        } else {
          RCLCPP_WARN(btLogger(),
                      "LoadMission: Mission JSON 'features' listesi boş/eksik "
                      "— test rotası kullanılıyor");
        }
      } catch (const nlohmann::json::exception &e) {
        RCLCPP_WARN(btLogger(),
                    "LoadMission: Mission JSON parse hatası: %s — test rotası "
                    "kullanılıyor",
                    e.what());
      }
    }

    if (!parse_ok) {
      waypoint_ids.clear(); // fallback'e düşülecek
    }
  }

  // Fallback: geojson yoksa ya da parse başarısızsa sabit test rotası
  if (waypoint_ids.empty()) {
    waypoint_ids = {"1","3","5","7","77","11","148","53","128","68","120","56","150","52","102","44","100","45"};
  }

  // GÖREV 1 — Teşhis: bulunan waypoint id'lerini logla
  {
    std::string ids_str;
    for (auto &id : waypoint_ids)
      ids_str += id + " ";
    RCLCPP_INFO(btLogger(), "LoadMission: bulunan waypoint id'leri: %s",
                ids_str.c_str());
  }

  // GÖREV 2 — İki kademeli rota denemesi
  auto route = globalSegmentGraph().planRoute(waypoint_ids);
  if (route.empty()) {
    // ── FALLBACK ETKİNLEŞTİ ─────────────────────────────────────────────────
    // planRoute() boş döndü: harita bağlantı sorunu veya yanlış koordinat
    // eşleşmesi olabilir. Fallback mekanizması korunuyor (BT çökmemeli)
    // ama SESSIZ GEÇMEMELI — her ikisi de uyarılmalı.
    RCLCPP_ERROR(btLogger(),
      "LoadMission: mission_json rotası BOŞ döndü! (harita bağlantı sorunu veya"
      " koordinat eşleşme hatası). SABİT TEST ROTASINA"
      " (1-3-5-7-77-11-148-53-128-68-120-56-150-52-102-44-100-45)"
      " düşülüyor — lütfen harita ve mission_json koordinatlarını kontrol edin!");
    globalRootBlackboard()->set("using_fallback_route", true);
    route = globalSegmentGraph().planRoute(
      {"1","3","5","7","77","11","148","53","128","68","120","56","150","52","102","44","100","45"});
    // ────────────────────────────────────────────────────────────────────────
  }
  if (route.empty()) {
    RCLCPP_ERROR(btLogger(),
                 "LoadMission: Rota planlama başarısız veya rota boş!");
    return BT::NodeStatus::FAILURE;
  }

  auto route_ptr = std::make_shared<Route>(route);
  // Kök blackboard'a doğrudan yaz — SubTree autoremap sınırını aşar
  globalRootBlackboard()->set("route_obj", route_ptr);
  globalRootBlackboard()->set("route_size", static_cast<int>(route.size()));
  globalRootBlackboard()->set("seg_index", 0);

  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus GetCurrentSegment::tick() {
  // Kök blackboard'dan oku — SubTree autoremap sınırını aşar
  std::shared_ptr<Route> route_ptr;
  if (!globalRootBlackboard()->get("route_obj", route_ptr) || !route_ptr) {
    RCLCPP_ERROR(
        btLogger(),
        "GetCurrentSegment: Blackboard'da 'route_obj' bulunamadı veya null!");
    return BT::NodeStatus::FAILURE;
  }

  int seg_index = 0;
  if (!globalRootBlackboard()->get("seg_index", seg_index)) {
    RCLCPP_ERROR(btLogger(), "GetCurrentSegment: 'seg_index' okunamadı!");
    return BT::NodeStatus::FAILURE;
  }

  if (seg_index < 0 || static_cast<size_t>(seg_index) >= route_ptr->size()) {
    RCLCPP_ERROR(
        btLogger(),
        "GetCurrentSegment: seg_index (%d) sınır dışında! (route_size: %zu)",
        seg_index, route_ptr->size());
    return BT::NodeStatus::FAILURE;
  }

  const Segment &segment = route_ptr->at(static_cast<size_t>(seg_index));

  // Kök blackboard'a yaz — SubTree autoremap sınırını aşar
  globalRootBlackboard()->set("seg_type", segment.type);

  std::ostringstream oss;
  oss << segment.goal_x << ";" << segment.goal_y << ";" << segment.goal_yaw;
  globalRootBlackboard()->set("seg_goal", oss.str());

  globalRootBlackboard()->set("seg_meta", segment.meta);

  // ── Planlanan Dönüş Hesabı ─────────────────────────────────────────────
  // path_xy'den giriş ve çıkış yönleri hesaplanır:
  //   giriş açısı : path[0]  → path[1]   arası atan2
  //   çıkış açısı : path[-2] → path[-1]  arası atan2
  //   fark (-180/+180 normalize): |fark| < 25° → STRAIGHT
  //                               fark > 0     → LEFT
  //                               fark < 0     → RIGHT
  // path_xy < 2 nokta ise STRAIGHT varsayılanı kullanılır.
  {
    std::string turn_str = "STRAIGHT"; // varsayılan

    const auto &path = segment.path_xy;
    if (path.size() >= 2) {
      // Giriş açısı: ilk iki nokta
      double entry_angle = std::atan2(path[1].second - path[0].second,
                                      path[1].first - path[0].first);

      // Çıkış açısı: son iki nokta
      size_t n = path.size();
      double exit_angle = std::atan2(path[n - 1].second - path[n - 2].second,
                                     path[n - 1].first - path[n - 2].first);

      // Fark (radyan → derece, -180/+180 aralığına normalize)
      double diff_deg = (exit_angle - entry_angle) * 180.0 / M_PI;
      while (diff_deg > 180.0)
        diff_deg -= 360.0;
      while (diff_deg < -180.0)
        diff_deg += 360.0;

      if (std::fabs(diff_deg) < 25.0) {
        turn_str = "STRAIGHT";
      } else if (diff_deg > 0.0) {
        turn_str = "LEFT";
      } else {
        turn_str = "RIGHT";
      }
    }

    globalRootBlackboard()->set("planned_turn", turn_str);
    RCLCPP_DEBUG(btLogger(),
                 "GetCurrentSegment: seg_index=%d → planned_turn=%s (path_xy "
                 "nokta sayısı: %zu)",
                 seg_index, turn_str.c_str(), segment.path_xy.size());
  }
  // ──────────────────────────────────────────────────────────────────────────

  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus ReplanRoute::tick() {
  std::string reason;
  getInput("reason", reason);

  // ── Adım 1: Mevcut konumu belirle ─────────────────────────────────────
  // Önce OdometryProvider'dan gerçek konum dene,
  // başarısız olursa blackboard'daki seg_goal'dan parse et.
  double cur_x = 0.0, cur_y = 0.0, cur_yaw = 0.0;
  bool pose_ok = OdometryProvider::instance().getPose(cur_x, cur_y, cur_yaw);

  if (!pose_ok) {
    // Odometry yok — son bilinen segment hedefini kullan
    std::string seg_goal_str;
    if (globalRootBlackboard()->get("seg_goal", seg_goal_str) &&
        !seg_goal_str.empty()) {
      std::istringstream iss(seg_goal_str);
      char delim;
      if (iss >> cur_x >> delim >> cur_y >> delim >> cur_yaw) {
        pose_ok = true;
        RCLCPP_WARN(btLogger(),
                    "ReplanRoute: Odometry yok, seg_goal konumu kullanılıyor "
                    "(%.2f, %.2f)",
                    cur_x, cur_y);
      }
    }
  }

  if (!pose_ok) {
    RCLCPP_ERROR(btLogger(), "ReplanRoute: Başlangıç konumu belirlenemedi "
                             "(odometry yok, seg_goal boş)!");
    return BT::NodeStatus::FAILURE;
  }

  // ── Adım 2: En yakın graph düğümünü bul ──────────────────────────────
  std::string start_id = globalSegmentGraph().findNearestNode(cur_x, cur_y);
  if (start_id.empty()) {
    RCLCPP_ERROR(btLogger(),
                 "ReplanRoute: findNearestNode başarısız (%.2f, %.2f) — harita "
                 "yüklendi mi?",
                 cur_x, cur_y);
    return BT::NodeStatus::FAILURE;
  }

  // ── Adım 3: Mevcut rotadan HEDEF düğümünü al ──────────────────────────
  // Orijinal misyon hedefi: mevcut route'un SON segmentinin id'si
  std::shared_ptr<Route> existing_route;
  if (!globalRootBlackboard()->get("route_obj", existing_route) ||
      !existing_route || existing_route->empty()) {
    RCLCPP_ERROR(btLogger(),
                 "ReplanRoute: Blackboard'da geçerli route_obj bulunamadı!");
    return BT::NodeStatus::FAILURE;
  }
  std::string goal_id = existing_route->at(existing_route->size() - 1).id;

  // ── Adım 4: Yeni rota planla ─────────────────────────────────────────
  RCLCPP_WARN(btLogger(), "ReplanRoute: sebep=%s, başlangıç=%s → hedef=%s",
              reason.c_str(), start_id.c_str(), goal_id.c_str());

  auto new_route = globalSegmentGraph().planRoute({start_id, goal_id});

  if (new_route.empty()) {
    RCLCPP_ERROR(
        btLogger(),
        "ReplanRoute: Yeni rota planlama başarısız! başlangıç=%s hedef=%s",
        start_id.c_str(), goal_id.c_str());
    return BT::NodeStatus::FAILURE;
  }

  // ── Adım 5: Blackboard'u güncelle ──────────────────────────────────────
  auto new_route_ptr = std::make_shared<Route>(new_route);
  globalRootBlackboard()->set("route_obj", new_route_ptr);
  globalRootBlackboard()->set("route_size", static_cast<int>(new_route.size()));
  globalRootBlackboard()->set("seg_index", 0); // yeni rotadan başla

  RCLCPP_WARN(
      btLogger(),
      "ReplanRoute: TAMAM — sebep=%s, yeni rota %d segment (baş=%s, son=%s)",
      reason.c_str(), static_cast<int>(new_route.size()),
      new_route.at(0).id.c_str(),
      new_route.at(new_route.size() - 1).id.c_str());

  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus CalculateLaneChange::tick() {
  std::string target_lane;
  getInput("target_lane", target_lane);
  
  double cur_x, cur_y, cur_yaw;
  if (!OdometryProvider::instance().getPose(cur_x, cur_y, cur_yaw)) {
    RCLCPP_WARN(btLogger(), "CalculateLaneChange: Odometry yok, lane change hesaplanamıyor.");
    return BT::NodeStatus::FAILURE;
  }

  std::string current_id = globalSegmentGraph().findNearestNode(cur_x, cur_y);
  if (current_id.empty()) {
    RCLCPP_WARN(btLogger(), "CalculateLaneChange: Bulunulan segment tespit edilemedi.");
    return BT::NodeStatus::FAILURE;
  }

  std::string parallel_id = globalSegmentGraph().findParallelLanelet(current_id);
  if (parallel_id.empty()) {
    RCLCPP_WARN(btLogger(), "CalculateLaneChange: Yan şerit (paralel lanelet) bulunamadı.");
    return BT::NodeStatus::FAILURE;
  }

  const auto* node = globalSegmentGraph().getNode(parallel_id);
  if (!node) return BT::NodeStatus::FAILURE;

  std::ostringstream oss;
  // Varsayılan olarak yan şeridin hedef x,y'sini ve kendi yaw'ını veya şeridin yaw'ını kullan
  oss << node->x << ";" << node->y << ";" << cur_yaw; 
  globalRootBlackboard()->set("seg_goal", oss.str());

  RCLCPP_INFO(btLogger(), "CalculateLaneChange: %s -> %s (hedef: %s)", current_id.c_str(), parallel_id.c_str(), oss.str().c_str());
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus FindParkingSlot::tick() {
  // TODO: Algı/park sensörü entegre olunca doluluk kontrolü eklenecek. Şimdilik hepsi boş varsayılıyor.
  std::string point_str;
  if (!globalRootBlackboard()->get("seg_goal", point_str) || point_str.empty()) {
    return BT::NodeStatus::FAILURE;
  }
  
  double tgt_x = 0.0, tgt_y = 0.0, tgt_yaw = 0.0;
  std::istringstream iss(point_str);
  char delim;
  if (!(iss >> tgt_x >> delim >> tgt_y >> delim >> tgt_yaw)) {
    return BT::NodeStatus::FAILURE;
  }

  const auto& spots = globalSegmentGraph().parking_spots_;
  if (spots.empty()) {
    RCLCPP_WARN(btLogger(), "FindParkingSlot: Haritada park noktası yok.");
    return BT::NodeStatus::FAILURE;
  }

  double min_dist = std::numeric_limits<double>::max();
  const ParkingSpot* best_spot = nullptr;

  for (const auto& sp : spots) {
    double d = std::hypot(sp.x - tgt_x, sp.y - tgt_y);
    if (d < min_dist) {
      min_dist = d;
      best_spot = &sp;
    }
  }

  if (best_spot) {
    std::ostringstream oss;
    oss << best_spot->x << ";" << best_spot->y << ";" << tgt_yaw;
    setOutput("slot_pose", oss.str());
    RCLCPP_INFO(btLogger(), "FindParkingSlot: En yakın park noktası %s seçildi.", best_spot->vertex_id.c_str());
    return BT::NodeStatus::SUCCESS;
  }
  
  return BT::NodeStatus::FAILURE;
}

// ═══════════════ 🔧 SENSÖR BAĞIMLI STUB'LAR ═══════════════
// Condition stub'ları FAILURE döner = "tehlike yok"
// SafetyReflexes'te Inverter ile SUCCESS'a çevrilir → devam et

BT::NodeStatus PedestrianAhead::tick() {
  if (!sub_) {
    auto ros = getRosNode(config());
    if (ros) {
      sub_ = ros->create_subscription<std_msgs::msg::Bool>(
          kTopicPedestrian, rclcpp::SensorDataQoS(),
          [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
            last_val_ = msg->data;
            has_data_ = true;
          });
    }
  }
  if (!has_data_) return BT::NodeStatus::FAILURE;
  return last_val_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::NodeStatus DynamicObstacleAhead::tick() {
  if (!sub_) {
    auto ros = getRosNode(config());
    if (ros) {
      sub_ = ros->create_subscription<std_msgs::msg::Bool>(
          kTopicDynamicObstacle, rclcpp::SensorDataQoS(),
          [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
            last_val_ = msg->data;
            has_data_ = true;
          });
    }
  }
  if (!has_data_) return BT::NodeStatus::FAILURE;
  return last_val_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::NodeStatus StaticObstacleInLane::tick() {
  if (!sub_) {
    auto ros = getRosNode(config());
    if (ros) {
      sub_ = ros->create_subscription<std_msgs::msg::Bool>(
          kTopicStaticObstacle, rclcpp::SensorDataQoS(),
          [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
            last_val_ = msg->data;
            has_data_ = true;
          });
    }
  }
  if (!has_data_) return BT::NodeStatus::FAILURE;
  return last_val_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::NodeStatus AvoidanceSpaceAvailable::tick() {
  // TODO: Nav2 local_costmap/costmap_raw topic'inden şerit doluluğu okunacak, algı/Nav2 entegrasyonu netleşince yazılacak.
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus IsTwoWayRoad::tick() {
  std::string point_str;
  if (!globalRootBlackboard()->get("seg_goal", point_str) || point_str.empty()) {
    return BT::NodeStatus::FAILURE;
  }
  
  double cur_x, cur_y, cur_yaw;
  std::istringstream iss(point_str);
  char delim;
  if (iss >> cur_x >> delim >> cur_y >> delim >> cur_yaw) {
    std::string seg_id = globalSegmentGraph().findNearestNode(cur_x, cur_y);
    if (!seg_id.empty()) {
      auto* node = globalSegmentGraph().getNode(seg_id);
      if (node) {
        // Graf yapısında two_way şimdilik her zaman false ama mantık eklendi.
        // segment verilerine erişim için Segment structına ihtiyacımız var.
        for (const auto& s : globalSegmentGraph().allSegments()) {
          if (s.id == seg_id) {
            return s.two_way ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
          }
        }
      }
    }
  }
  
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus TrafficLightAhead::tick() {
  std::string color;
  if (!PerceptionProvider::instance().getTrafficLightColor(color)) {
    return BT::NodeStatus::FAILURE;
  }
  setOutput("light_color", color);
  return (color != "none" && color != "NONE") ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::NodeStatus StopSignAhead::tick() {
  if (!sub_) {
    auto ros = getRosNode(config());
    if (ros) {
      sub_ = ros->create_subscription<std_msgs::msg::Bool>(
          kTopicStopSign, rclcpp::SensorDataQoS(),
          [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
            last_val_ = msg->data;
            has_data_ = true;
          });
    }
  }
  if (!has_data_) return BT::NodeStatus::FAILURE;
  return last_val_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::NodeStatus GlobalRoadSignAhead::tick() {
  if (!sub_) {
    auto ros = getRosNode(config());
    if (ros) {
      sub_ = ros->create_subscription<std_msgs::msg::String>(
          kTopicRoadSign, rclcpp::SensorDataQoS(),
          [this](std_msgs::msg::String::ConstSharedPtr msg) {
            last_val_ = msg->data;
            has_data_ = true;
          });
    }
  }
  if (!has_data_) return BT::NodeStatus::FAILURE;
  setOutput("sign_type", last_val_);
  return (last_val_ != "none" && last_val_ != "NONE") ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::NodeStatus TurnConflictsWithSigns::tick() {
  std::string planned_turn, detected_signs;
  getInput("planned_turn", planned_turn);
  getInput("detected_signs", detected_signs);
  // TODO: Planlanan dönüş ile tabelalar çelişiyor mu kontrol et
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus CheckStopAccuracy::tick() {
  auto &odom = OdometryProvider::instance();

  // Veri yoksa veya bayatsa → FAILURE (güvenli taraf: "doğru durdum" diyemeyiz)
  if (!odom.isFresh()) {
    RCLCPP_WARN_THROTTLE(
        btLogger(), *getRosNode(config()), 2000,
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
  if (!globalRootBlackboard()->get("seg_goal", point_str) ||
      point_str.empty()) {
    RCLCPP_ERROR(btLogger(), "CheckStopAccuracy: 'point' portu boş!");
    return BT::NodeStatus::FAILURE;
  }

  double tgt_x = 0.0, tgt_y = 0.0, tgt_yaw = 0.0;
  {
    std::istringstream iss(point_str);
    char delim;
    if (!(iss >> tgt_x >> delim >> tgt_y >> delim >> tgt_yaw)) {
      RCLCPP_ERROR(btLogger(), "CheckStopAccuracy: 'point' parse hatası: '%s'",
                   point_str.c_str());
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
  double tgt_yaw_rad = tgt_yaw * M_PI / 180.0; // XML'de derece olarak gelir
  double yaw_err = cur_yaw - tgt_yaw_rad;
  // Normalize to [-π, π]
  while (yaw_err > M_PI)
    yaw_err -= 2.0 * M_PI;
  while (yaw_err < -M_PI)
    yaw_err += 2.0 * M_PI;

  bool pos_ok = dist <= tolerance;
  bool yaw_ok = std::fabs(yaw_err) <= heading_tolerance_rad;

  RCLCPP_INFO_THROTTLE(
      btLogger(), *getRosNode(config()), 1000,
      "CheckStopAccuracy: dist=%.3f (tol=%.2f) yaw_err=%.1f° (tol=%.1f°) → %s",
      dist, tolerance, yaw_err * 180.0 / M_PI, heading_tolerance_deg,
      (pos_ok && yaw_ok) ? "SUCCESS" : "FAILURE");

  return (pos_ok && yaw_ok) ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::NodeStatus IsStuck::tick() {
  auto &odom = OdometryProvider::instance();

  // Veri yoksa veya bayatsa → FAILURE (güvenli taraf: GPS kesildiyse
  // yanlışlıkla recovery moduna girmemeli)
  if (!odom.isFresh()) {
    stuck_active_ = false; // timer'ı sıfırla
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
      RCLCPP_WARN(
          btLogger(),
          "IsStuck: hız=%.3f m/s, %.1f sn boyunca düşük → STUCK (SUCCESS)",
          speed, elapsed);
      return BT::NodeStatus::SUCCESS; // takılmış!
    }
  } else {
    // Hız eşiğin üstünde, timer'ı sıfırla
    stuck_active_ = false;
  }

  return BT::NodeStatus::FAILURE;
}

// ═══════════════ ⚡ NAV2/HAREKET BAĞIMLI ═══════════════

BT::NodeStatus FollowLaneSegment::onStart() {
  // ── 1. "goal" portunu oku ve parse et ────────────────────────────────────
  std::string goal_str;
  if (!getInput("goal", goal_str) || goal_str.empty()) {
    // port bağlı değilse kök blackboard'dan dene
    if (!globalRootBlackboard()->get("seg_goal", goal_str) ||
        goal_str.empty()) {
      RCLCPP_ERROR(btLogger(),
                   "FollowLaneSegment: 'goal' portu/blackboard boş!");
      return BT::NodeStatus::FAILURE;
    }
  }

  double x = 0.0, y = 0.0, yaw = 0.0;
  try {
    size_t pos1 = 0, pos2 = 0;
    x = std::stod(goal_str, &pos1);              // ilk sayı
    pos1++;                                      // ';' atla
    y = std::stod(goal_str.substr(pos1), &pos2); // ikinci sayı
    pos1 += pos2 + 1;                            // ';' atla
    yaw = std::stod(goal_str.substr(pos1));      // üçüncü sayı
  } catch (const std::exception &e) {
    RCLCPP_ERROR(btLogger(), "FollowLaneSegment: goal parse hatası '%s': %s",
                 goal_str.c_str(), e.what());
    return BT::NodeStatus::FAILURE;
  }

  // ── 2. Lazy-init: action client ──────────────────────────────────────────
  auto ros = getRosNode(config());
  if (!ros)
    return BT::NodeStatus::FAILURE;

  if (!action_client_) {
    action_client_ =
        rclcpp_action::create_client<NavigateToPose>(ros, "navigate_to_pose");
  }

  // ── 3. Action server hazır mı? ────────────────────────────────────────────
  if (!action_client_->wait_for_action_server(std::chrono::seconds(10))) {
    RCLCPP_ERROR(
        btLogger(),
        "FollowLaneSegment: Nav2 navigate_to_pose action server bulunamadı, "
        "Nav2 sistemi çalışıyor mu kontrol edin");
    return BT::NodeStatus::FAILURE;
  }

  // ── 4. Goal mesajı oluştur ───────────────────────────────────────────────
  NavigateToPose::Goal nav_goal;
  nav_goal.pose.header.frame_id = "map";
  nav_goal.pose.header.stamp = ros->now();
  nav_goal.pose.pose.position.x = x;
  nav_goal.pose.pose.position.y = y;
  nav_goal.pose.pose.position.z = 0.0;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw);
  nav_goal.pose.pose.orientation = tf2::toMsg(q);

  // ── 5. Goal gönder, result callback bağla ───────────────────────────────
  goal_finished_ = false;
  goal_succeeded_ = false;
  goal_handle_ = nullptr;

  rclcpp_action::Client<NavigateToPose>::SendGoalOptions opts;

  opts.goal_response_callback = [this](const GoalHandle::SharedPtr &handle) {
    goal_handle_ = handle;
    if (!handle) {
      RCLCPP_ERROR(btLogger(), "FollowLaneSegment: Nav2 goal REDDEDİLDİ!");
      goal_finished_ = true;
      goal_succeeded_ = false;
    }
  };

  opts.result_callback = [this](const GoalHandle::WrappedResult &result) {
    goal_finished_ = true;
    goal_succeeded_ = (result.code == rclcpp_action::ResultCode::SUCCEEDED);
  };

  goal_handle_future_ = action_client_->async_send_goal(nav_goal, opts);
  goal_sent_ = true;

  RCLCPP_INFO(btLogger(),
              "FollowLaneSegment: NavigateToPose hedefi gönderildi x=%.2f "
              "y=%.2f yaw=%.2f",
              x, y, yaw);

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus FollowLaneSegment::onRunning() {
  // Callback'lerin işlenmesi için spin_some (ana döngü de yapıyor, ama güvenli)
  auto ros = getRosNode(config());
  if (ros)
    rclcpp::spin_some(ros);

  if (goal_finished_) {
    if (goal_succeeded_) {
      RCLCPP_INFO(btLogger(),
                  "FollowLaneSegment: Nav2 hedefe ulaşıldı → SUCCESS");
      return BT::NodeStatus::SUCCESS;
    } else {
      RCLCPP_ERROR(btLogger(),
                   "FollowLaneSegment: Nav2 hedefe ulaşamadı → FAILURE");
      return BT::NodeStatus::FAILURE;
    }
  }

  return BT::NodeStatus::RUNNING;
}

void FollowLaneSegment::onHalted() {
  if (goal_handle_ && action_client_) {
    action_client_->async_cancel_goal(goal_handle_);
    RCLCPP_WARN(btLogger(), "FollowLaneSegment: HALTED — Nav2 goal iptal "
                            "edildi (async_cancel_goal)");
  } else {
    RCLCPP_WARN(btLogger(),
                "FollowLaneSegment: HALTED (henüz goal handle yok)");
  }
  goal_sent_ = false;
  goal_finished_ = false;
  goal_succeeded_ = false;
}

BT::NodeStatus WaitForClear::onStart() {
  std::string hazard;
  getInput("hazard", hazard);
  RCLCPP_INFO(btLogger(), "WaitForClear: STUB %s bekleniyor", hazard.c_str());
  start_time_ = std::chrono::steady_clock::now();
  return BT::NodeStatus::RUNNING;
}
BT::NodeStatus WaitForClear::onRunning() {
  if (std::chrono::steady_clock::now() - start_time_ >
      std::chrono::seconds(3)) {
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::RUNNING;
}
void WaitForClear::onHalted() {
  RCLCPP_WARN(btLogger(), "WaitForClear: HALTED");
}

BT::NodeStatus WaitForGreenLight::onStart() {
  RCLCPP_INFO(btLogger(), "WaitForGreenLight: STUB bekleniyor");
  return BT::NodeStatus::RUNNING;
}
BT::NodeStatus WaitForGreenLight::onRunning() {
  std::string color;
  if (PerceptionProvider::instance().getTrafficLightColor(color)) {
    if (color == "green" || color == "GREEN") {
      RCLCPP_INFO(btLogger(), "WaitForGreenLight: Yeşil ışık tespit edildi.");
      return BT::NodeStatus::SUCCESS;
    }
  }
  return BT::NodeStatus::RUNNING;
}
void WaitForGreenLight::onHalted() {
  RCLCPP_WARN(btLogger(), "WaitForGreenLight: HALTED");
}

BT::NodeStatus ProceedOnGreen::tick() {
  RCLCPP_INFO(btLogger(), "ProceedOnGreen: STUB");
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus StopAtStopLine::onStart() {
  RCLCPP_INFO(btLogger(), "StopAtStopLine: Mesafe kontrolü başlıyor.");
  return BT::NodeStatus::RUNNING;
}
BT::NodeStatus StopAtStopLine::onRunning() {
  if (!sub_) {
    auto ros = getRosNode(config());
    if (ros) {
      sub_ = ros->create_subscription<std_msgs::msg::Float32>(
          kTopicStopLineDistance, rclcpp::SensorDataQoS(),
          [this](std_msgs::msg::Float32::ConstSharedPtr msg) {
            last_val_ = msg->data;
            has_data_ = true;
          });
    }
  }
  if (!has_data_) return BT::NodeStatus::RUNNING; // Bilgi yoksa bekle (Stateful mantığı)
  
  double tolerance = 1.0;
  getInput("tolerance", tolerance);

  if (last_val_ >= 0 && last_val_ <= tolerance) {
    RCLCPP_INFO(btLogger(), "StopAtStopLine: Dur çizgisine ulaşıldı (%.2fm).", last_val_);
    return BT::NodeStatus::SUCCESS;
  }
  
  return BT::NodeStatus::RUNNING;
}
void StopAtStopLine::onHalted() {
  RCLCPP_WARN(btLogger(), "StopAtStopLine: HALTED");
}

BT::NodeStatus YieldAtRoundabout::tick() {
  // TODO: Dinamik engel algısı (kavşak içi araçlar) entegre edilerek kavşakta bekleme/geçme kararı verilecek. Şimdilik STUB.
  RCLCPP_INFO(btLogger(), "YieldAtRoundabout: STUB");
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus ExecuteParking::onStart() {
  // TODO: gerçek dik park manevrası Nav2 veya özel bir controller ile yapılacak, şu an zaman bazlı simülasyon.
  RCLCPP_INFO(btLogger(), "ExecuteParking: STUB START");
  start_time_ = std::chrono::steady_clock::now();
  return BT::NodeStatus::RUNNING;
}
BT::NodeStatus ExecuteParking::onRunning() {
  double limit = 180.0;
  getInput("time_limit_sec", limit);
  auto sec = std::chrono::duration<double>(std::chrono::steady_clock::now() -
                                           start_time_)
                 .count();
  if (sec >= limit)
    return BT::NodeStatus::FAILURE;
  if (sec >= 5.0)
    return BT::NodeStatus::SUCCESS;
  return BT::NodeStatus::RUNNING;
}
void ExecuteParking::onHalted() {
  RCLCPP_WARN(btLogger(), "ExecuteParking: HALTED");
}

BT::NodeStatus BackUpAction::tick() {
  double dist = 0.3, speed = 0.05;
  getInput("backup_dist", dist);
  getInput("backup_speed", speed);
  RCLCPP_INFO(btLogger(), "BackUp: STUB dist=%.2f speed=%.2f", dist, speed);
  // TODO: /cmd_vel'e negatif linear.x publish
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus SpinAction::tick() {
  double dist = 1.57;
  getInput("spin_dist", dist);
  RCLCPP_INFO(btLogger(), "Spin: STUB dist=%.2f rad", dist);
  // TODO: /cmd_vel'e angular.z publish
  return BT::NodeStatus::SUCCESS;
}

} // namespace robotaksi_bt
