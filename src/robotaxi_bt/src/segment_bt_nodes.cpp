// ============================================================================
// Teknofest 2026 Robotaksi — Segment BT Custom Node İmplementasyonları
//
// segment_bt.xml'deki TÜM custom node'ların tick() implementasyonları.
//
// STUB MANTIGI:
//   - Her node çalıştığında RCLCPP_INFO ile loglar (debug için)
//   - Condition node'ları varsayılan olarak FAILURE döner (tehlike YOK)
//   - Action node'ları varsayılan olarak SUCCESS döner
//   - Stateful node'lar (RUNNING dönmesi gerekenler) timer ile simüle eder
//
// TAKIM İÇİN:
//   Bu stub'lar derlenir ve çalışır. Gerçek sensör/actuator bağlantılarını
//   "// TODO: Gerçek implementasyon" yorumlu yerlere doldurun.
// ============================================================================

#include "robotaxi_bt/segment_bt_nodes.hpp"

namespace robotaxi_bt {

static auto logger() { return rclcpp::get_logger("segment_bt"); }

// ═════════════════════════════════════════════════════════════
// ACTION NODE İMPLEMENTASYONLARI
// ═════════════════════════════════════════════════════════════

// ─── LoadMission ───
BT::NodeStatus LoadMission::tick()
{
  std::string geojson_file;
  int tour = 1;
  getInput("geojson_file", geojson_file);
  getInput("tour", tour);

  RCLCPP_INFO(logger(), "LoadMission: geojson=%s, tur=%d", geojson_file.c_str(), tour);

  // TODO: Gerçek implementasyon
  // 1. geojson_file'ı oku → görev noktalarını çıkar
  // 2. segment_map.yaml'ı yükle (graph_.loadFromYAML(...))
  // 3. Görev noktalarını en yakın graf düğümlerine eşle
  // 4. graph_.planRoute() ile segment rotası oluştur
  // 5. Tur bazında yolcu noktalarını dahil et/çıkar

  // Stub: boş rota oluştur
  setOutput("route", std::string("[]"));
  setOutput("route_size", 0);
  setOutput("seg_index", 0);

  RCLCPP_WARN(logger(), "LoadMission: STUB — gerçek graf/rota henüz yüklenmedi");
  return BT::NodeStatus::SUCCESS;
}

// ─── GetCurrentSegment ───
BT::NodeStatus GetCurrentSegment::tick()
{
  std::string route;
  int seg_index = 0;
  getInput("route", route);
  getInput("seg_index", seg_index);

  // TODO: Gerçek implementasyon
  // route JSON'ını parse et, seg_index'teki segmenti al,
  // seg_type, seg_goal, seg_meta çıktılarını set et.

  RCLCPP_INFO(logger(), "GetCurrentSegment: index=%d", seg_index);

  // Stub: varsayılan LANE_FOLLOW
  setOutput("seg_type", std::string("LANE_FOLLOW"));
  setOutput("seg_goal", std::string("0.0;0.0;0.0"));
  setOutput("seg_meta", std::string(""));

  return BT::NodeStatus::SUCCESS;
}

// ─── AdvanceSegment ───
BT::NodeStatus AdvanceSegment::tick()
{
  int seg_index = 0;
  getInput("seg_index", seg_index);
  seg_index++;
  setOutput("seg_index", seg_index);
  RCLCPP_INFO(logger(), "AdvanceSegment: yeni index=%d", seg_index);
  return BT::NodeStatus::SUCCESS;
}

// ─── SetMaxSpeed ───
BT::NodeStatus SetMaxSpeed::tick()
{
  double speed = 0.0;
  getInput("speed", speed);
  RCLCPP_INFO(logger(), "SetMaxSpeed: %.2f m/s", speed);

  // TODO: Gerçek implementasyon
  // Nav2 controller'a hız limiti gönder veya cmd_vel multiplier ayarla
  // Örnek: dynamic_reconfigure ile max_vel_x parametresini güncelle

  return BT::NodeStatus::SUCCESS;
}

// ─── FollowLaneSegment ───
BT::NodeStatus FollowLaneSegment::onStart()
{
  std::string goal;
  getInput("goal", goal);
  RCLCPP_INFO(logger(), "FollowLaneSegment: START → goal=%s", goal.c_str());

  // TODO: Gerçek implementasyon
  // SEÇENEK A: Vision lane-following başlat (cmd_vel üzerinden)
  //   - Kameradan şerit tespiti → PID ile direksiyon
  //   - goal pozisyonuna ulaşınca SUCCESS
  // SEÇENEK B: Nav2 FollowPath/NavigateToPose çağır
  //   - action client ile goal gönder
  //   - feedback'te mesafe kontrol et

  start_time_ = std::chrono::steady_clock::now();
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus FollowLaneSegment::onRunning()
{
  // TODO: Gerçek implementasyon
  // - Hedefe mesafeyi kontrol et
  // - Hedefe ulaştıysa SUCCESS
  // - Hâlâ yoldaysa RUNNING
  // - Hata oluştuysa FAILURE

  // Stub: 2 saniye sonra SUCCESS dön (simülasyon)
  auto elapsed = std::chrono::steady_clock::now() - start_time_;
  if (elapsed > std::chrono::seconds(2)) {
    RCLCPP_INFO(logger(), "FollowLaneSegment: hedefe ulaşıldı (stub)");
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::RUNNING;
}

void FollowLaneSegment::onHalted()
{
  RCLCPP_WARN(logger(), "FollowLaneSegment: HALTED (güvenlik refleksi?)");
  // TODO: Nav2 action'ı iptal et veya cmd_vel sıfırla
}

// ─── StopVehicle ───
BT::NodeStatus StopVehicle::tick()
{
  RCLCPP_INFO(logger(), "StopVehicle: tam dur");

  // TODO: Gerçek implementasyon
  // geometry_msgs::msg::Twist zero_vel;
  // cmd_vel_pub->publish(zero_vel);

  return BT::NodeStatus::SUCCESS;
}

// ─── WaitForClear ───
BT::NodeStatus WaitForClear::onStart()
{
  std::string hazard;
  getInput("hazard", hazard);
  RCLCPP_INFO(logger(), "WaitForClear: %s tehlikesi temizlenene dek bekleniyor", hazard.c_str());
  start_time_ = std::chrono::steady_clock::now();
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus WaitForClear::onRunning()
{
  // TODO: Gerçek implementasyon
  // İlgili sensör topic'ini kontrol et:
  //   - hazard=="pedestrian" → yaya topic'i
  //   - hazard=="dynamic"   → dinamik engel topic'i
  //   - hazard=="static"    → statik engel topic'i
  // Temizlendiyse SUCCESS, hâlâ varsa RUNNING

  // Stub: 3 saniye sonra temizlendi say
  auto elapsed = std::chrono::steady_clock::now() - start_time_;
  if (elapsed > std::chrono::seconds(3)) {
    RCLCPP_INFO(logger(), "WaitForClear: tehlike temizlendi (stub)");
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::RUNNING;
}

void WaitForClear::onHalted()
{
  RCLCPP_WARN(logger(), "WaitForClear: HALTED");
}

// ─── StopAtStopLine ───
BT::NodeStatus StopAtStopLine::tick()
{
  double tolerance = 5.0;
  getInput("tolerance", tolerance);
  RCLCPP_INFO(logger(), "StopAtStopLine: tolerans=%.1fm", tolerance);

  // TODO: Gerçek implementasyon
  // Kamera/Lidar ile stop çizgisini tespit et
  // 0 < mesafe < tolerance aralığında dur
  // Şartname: 0<d<5m → +60 puan

  return BT::NodeStatus::SUCCESS;
}

// ─── WaitForGreenLight ───
BT::NodeStatus WaitForGreenLight::onStart()
{
  RCLCPP_INFO(logger(), "WaitForGreenLight: yeşil ışık bekleniyor");
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus WaitForGreenLight::onRunning()
{
  // TODO: Gerçek implementasyon
  // Trafik ışığı topic'inden renk oku
  // Yeşil → SUCCESS, kırmızı/sarı → RUNNING

  // Stub: hemen yeşil say
  RCLCPP_INFO(logger(), "WaitForGreenLight: yeşil ışık (stub)");
  return BT::NodeStatus::SUCCESS;
}

void WaitForGreenLight::onHalted()
{
  RCLCPP_WARN(logger(), "WaitForGreenLight: HALTED");
}

// ─── ProceedOnGreen ───
BT::NodeStatus ProceedOnGreen::tick()
{
  double max_react = 5.0;
  getInput("max_react_sec", max_react);
  RCLCPP_INFO(logger(), "ProceedOnGreen: max_react=%.1fs", max_react);

  // TODO: Gerçek implementasyon
  // Yeşil ışık görüldüğünde hızlıca kalkış yap
  // Şartname: 5 sn içinde kalkış → +40 puan

  return BT::NodeStatus::SUCCESS;
}

// ─── StopAndProceed ───
BT::NodeStatus StopAndProceed::tick()
{
  RCLCPP_INFO(logger(), "StopAndProceed: DUR tabelası — tam dur, sonra devam");

  // TODO: Gerçek implementasyon
  // 1. StopVehicle çağır
  // 2. 2-3 saniye bekle
  // 3. Devam et

  return BT::NodeStatus::SUCCESS;
}

// ─── CalculateLaneChange ───
BT::NodeStatus CalculateLaneChange::tick()
{
  std::string current_pose, target_lane;
  getInput("current_pose", current_pose);
  getInput("target_lane", target_lane);
  RCLCPP_INFO(logger(), "CalculateLaneChange: hedef_serit=%s", target_lane.c_str());

  // TODO: Gerçek implementasyon
  // Mevcut pozisyon + hedef şerit → yeni seg_goal hesapla
  // Genellikle yanal offset (±3.5m standart şerit genişliği)

  setOutput("seg_goal", std::string("0.0;0.0;0.0"));
  return BT::NodeStatus::SUCCESS;
}

// ─── ReplanRoute ───
BT::NodeStatus ReplanRoute::tick()
{
  std::string reason;
  getInput("reason", reason);
  RCLCPP_WARN(logger(), "ReplanRoute: sebep=%s — rota yeniden planlanıyor", reason.c_str());

  // TODO: Gerçek implementasyon
  // 1. Mevcut pozisyonu al
  // 2. Kalan görev noktalarını belirle
  // 3. Segment grafı üzerinde yeni rota hesapla
  // 4. route, route_size, seg_index çıktılarını güncelle

  setOutput("route", std::string("[]"));
  setOutput("route_size", 0);
  setOutput("seg_index", 0);

  return BT::NodeStatus::SUCCESS;
}

// ─── TurnHeadlights ───
BT::NodeStatus TurnHeadlights::tick()
{
  std::string state;
  getInput("state", state);
  RCLCPP_INFO(logger(), "TurnHeadlights: %s", state.c_str());

  // TODO: Gerçek implementasyon
  // GPIO pin toggle veya araç CAN bus komutu gönder

  return BT::NodeStatus::SUCCESS;
}

// ─── YieldAtRoundabout ───
BT::NodeStatus YieldAtRoundabout::tick()
{
  RCLCPP_INFO(logger(), "YieldAtRoundabout: dönel kavşak girişinde boşluk bekleniyor");

  // TODO: Gerçek implementasyon
  // Lidar/kamera ile dönel kavşak trafiğini kontrol et
  // Boşluk varsa SUCCESS, yoksa FAILURE → tekrar dene

  return BT::NodeStatus::SUCCESS;
}

// ─── ExecuteRoundabout ───
BT::NodeStatus ExecuteRoundabout::tick()
{
  std::string exit_node;
  getInput("exit_node", exit_node);
  RCLCPP_INFO(logger(), "ExecuteRoundabout: çıkış=%s", exit_node.c_str());

  // TODO: Gerçek implementasyon
  // Dönel kavşak içinde seyir + doğru çıkıştan ayrılma

  return BT::NodeStatus::SUCCESS;
}

// ─── Dwell ───
BT::NodeStatus Dwell::onStart()
{
  double min_sec = 15.0, max_sec = 20.0;
  getInput("min_sec", min_sec);
  getInput("max_sec", max_sec);
  wait_duration_sec_ = (min_sec + max_sec) / 2.0;  // orta değer
  RCLCPP_INFO(logger(), "Dwell: %.0f sn bekleniyor (min=%.0f, max=%.0f)", wait_duration_sec_, min_sec, max_sec);
  start_time_ = std::chrono::steady_clock::now();
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus Dwell::onRunning()
{
  auto elapsed = std::chrono::steady_clock::now() - start_time_;
  double sec = std::chrono::duration<double>(elapsed).count();
  if (sec >= wait_duration_sec_) {
    RCLCPP_INFO(logger(), "Dwell: %.0f sn bekleme tamamlandı", sec);
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::RUNNING;
}

void Dwell::onHalted()
{
  RCLCPP_WARN(logger(), "Dwell: HALTED (bekleme kesildi)");
}

// ─── SignalPassengerEvent ───
BT::NodeStatus SignalPassengerEvent::tick()
{
  std::string event_type;
  getInput("event_type", event_type);
  RCLCPP_INFO(logger(), "SignalPassengerEvent: %s", event_type.c_str());

  // TODO: Gerçek implementasyon
  // PLC'ye yolcu al/bırak sinyali gönder
  // Veya HMI/dashboard'a bildirim

  return BT::NodeStatus::SUCCESS;
}

// ─── RecordMissionPoint ───
BT::NodeStatus RecordMissionPoint::tick()
{
  std::string point;
  double tolerance = 1.0;
  getInput("point", point);
  getInput("tolerance", tolerance);
  RCLCPP_INFO(logger(), "RecordMissionPoint: nokta=%s, tolerans=%.1fm", point.c_str(), tolerance);

  // TODO: Gerçek implementasyon
  // Mevcut pozisyonu al, görev noktasıyla mesafeyi hesapla
  // 1m içindeyse → puanlama sistemine kaydet (+30)

  return BT::NodeStatus::SUCCESS;
}

// ─── RecordParkEntryReached ───
BT::NodeStatus RecordParkEntryReached::tick()
{
  RCLCPP_INFO(logger(), "RecordParkEntryReached: park giriş noktası geçildi (+20)");

  // TODO: Gerçek implementasyon
  // Puanlama sistemine park girişi kaydı

  return BT::NodeStatus::SUCCESS;
}

// ─── FindParkingSlot ───
BT::NodeStatus FindParkingSlot::tick()
{
  RCLCPP_INFO(logger(), "FindParkingSlot: uygun slot aranıyor");

  // TODO: Gerçek implementasyon
  // Kamera/Lidar ile:
  //   1. P-3a tabelası olan slotları bul
  //   2. P-1 (park yasak) olmayan slotları filtrele
  //   3. Önü boş olan slotu seç
  // Bulunan slot pozisyonunu çıktıya yaz

  setOutput("slot_pose", std::string("0.0;0.0;0.0"));
  return BT::NodeStatus::SUCCESS;
}

// ─── ExecuteParking ───
BT::NodeStatus ExecuteParking::onStart()
{
  std::string slot;
  double time_limit = 180.0;
  getInput("slot", slot);
  getInput("time_limit_sec", time_limit);
  RCLCPP_INFO(logger(), "ExecuteParking: START — slot=%s, limit=%.0fs", slot.c_str(), time_limit);

  // TODO: Gerçek implementasyon
  // Dik (perpendicular) park manevrası:
  //   1. Park slotuna yaklaş
  //   2. Geri vitese geç
  //   3. Direksiyon açısı + geri sürüş
  //   4. Park tamamlandığında SUCCESS
  //   5. 3 dakika aşılırsa FAILURE

  start_time_ = std::chrono::steady_clock::now();
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus ExecuteParking::onRunning()
{
  auto elapsed = std::chrono::steady_clock::now() - start_time_;
  double sec = std::chrono::duration<double>(elapsed).count();

  double time_limit = 180.0;
  getInput("time_limit_sec", time_limit);

  if (sec >= time_limit) {
    RCLCPP_ERROR(logger(), "ExecuteParking: ZAMAN AŞIMI — %.0f sn", sec);
    return BT::NodeStatus::FAILURE;
  }

  // Stub: 5 saniye sonra park tamamlandı
  if (sec >= 5.0) {
    RCLCPP_INFO(logger(), "ExecuteParking: park tamamlandı (+80) (stub)");
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::RUNNING;
}

void ExecuteParking::onHalted()
{
  RCLCPP_WARN(logger(), "ExecuteParking: HALTED");
}

// ─── BackUp ───
BT::NodeStatus BackUpAction::tick()
{
  double dist = 0.3, speed = 0.05;
  getInput("backup_dist", dist);
  getInput("backup_speed", speed);
  RCLCPP_INFO(logger(), "BackUp: dist=%.2fm, speed=%.2fm/s", dist, speed);

  // TODO: Gerçek implementasyon
  // Nav2 BackUp action server'ı çağır veya
  // doğrudan cmd_vel ile negatif hız gönder

  return BT::NodeStatus::SUCCESS;
}

// ─── Spin ───
BT::NodeStatus SpinAction::tick()
{
  double dist = 1.57;
  getInput("spin_dist", dist);
  RCLCPP_INFO(logger(), "Spin: dist=%.2f rad", dist);

  // TODO: Gerçek implementasyon
  // Nav2 Spin action server'ı çağır veya
  // doğrudan cmd_vel ile angular.z gönder

  return BT::NodeStatus::SUCCESS;
}


// ═════════════════════════════════════════════════════════════
// CONDITION NODE İMPLEMENTASYONLARI
//
// Varsayılan: FAILURE (tehlike/koşul YOK)
// ReactiveSequence'ta SafetyReflexes içinde kullanıldığında:
//   FAILURE = "tehlike yok → devam et" (Inverter ile SUCCESS'a çevrilir)
// ═════════════════════════════════════════════════════════════

// ─── HasMoreSegments ───
BT::NodeStatus HasMoreSegments::tick()
{
  int seg_index = 0, route_size = 0;
  getInput("seg_index", seg_index);
  getInput("route_size", route_size);

  bool has_more = (seg_index < route_size);
  RCLCPP_DEBUG(logger(), "HasMoreSegments: %d / %d → %s",
               seg_index, route_size, has_more ? "TRUE" : "FALSE");

  return has_more ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

// ─── IsSegmentType ───
BT::NodeStatus IsSegmentType::tick()
{
  std::string seg_type, expected;
  getInput("seg_type", seg_type);
  getInput("expected", expected);

  bool match = (seg_type == expected);
  RCLCPP_DEBUG(logger(), "IsSegmentType: %s == %s → %s",
               seg_type.c_str(), expected.c_str(), match ? "TRUE" : "FALSE");

  return match ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

// ─── IsTrafficControlActive ───
BT::NodeStatus IsTrafficControlActive::tick()
{
  int tour = 1;
  getInput("tour", tour);

  bool active = (tour >= 2);
  RCLCPP_DEBUG(logger(), "IsTrafficControlActive: tur=%d → %s",
               tour, active ? "AKTIF" : "PASIF");

  return active ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

// ─── EmergencyStopRequested ───
BT::NodeStatus EmergencyStopRequested::tick()
{
  // TODO: Gerçek implementasyon
  // UMS-1 acil stop butonu topic'ini kontrol et
  // Basıldıysa SUCCESS (tehlike VAR), basılmadıysa FAILURE

  return BT::NodeStatus::FAILURE;  // varsayılan: acil durum YOK
}

// ─── PedestrianAhead ───
BT::NodeStatus PedestrianAhead::tick()
{
  // TODO: Gerçek implementasyon
  // Yaya algılama topic'ini kontrol et (YOLO/kamera)
  // Yaya varsa SUCCESS, yoksa FAILURE

  return BT::NodeStatus::FAILURE;  // varsayılan: yaya YOK
}

// ─── DynamicObstacleAhead ───
BT::NodeStatus DynamicObstacleAhead::tick()
{
  // TODO: Gerçek implementasyon
  // Dinamik engel algılama topic'ini kontrol et (Lidar clustering)
  // Engel varsa SUCCESS, yoksa FAILURE

  return BT::NodeStatus::FAILURE;  // varsayılan: dinamik engel YOK
}

// ─── StaticObstacleInLane ───
BT::NodeStatus StaticObstacleInLane::tick()
{
  // TODO: Gerçek implementasyon
  // Costmap / Lidar ile şeritteki statik engeli kontrol et
  // Engel varsa SUCCESS, yoksa FAILURE

  return BT::NodeStatus::FAILURE;  // varsayılan: statik engel YOK
}

// ─── AvoidanceSpaceAvailable ───
BT::NodeStatus AvoidanceSpaceAvailable::tick()
{
  // TODO: Gerçek implementasyon
  // Yan şeridin boş olup olmadığını kontrol et
  // Boşsa SUCCESS + free_lane çıktısı

  return BT::NodeStatus::FAILURE;  // varsayılan: sakınma alanı YOK
}

// ─── TrafficLightAhead ───
BT::NodeStatus TrafficLightAhead::tick()
{
  // TODO: Gerçek implementasyon
  // Kamera trafik ışığı tespiti
  // Işık görünüyorsa SUCCESS + light_color çıktısı
  // Görünmüyorsa FAILURE

  return BT::NodeStatus::FAILURE;  // varsayılan: ışık YOK
}

// ─── IsLightRed ───
BT::NodeStatus IsLightRed::tick()
{
  std::string color;
  getInput("color", color);

  bool is_red = (color == "red" || color == "RED" || color == "kirmizi");
  RCLCPP_DEBUG(logger(), "IsLightRed: color=%s → %s", color.c_str(), is_red ? "KIRMIZI" : "DEGIL");

  return is_red ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

// ─── StopSignAhead ───
BT::NodeStatus StopSignAhead::tick()
{
  // TODO: Gerçek implementasyon
  // Kamera tabela tespiti — DUR (TT-2) tabelası var mı?

  return BT::NodeStatus::FAILURE;  // varsayılan: DUR tabelası YOK
}

// ─── TurnConflictsWithSigns ───
BT::NodeStatus TurnConflictsWithSigns::tick()
{
  std::string planned_turn, detected_signs;
  getInput("planned_turn", planned_turn);
  getInput("detected_signs", detected_signs);

  // TODO: Gerçek implementasyon
  // Planlanan dönüş yönü ile algılanan tabelalar arasında çelişki kontrolü
  // Örnek: planned_turn="left" ama TT-4 "Sola dönülmez" tabelası var → çelişki

  RCLCPP_DEBUG(logger(), "TurnConflictsWithSigns: plan=%s, signs=%s",
               planned_turn.c_str(), detected_signs.c_str());

  return BT::NodeStatus::FAILURE;  // varsayılan: çelişki YOK
}

// ─── CheckStopAccuracy ───
BT::NodeStatus CheckStopAccuracy::tick()
{
  std::string point;
  double tolerance = 1.0;
  getInput("point", point);
  getInput("tolerance", tolerance);

  // TODO: Gerçek implementasyon
  // Mevcut pozisyon ile hedef noktayı karşılaştır
  // Mesafe < tolerance → SUCCESS (+70 puan yolcu noktası)
  // Mesafe >= tolerance → FAILURE (pozisyon düzeltme gerekli)

  RCLCPP_INFO(logger(), "CheckStopAccuracy: nokta=%s, tolerans=%.1fm (stub: OK)",
              point.c_str(), tolerance);

  return BT::NodeStatus::SUCCESS;  // stub: her zaman doğru dur
}

// ─── IsStuck ───
BT::NodeStatus IsStuck::tick()
{
  // TODO: Gerçek implementasyon
  // Odometre verisi ile hareket kontrolü:
  //   - Son N saniyede mesafe < eşik → sıkışmış (SUCCESS)
  //   - Hareket ediyorsa → sıkışmamış (FAILURE)

  return BT::NodeStatus::FAILURE;  // varsayılan: sıkışmamış
}

// ═════════════════════════════════════════════════════════════
// GLOBAL YOL LEVHASI NODE'LARI (KATMAN-5)
// Her segmentte aktif — LANE_FOLLOW dahil!
// B-50h/i, B-14a, hız limiti levhaları burada ele alınır.
// ═════════════════════════════════════════════════════════════

// ─── GlobalRoadSignAhead ───
// Kamera/tabela algılama sisteminden kavşak-dışı levha oku.
// NONE dönerse "levha yok" → Fallback'te Inverter SUCCESS → katman atlanır
BT::NodeStatus GlobalRoadSignAhead::tick()
{
  // TODO: Gerçek implementasyon
  // Kamera tabela tespiti topic'ini oku (tüm segmentte aktif).
  // Sadece kavşak-dışı levhalar:
  //   B-50h/i → setOutput("sign_type", "LANE_MERGE") → SUCCESS
  //   B-14a   → setOutput("sign_type", "PEDESTRIAN_CROSS") → SUCCESS
  //   Hız lim → setOutput("sign_type", "SPEED_LIMIT_30") → SUCCESS
  //   Levha yok veya kavşak tipi → setOutput("sign_type", "NONE") → FAILURE
  //
  // Kavşak-spesifik levhalar (TT-4, TT-26, DUR, ışık) bu node'da
  // işlenmez — onlar Seg_Intersection içinde ele alınır.

  setOutput("sign_type", std::string("NONE"));
  return BT::NodeStatus::FAILURE;  // varsayılan: global levha YOK
}

// ─── IsRoadSignType ───
BT::NodeStatus IsRoadSignType::tick()
{
  std::string sign_type, expected;
  getInput("sign_type", sign_type);
  getInput("expected", expected);

  bool match = (sign_type == expected);
  RCLCPP_DEBUG(logger(), "IsRoadSignType: %s == %s → %s",
               sign_type.c_str(), expected.c_str(), match ? "EVET" : "HAYIR");

  return match ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

// ─── ApplySpeedLimit ───
BT::NodeStatus ApplySpeedLimit::tick()
{
  std::string sign_data;
  getInput("sign_data", sign_data);
  RCLCPP_INFO(logger(), "ApplySpeedLimit: levha=%s", sign_data.c_str());

  // TODO: Gerçek implementasyon
  // sign_data'dan hız limitini parse et (örn: "SPEED_LIMIT_30" → 30 km/h = 0.83 m/s)
  // SetMaxSpeed mantığıyla controller'a yeni limiti bildir.
  // Şartname: hız limiti aşımı → -puan riski

  return BT::NodeStatus::SUCCESS;
}

// ─── LogUnknownSign ───
BT::NodeStatus LogUnknownSign::tick()
{
  std::string sign_type;
  getInput("sign_type", sign_type);
  RCLCPP_WARN(logger(), "LogUnknownSign: tanımsız levha tipi = '%s'", sign_type.c_str());
  return BT::NodeStatus::SUCCESS;
}

// ═════════════════════════════════════════════════════════════
// v3 YENİ NODE'LAR
// ═════════════════════════════════════════════════════════════

// ─── ClearHandledFlags: segment geçişinde latch bayraklarını sıfırla ───
BT::NodeStatus ClearHandledFlags::tick()
{
  // TODO: Gerçek implementasyon
  // Blackboard'daki handled_stop_sign, handled_traffic_light gibi
  // bayrakları false'a çek. Bu sayede yeni segmentte aynı tabela
  // tekrar tetiklenebilir ama aynı segment içinde sonsuz döngü olmaz.
  RCLCPP_DEBUG(logger(), "ClearHandledFlags: bayraklar sıfırlandı");
  return BT::NodeStatus::SUCCESS;
}

// ─── WaitForGoSignal: UMS-2 start sinyali bekle ───
BT::NodeStatus WaitForGoSignal::onStart()
{
  RCLCPP_INFO(logger(), "WaitForGoSignal: UMS-2 Go sinyali bekleniyor...");
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus WaitForGoSignal::onRunning()
{
  // TODO: Gerçek implementasyon
  // UMS-2 topic'ini dinle. Go sinyali geldiğinde SUCCESS dön.
  // Stub: hemen başlat
  RCLCPP_INFO(logger(), "WaitForGoSignal: Go sinyali alındı (stub)");
  return BT::NodeStatus::SUCCESS;
}

void WaitForGoSignal::onHalted()
{
  RCLCPP_WARN(logger(), "WaitForGoSignal: HALTED");
}

// ─── IsTwoWayRoad: iki yönlü yol kontrolü ───
BT::NodeStatus IsTwoWayRoad::tick()
{
  // TODO: Gerçek implementasyon
  // Harita verisinden (segment_map.yaml) veya B-52a tabelasından
  // mevcut yolun iki yönlü olup olmadığını kontrol et.
  // İki yönlüyse SUCCESS → karşı şeride geçme!
  return BT::NodeStatus::FAILURE;  // varsayılan: tek yönlü
}

}  // namespace robotaxi_bt

