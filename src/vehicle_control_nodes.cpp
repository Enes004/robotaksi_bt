// ============================================================================
// vehicle_control_nodes.cpp — Araç Kontrol Node İmplementasyonları
//
// ✅ Tam implemente — ROS2 publisher ile gerçek topic'lere yayın yapar.
// ============================================================================

#include "robotaksi_bt/vehicle_control_nodes.hpp"

namespace robotaksi_bt {

// ═══════════════════════════════════════════════════════════════
// SetMaxSpeed::tick()
//
// 1. "speed" portunu oku (double, m/s cinsinden)
// 2. Blackboard'a "current_max_speed" olarak kaydet
// 3. /vehicle/max_speed topic'ine Float64 olarak publish et
//
// NEDEN HEM BLACKBOARD HEM TOPIC?
//   Blackboard: BT içindeki diğer node'lar okuyabilir (ör: FollowLaneSegment)
//   Topic: BT dışındaki ROS node'ları okuyabilir (ör: controller_server)
// ═══════════════════════════════════════════════════════════════
BT::NodeStatus SetMaxSpeed::tick()
{
  double speed = 0.0;

  // Önce normal port sistemiyle dene (aynı SubTree seviyesinde çalışır)
  bool found = getInput("speed", speed).operator bool();

  if (!found) {
    // getInput başarısız → autoremap SubTree sınırında kopmuş.
    // TODO: XML'deki hangi hız değişkeninin aktif olduğunu doğrudan bilmiyoruz,
    //       sırayla deniyoruz — daha temiz çözüm SetMaxSpeed'in XML'de
    //       kullandığı gerçek port adını okumaktan geçer.
    const char* fallback_keys[] = {
      "cruise_speed", "intersection_speed", "roundabout_speed",
      "tunnel_speed", "parking_speed"
    };
    for (const char* key : fallback_keys) {
      std::string str_val;
      if (globalRootBlackboard()->get(key, str_val)) {
        try {
          speed = std::stod(str_val);
          found = true;
          break;
        } catch (const std::exception&) {
          continue;  // bu anahtar sayıya çevrilemedi, sıradakini dene
        }
      }
    }
  }

  if (!found) {
    RCLCPP_ERROR(btLogger(), "SetMaxSpeed: 'speed' portu okunamadı!");
    return BT::NodeStatus::FAILURE;
  }

  // Blackboard'a kaydet — diğer BT node'ları okuyabilsin
  config().blackboard->set("current_max_speed", speed);

  // Lazy-init: publisher'ı ilk tick'te oluştur, sonrakilerde tekrar kullan
  if (!pub_) {
    auto ros = getRosNode(config());
    if (ros) {
      pub_ = ros->create_publisher<std_msgs::msg::Float64>("/vehicle/max_speed", 10);
    }
  }
  if (pub_) {
    std_msgs::msg::Float64 msg;
    msg.data = speed;
    pub_->publish(msg);
  }

  RCLCPP_INFO(btLogger(), "SetMaxSpeed: %.2f m/s ayarlandı", speed);
  return BT::NodeStatus::SUCCESS;
}

// ═══════════════════════════════════════════════════════════════
// StopVehicle — StatefulActionNode implementasyonu
//
// publishZeroVel(): publisher lazy-init + sıfır Twist yayını
// onStart():   hold_sec oku, zamanlayıcı başlat, bir kez yayınla
//              hold_sec<=0 → anında SUCCESS (eski SyncAction gibi)
//              hold_sec>0  → RUNNING (sürekli yayın moduna geç)
// onRunning(): her tick'te tekrar yayınla
//              süre dolduysa SUCCESS, dolmadıysa RUNNING
// onHalted():  log
// ═══════════════════════════════════════════════════════════════

void StopVehicle::publishZeroVel()
{
  // Lazy-init: publisher ilk çağrıda oluşturulur, sonra tekrar kullanılır
  if (!pub_) {
    auto ros = getRosNode(config());
    if (ros) {
      pub_ = ros->create_publisher<geometry_msgs::msg::Twist>(kCmdVelTopic, 10);
    }
  }
  if (pub_) {
    geometry_msgs::msg::Twist zero_vel;
    // Twist() varsayılan: tüm alanlar 0.0 — tam durma
    pub_->publish(zero_vel);
  }
}

BT::NodeStatus StopVehicle::onStart()
{
  // hold_sec portunu oku (varsayılan 0.0)
  hold_sec_ = 0.0;
  getInput("hold_sec", hold_sec_);

  // Zamanlayıcıyı her seferinde sıfırla
  start_time_ = std::chrono::steady_clock::now();

  // Her iki modda da ilk dur komutunu hemen gönder
  publishZeroVel();

  if (hold_sec_ <= 0.0) {
    // ANLÂTIK MOD — eski SyncAction davranışı
    // Sequence bir sonraki node'a (CheckStopAccuracy, Dwell...) geçebilir
    RCLCPP_INFO(btLogger(), "StopVehicle: %s sıfırlandı → SUCCESS (anlık)", kCmdVelTopic);
    return BT::NodeStatus::SUCCESS;
  }

  // TUTMA MODU — hold_sec boyunca sürekli yayın yapılacak
  RCLCPP_INFO(btLogger(), "StopVehicle: %s sıfırlandı → RUNNING (%.1f sn tutma)",
              kCmdVelTopic, hold_sec_);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus StopVehicle::onRunning()
{
  // Sürekli sıfır Twist yayınla (~20 Hz BT tick hızıyla)
  publishZeroVel();

  double elapsed = std::chrono::duration<double>(
    std::chrono::steady_clock::now() - start_time_).count();

  if (elapsed >= hold_sec_) {
    RCLCPP_INFO(btLogger(), "StopVehicle: %.1f sn tutma tamamlandı → SUCCESS", elapsed);
    return BT::NodeStatus::SUCCESS;
  }

  return BT::NodeStatus::RUNNING;
}

void StopVehicle::onHalted()
{
  RCLCPP_WARN(btLogger(), "StopVehicle: HALTED (%.1f / %.1f sn)",
    std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time_).count(),
    hold_sec_);
}

// ═══════════════════════════════════════════════════════════════
// TurnHeadlights::tick()
//
// Far kontrolü: "on" → true, "off" → false
// /vehicle/headlights topic'ine std_msgs::msg::Bool publish eder.
// Blackboard'a da kaydeder — diğer node'lar far durumunu sorgulayabilir.
// ═══════════════════════════════════════════════════════════════
BT::NodeStatus TurnHeadlights::tick()
{
  std::string state;
  if (!getInput("state", state)) {
    RCLCPP_ERROR(btLogger(), "TurnHeadlights: 'state' portu okunamadı!");
    return BT::NodeStatus::FAILURE;
  }

  bool lights_on = (state == "on" || state == "ON");

  // Blackboard — diğer node'lar okuyabilir
  config().blackboard->set("headlights_on", lights_on);

  // Lazy-init: publisher'ı ilk tick'te oluştur
  if (!pub_) {
    auto ros = getRosNode(config());
    if (ros) {
      pub_ = ros->create_publisher<std_msgs::msg::Bool>("/vehicle/headlights", 10);
    }
  }
  if (pub_) {
    std_msgs::msg::Bool msg;
    msg.data = lights_on;
    pub_->publish(msg);
  }

  RCLCPP_INFO(btLogger(), "TurnHeadlights: farlar %s", lights_on ? "AÇILDI" : "KAPANDI");
  return BT::NodeStatus::SUCCESS;
}

// ═══════════════════════════════════════════════════════════════
// Dwell — StatefulActionNode İmplementasyonu
//
// onStart(): İlk tick — zamanlayıcı başlar, RUNNING döner
// onRunning(): Sonraki tick'ler — süre doldu mu kontrol eder
// onHalted(): İptal — temizlik yapar
//
// min_sec ve max_sec arasında ortalamasını alarak bekler.
// İleride: random(min, max) ile daha gerçekçi yapılabilir.
// ═══════════════════════════════════════════════════════════════
BT::NodeStatus Dwell::onStart()
{
  double min_sec = 15.0, max_sec = 20.0;
  getInput("min_sec", min_sec);
  getInput("max_sec", max_sec);

  // Orta değeri al (basit yaklaşım)
  wait_duration_sec_ = (min_sec + max_sec) / 2.0;

  // Zamanlayıcıyı başlat — steady_clock monotoniktir (sistem saati değişse de etkilenmez)
  start_time_ = std::chrono::steady_clock::now();

  RCLCPP_INFO(btLogger(), "Dwell: %.0f sn bekleme başladı (min=%.0f, max=%.0f)",
              wait_duration_sec_, min_sec, max_sec);

  // RUNNING = "henüz bitmedi, sonraki tick'te tekrar kontrol et"
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus Dwell::onRunning()
{
  auto elapsed = std::chrono::steady_clock::now() - start_time_;
  double sec = std::chrono::duration<double>(elapsed).count();

  if (sec >= wait_duration_sec_) {
    RCLCPP_INFO(btLogger(), "Dwell: %.1f sn bekleme tamamlandı", sec);
    return BT::NodeStatus::SUCCESS;  // İş bitti
  }

  // Henüz süre dolmadı — RUNNING dön, sonraki tick'te tekrar kontrol
  return BT::NodeStatus::RUNNING;
}

void Dwell::onHalted()
{
  // onHalted: ağaç durdurulduğunda (ör: güvenlik refleksi) çağrılır
  // Temizlik yap — bu durumda sadece log basmak yeterli
  RCLCPP_WARN(btLogger(), "Dwell: HALTED — bekleme kesildi");
}

}  // namespace robotaksi_bt
