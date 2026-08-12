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

  // Port okuma — başarısızsa FAILURE dön
  if (!getInput("speed", speed)) {
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
// StopVehicle::tick()
//
// /cmd_vel topic'ine sıfır Twist mesajı yayınlar.
// geometry_msgs::msg::Twist varsayılan olarak tüm alanları 0.0'dır.
//
// NOT: Bu node "dur komutu gönder" demektir.
//      Aracın GERÇEKTEN durduğunu doğrulamak CheckStopAccuracy'nin işidir.
//      Fiziksel gecikme olabilir (fren mesafesi, ivme).
// ═══════════════════════════════════════════════════════════════
BT::NodeStatus StopVehicle::tick()
{
  // Lazy-init: publisher'ı ilk tick'te oluştur
  if (!pub_) {
    auto ros = getRosNode(config());
    if (ros) {
      pub_ = ros->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    }
  }
  if (pub_) {
    geometry_msgs::msg::Twist zero_vel;
    // Twist() varsayılan olarak tüm alanlar 0.0 — tam durma
    pub_->publish(zero_vel);
  }

  RCLCPP_INFO(btLogger(), "StopVehicle: /cmd_vel sıfırlandı");
  return BT::NodeStatus::SUCCESS;
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
