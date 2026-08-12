// ============================================================================
// traffic_logic_nodes.cpp — Trafik Mantık Node İmplementasyonları
// ============================================================================

#include "robotaksi_bt/traffic_logic_nodes.hpp"

namespace robotaksi_bt {

// ═══════════════════════════════════════════════════════════════
// IsLightRed::tick()
//
// Blackboard'daki light_color değerini alır ve kırmızı mı kontrol eder.
// "red", "RED", "kirmizi" → SUCCESS (kırmızı ışık)
// Diğer tüm değerler → FAILURE (kırmızı değil)
//
// HandleTrafficLight ağacında kullanılır:
//   <Inverter><IsLightRed color="{light_color}" /></Inverter>
//   Kırmızı değilse → Inverter → SUCCESS → devam et
//   Kırmızı ise → dur ve bekle
// ═══════════════════════════════════════════════════════════════
BT::NodeStatus IsLightRed::tick() {
  std::string color;
  getInput("color", color);

  bool is_red = (color == "red" || color == "RED" || color == "kirmizi");

  RCLCPP_DEBUG(btLogger(), "IsLightRed: color='%s' → %s", color.c_str(),
               is_red ? "KIRMIZI" : "DEĞİL");

  return is_red ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

// ═══════════════════════════════════════════════════════════════
// IsRoadSignType::tick()
//
// GlobalRoadSignAhead'den gelen levha tipini karşılaştırır.
// HandleRoadSigns ağacında switch/case deseniyle kullanılır:
//
//   <IsRoadSignType sign_type="{detected_road_sign}" expected="LANE_MERGE" />
//   <IsRoadSignType sign_type="{detected_road_sign}" expected="NO_ENTRY" />
//
// Geçerli tipler: LANE_MERGE, PEDESTRIAN_CROSS, PASS_DIRECTION, NO_ENTRY
// ═══════════════════════════════════════════════════════════════
BT::NodeStatus IsRoadSignType::tick() {
  std::string sign_type, expected;
  getInput("sign_type", sign_type);
  getInput("expected", expected);

  bool match = (sign_type == expected);

  RCLCPP_DEBUG(btLogger(), "IsRoadSignType: '%s' == '%s' → %s",
               sign_type.c_str(), expected.c_str(), match ? "EVET" : "HAYIR");

  return match ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

// ═══════════════════════════════════════════════════════════════
// StopAndProceed::tick()
//
// DUR (TT-2) tabelası görüldüğünde çalışır.
// Latch mekanizması: aynı segment içinde tekrar tetiklenmez.
//
// AKIŞ:
//   1. handled_stop_sign bayrağını kontrol et
//   2. Zaten true ise → SUCCESS (tekrar durma)
//   3. false ise → true yap, dur komutu ver
//
// ClearHandledFlags sonraki segmentte bayrağı sıfırlar.
// ═══════════════════════════════════════════════════════════════
BT::NodeStatus StopAndProceed::tick() {
  auto bb = config().blackboard;

  // Latch kontrolü
  bool handled = false;
  bb->get("handled_stop_sign", handled);
  if (handled) {
    RCLCPP_DEBUG(btLogger(), "StopAndProceed: zaten işlendi, atlanıyor");
    return BT::NodeStatus::SUCCESS;
  }

  // Bayrağı set et
  bb->set("handled_stop_sign", true);

  RCLCPP_INFO(btLogger(),
              "StopAndProceed: DUR tabelası — duruldu, devam ediliyor");
  return BT::NodeStatus::SUCCESS;
}

// ═══════════════════════════════════════════════════════════════
// LogUnknownSign::tick()
//
// HandleRoadSigns Fallback'inin son çocuğu.
// Hiçbir bilinen tipe uymayan levha burada loglanır.
// ForceSuccess ile sarılmadığı halde Fallback'in son çocuğu
// olduğu için SUCCESS dönmesi Fallback'i tamamlar.
// ═══════════════════════════════════════════════════════════════
BT::NodeStatus LogUnknownSign::tick() {
  std::string sign_type;
  getInput("sign_type", sign_type);

  RCLCPP_WARN(btLogger(), "LogUnknownSign: tanımsız levha = '%s'",
              sign_type.c_str());
  return BT::NodeStatus::SUCCESS;
}

} // namespace robotaksi_bt

