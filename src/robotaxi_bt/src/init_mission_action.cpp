// InitMissionAction — GeoJSON'dan gelen waypoints.txt dosyasını okur
//
// AKIŞ:
//   1. Yarışma günü hakemler GeoJSON verir
//   2. Python script çalıştırılır:
//      python3 geojson_parser.py gorev.geojson --waypoints
//      → waypoints.txt dosyası üretir
//   3. Bu C++ kodu o dosyayı okur
//   4. goals listesine ekler
//   5. BT blackboard'a yazar
//   6. Robot gider!

// BT.CPP v4 uyumlu — header v4 include'ları kullanır
#include "robotaxi_bt/init_mission_action.hpp"
#include "rclcpp/rclcpp.hpp"

namespace robotaxi_bt {

BT::NodeStatus InitMissionAction::tick()
{
  // ═══════════════════════════════════════════════════════════
  // ADIM 1: Dosya yolunu al
  // ═══════════════════════════════════════════════════════════
  // BT XML'den "waypoints_file" portunu oku
  // Eğer belirtilmemişse varsayılan: "waypoints.txt"
  std::string filepath;
  if (!getInput("waypoints_file", filepath)) {
    filepath = "waypoints.txt";
  }

  RCLCPP_INFO(rclcpp::get_logger("InitMissionAction"),
    "Waypoints dosyasi okunuyor: %s", filepath.c_str());

  // ═══════════════════════════════════════════════════════════
  // ADIM 2: Dosyayı aç
  // ═══════════════════════════════════════════════════════════
  std::ifstream file(filepath);

  if (!file.is_open()) {
    // Dosya açılamadıysa HATA ver
    RCLCPP_ERROR(rclcpp::get_logger("InitMissionAction"),
      "HATA: Waypoints dosyasi acilamadi: %s", filepath.c_str());
    RCLCPP_ERROR(rclcpp::get_logger("InitMissionAction"),
      "Cozum: python3 geojson_parser.py gorev.geojson --waypoints");
    return BT::NodeStatus::FAILURE;
  }

  // ═══════════════════════════════════════════════════════════
  // ADIM 3: Dosyayı satır satır oku
  // ═══════════════════════════════════════════════════════════
  //
  // waypoints.txt formatı (Python script üretir):
  //   # yorum satırları
  //   isim tip x y qz qw
  //
  // Örnek:
  //   start start 0.0000 0.0000 -0.8535 0.5211
  //   gorev_1 gorev -32.3162 -62.9256 0.5253 0.8509
  //   park_giris park_giris 18.2519 -21.0333 0.6419 0.7668
  //
  std::vector<geometry_msgs::msg::PoseStamped> goals_list;

  // İlk görev tipini kaydetmek için
  std::string first_goal_type = "";

  std::string line;              // her satırı buraya okuyacağız
  while (std::getline(file, line))
  {
    // Boş satırları ve yorum satırlarını (#) atla
    if (line.empty() || line[0] == '#') {
      continue;
    }

    // Satırı parçalara ayır: isim tip x y qz qw
    std::istringstream iss(line);
    std::string name, type;
    double x, y, qz, qw;

    // >> operatörü boşluklara göre ayırır
    // "gorev_1 gorev -32.3162 -62.9256 0.5253 0.8509"
    //  ↓        ↓      ↓        ↓       ↓      ↓
    //  name    type     x        y       qz     qw
    if (!(iss >> name >> type >> x >> y >> qz >> qw)) {
      RCLCPP_WARN(rclcpp::get_logger("InitMissionAction"),
        "Hatali satir atlandi: %s", line.c_str());
      continue;
    }

    // ─────────────────────────────────────────────
    // PoseStamped mesajı oluştur
    // ─────────────────────────────────────────────
    // PoseStamped = konum (x,y,z) + yön (quaternion) + zaman + frame
    geometry_msgs::msg::PoseStamped goal;
    goal.header.frame_id = "map";                  // koordinat çerçevesi
    goal.header.stamp = rclcpp::Clock().now();      // şimdiki zaman
    goal.pose.position.x = x;                       // doğu-batı (metre)
    goal.pose.position.y = y;                       // kuzey-güney (metre)
    goal.pose.position.z = 0.0;                     // yükseklik (düz zemin)
    goal.pose.orientation.x = 0.0;                  // quaternion x (0 - sadece yaw)
    goal.pose.orientation.y = 0.0;                  // quaternion y (0 - sadece yaw)
    goal.pose.orientation.z = qz;                   // quaternion z (yaw bileşeni)
    goal.pose.orientation.w = qw;                   // quaternion w (yaw bileşeni)

    goals_list.push_back(goal);

    // İlk görev tipi (start hariç) = current_goal_type başlangıç değeri
    if (first_goal_type.empty() && type != "start") {
      first_goal_type = type;   // "gorev" veya "park_giris"
    }

    RCLCPP_INFO(rclcpp::get_logger("InitMissionAction"),
      "  [%zu] %s (tip: %s) -> x=%.2f, y=%.2f",
      goals_list.size(), name.c_str(), type.c_str(), x, y);
  }

  file.close();

  // ═══════════════════════════════════════════════════════════
  // ADIM 4: Kontrol — en az 1 waypoint okuduk mu?
  // ═══════════════════════════════════════════════════════════
  if (goals_list.empty()) {
    RCLCPP_ERROR(rclcpp::get_logger("InitMissionAction"),
      "HATA: Dosyada gecerli waypoint bulunamadi!");
    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_INFO(rclcpp::get_logger("InitMissionAction"),
    "Toplam %zu waypoint yuklendi. Ilk goal_type: %s",
    goals_list.size(), first_goal_type.c_str());

  // ═══════════════════════════════════════════════════════════
  // ADIM 5: Blackboard'a yaz
  // ═══════════════════════════════════════════════════════════
  // setOutput → BT blackboard'a veri yazar
  // Diğer node'lar bu verileri {goals}, {current_goal_type} vs. ile okur

  setOutput("goals", goals_list);                        // waypoint listesi
  setOutput("current_goal_type", first_goal_type);       // "gorev" veya "park_giris"
  setOutput("passenger_served", std::string(""));        // başlangıçta boş

  // SUCCESS = her şey yolunda, BT devam edebilir
  return BT::NodeStatus::SUCCESS;
}

}  // namespace robotaxi_bt