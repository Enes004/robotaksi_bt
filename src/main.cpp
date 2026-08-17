// ============================================================================
// BT Runner — Segment BT'yi sürekli döngüde çalıştırır
//
// Kullanım:
//   ros2 run robotaksi_bt bt_test
//   ros2 run robotaksi_bt bt_test /tam/yol/segment_bt.xml
//
// Döngü: 20 Hz — her iterasyonda:
//   1. rclcpp::spin_some() → ROS callback'lerini işle
//   2. tree.tickRoot()     → BT ağacını tick'le
//   RUNNING dışında sonuç gelirse döngüden çık.
//
// BT.CPP v3 uyumlu.
// ============================================================================

#include "robotaksi_bt/register_all_nodes.hpp"
#include "robotaksi_bt/bt_node_base.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "rclcpp/rclcpp.hpp"
#include <iostream>
#include <filesystem>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  // ROS2 node oluştur — BT node'ları bu shared_ptr üzerinden
  // publisher/subscriber oluşturabilir (Blackboard'a eklenir)
  auto ros_node = rclcpp::Node::make_shared("bt_test_node");

  // ── ROS Parametreleri ──
  // Blackboard'a yazılacak değerler. Komut satırından veya launch dosyasından:
  //   --ros-args -p tour:=2 -p mission_json:=/path/to/gorev.geojson
  ros_node->declare_parameter<int>("tour", 1);
  ros_node->declare_parameter<std::string>("mission_json", "");

  // Odometry provider'ı başlat — tüm BT node'ları erişebilir
  robotaksi_bt::OdometryProvider::instance().init(ros_node);

  std::cout << "\n=== Segment BT Runner ===" << std::endl;

  // BT Factory — tüm node'ları kaydet
  BT::BehaviorTreeFactory factory;

  // Yeni modüler node kayıtları
  robotaksi_bt::registerAllNodes(factory);

  // Komut satırından XML dosya yolu (opsiyonel)
  std::string xml_file;
  if (argc > 1) {
    xml_file = argv[1];
  }

  if (!xml_file.empty() && std::filesystem::exists(xml_file)) {
    std::cout << "XML dosyasi: " << xml_file << std::endl;

    try {
      auto tree = factory.createTreeFromFile(xml_file);

      // Kök blackboard'u global singleton'a kaydet — SubTree sınırlarını
      // aşan node'lar (autoremap koptuğunda) buradan erişir.
      robotaksi_bt::globalRootBlackboard() = tree.rootBlackboard();

      // ROS node'u Blackboard'a ekle — tüm BT node'ları erişebilir
      tree.rootBlackboard()->set("ros_node", ros_node);

      // Misyon parametrelerini Blackboard'a yaz — XML'deki {tour} ve
      // {geojson_file} anahtarları bu değerleri okur.
      int tour_val = ros_node->get_parameter("tour").as_int();
      std::string mission_json_val = ros_node->get_parameter("mission_json").as_string();
      tree.rootBlackboard()->set("tour", tour_val);
      tree.rootBlackboard()->set("geojson_file", mission_json_val);

      RCLCPP_INFO(ros_node->get_logger(),
        "Tour: %d, Mission JSON: %s",
        tour_val,
        mission_json_val.empty() ? "(belirtilmedi)" : mission_json_val.c_str());

      // ── mission_json boşsa başlamadan önce belirgin uyarı bas ──
      if (mission_json_val.empty()) {
        RCLCPP_WARN(ros_node->get_logger(),
          "mission_json parametresi verilmedi — LoadMission sabit test rotasını"
          " kullanacak. Gerçek görev için: --ros-args -p mission_json:=/yol/gorev.geojson");
      }

      std::cout << "\n✅ XML başarıyla yüklendi!" << std::endl;
      std::cout << "   Kayıtlı node sayısı: " << factory.manifests().size() << std::endl;

      // ── 20 Hz sürekli döngü ──
      rclcpp::Rate rate(20.0);  // 20 Hz = 50ms per tick
      BT::NodeStatus result = BT::NodeStatus::RUNNING;

      std::cout << "\n--- BT döngüsü başlıyor (20 Hz) ---" << std::endl;

      while (rclcpp::ok() && result == BT::NodeStatus::RUNNING) {
        // 1. ROS callback'lerini işle (subscriber mesajları, timer'lar vb.)
        rclcpp::spin_some(ros_node);

        // 2. BT ağacını tick'le
        result = tree.tickRoot();

        // 3. Rate'e uy (20 Hz)
        rate.sleep();
      }

      // ── Fallback rota kullanıldıysa belirgin uyarı ──────────────────────
      {
        bool used_fallback = false;
        tree.rootBlackboard()->get("using_fallback_route", used_fallback);
        if (used_fallback) {
          std::cerr << "\n";
          std::cerr << "⚠️  ⚠️  ⚠️  UYARI: GERÇEK GÖREV ROTASI BULUNAMADI  ⚠️  ⚠️  ⚠️\n";
          std::cerr << "    Sistem SABİT TEST ROTASINA düştü, gerçek mission_json\n";
          std::cerr << "    rotası KULLANILMADI. Harita/koordinat eşleşmesini kontrol edin.\n";
          std::cerr << "\n";
        }
      }
      // ─────────────────────────────────────────────────────────────────────

      // ── Segment özet raporu ──────────────────────────────────────────────
      robotaksi_bt::RunStats::instance().print();
      // ────────────────────────────────────────────────────────────────────

      std::cout << "\n--- BT döngüsü bitti ---" << std::endl;
      std::cout << "Son sonuç: " << BT::toStr(result) << std::endl;

    } catch (const std::exception& e) {
      std::cerr << "\n❌ XML yükleme hatası: " << e.what() << std::endl;
      rclcpp::shutdown();
      return 1;
    }
  } else {
    std::cout << "\nXML dosyasi belirtilmedi. Node kayıt testi yapılıyor..." << std::endl;
    std::cout << "\n✅ Kayıtlı node'lar:" << std::endl;

    for (const auto& [name, manifest] : factory.manifests()) {
      std::cout << "   [" << BT::toStr(manifest.type) << "] " << name;
      if (!manifest.ports.empty()) {
        std::cout << " (";
        bool first = true;
        for (const auto& [port_name, port_info] : manifest.ports) {
          if (!first) std::cout << ", ";
          std::cout << port_name;
          first = false;
        }
        std::cout << ")";
      }
      std::cout << std::endl;
    }

    std::cout << "\nToplam: " << factory.manifests().size() << " node kayıtlı." << std::endl;
    std::cout << "\nKullanım: ros2 run robotaksi_bt bt_test <segment_bt.xml>" << std::endl;
  }

  std::cout << std::endl;
  rclcpp::shutdown();
  return 0;
}