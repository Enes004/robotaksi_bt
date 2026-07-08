// BT Test — Segment BT node'larını test eder
//
// Kullanım:
//   ros2 run robotaxi_bt bt_test
//   ros2 run robotaxi_bt bt_test /tam/yol/segment_bt.xml
//
// BT.CPP v3 uyumlu.

#include "robotaxi_bt/segment_bt_nodes.hpp"
#include "robotaxi_bt/init_mission_action.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "rclcpp/rclcpp.hpp"
#include <iostream>
#include <filesystem>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::cout << "\n=== Segment BT Test ===" << std::endl;

  // BT Factory — tüm node'ları kaydet
  BT::BehaviorTreeFactory factory;

  // Segment BT node'ları (yeni)
  robotaxi_bt::registerSegmentBTNodes(factory);

  // Eski InitMissionAction (geriye dönük uyumluluk)
  factory.registerNodeType<robotaxi_bt::InitMissionAction>("InitMissionAction");

  // Komut satırından XML dosya yolu (opsiyonel)
  std::string xml_file;
  if (argc > 1) {
    xml_file = argv[1];
  }

  if (!xml_file.empty() && std::filesystem::exists(xml_file)) {
    // Dosyadan yükle
    std::cout << "XML dosyasi: " << xml_file << std::endl;

    try {
      auto tree = factory.createTreeFromFile(xml_file);
      std::cout << "\n✅ XML başarıyla yüklendi!" << std::endl;
      std::cout << "   Kayıtlı node sayısı: " << factory.manifests().size() << std::endl;

      // İlk tick
      std::cout << "\n--- İlk tick ---" << std::endl;
      BT::NodeStatus result = tree.tickRoot();
      std::cout << "Sonuç: " << BT::toStr(result) << std::endl;

    } catch (const std::exception& e) {
      std::cerr << "\n❌ XML yükleme hatası: " << e.what() << std::endl;
      std::cerr << "   Olası sebepler:" << std::endl;
      std::cerr << "   - XML formatı hatalı" << std::endl;
      std::cerr << "   - Kayıtlı olmayan node kullanılmış" << std::endl;
      rclcpp::shutdown();
      return 1;
    }
  } else {
    // Basit test: sadece node kayıtlarını doğrula
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
    std::cout << "\nKullanım: ros2 run robotaxi_bt bt_test <segment_bt.xml>" << std::endl;
  }

  std::cout << std::endl;
  rclcpp::shutdown();
  return 0;
}