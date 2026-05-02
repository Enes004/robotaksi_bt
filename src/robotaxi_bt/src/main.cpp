// BT Test — InitMissionAction'ı dosyadan çalıştırıp test eder
//
// Kullanım:
//   ros2 run robotaxi_bt bt_test
//   ros2 run robotaxi_bt bt_test /tam/yol/waypoints.txt

#include "robotaxi_bt/init_mission_action.hpp"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "rclcpp/rclcpp.hpp"
#include <iostream>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  // Komut satırından dosya yolu (opsiyonel)
  std::string wp_file = "waypoints.txt";
  if (argc > 1) {
    wp_file = argv[1];
  }

  std::cout << "\n=== InitMissionAction Test ===" << std::endl;
  std::cout << "Waypoints dosyasi: " << wp_file << std::endl;
  std::cout << std::endl;

  // BT Factory — node'ları kaydet
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<robotaxi_bt::InitMissionAction>("InitMissionAction");

  // Basit bir BT XML — sadece InitMissionAction'ı çalıştırır
  std::string xml = R"(
    <root main_tree_to_execute="TestTree">
      <BehaviorTree ID="TestTree">
        <Action ID="InitMissionAction"
                waypoints_file=")" + wp_file + R"("
                goals="{goals}"
                current_goal_type="{current_goal_type}"
                passenger_served="{passenger_served}"/>
      </BehaviorTree>
    </root>
  )";

  auto tree = factory.createTreeFromText(xml);
  BT::NodeStatus result = tree.tickRoot();

  // Sonuç
  if (result == BT::NodeStatus::SUCCESS) {
    std::cout << "\n✅ InitMissionAction BASARILI!" << std::endl;

    // Blackboard'dan goals sayısını oku
    auto bb = tree.rootBlackboard();
    auto goals = bb->get<std::vector<geometry_msgs::msg::PoseStamped>>("goals");
    auto goal_type = bb->get<std::string>("current_goal_type");

    std::cout << "   Yuklenen waypoint sayisi: " << goals.size() << std::endl;
    std::cout << "   Ilk goal_type: " << goal_type << std::endl;

    // Her waypoint'i göster
    for (size_t i = 0; i < goals.size(); i++) {
      auto& g = goals[i];
      std::cout << "   [" << i << "] x=" << g.pose.position.x
                << " y=" << g.pose.position.y
                << " qz=" << g.pose.orientation.z
                << " qw=" << g.pose.orientation.w << std::endl;
    }
  } else {
    std::cout << "\n❌ InitMissionAction BASARISIZ!" << std::endl;
    std::cout << "   waypoints.txt dosyasi bulunamadi veya bozuk" << std::endl;
    std::cout << "   Cozum: python3 geojson_parser.py gorev.geojson --waypoints" << std::endl;
  }

  std::cout << std::endl;
  rclcpp::shutdown();
  return (result == BT::NodeStatus::SUCCESS) ? 0 : 1;
}