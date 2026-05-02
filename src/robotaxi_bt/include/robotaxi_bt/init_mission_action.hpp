// 1-) HEADER GUARD
#ifndef INIT_MISSION_ACTION_HPP
#define INIT_MISSION_ACTION_HPP

// 2-) Libraries ( ROS2 message types and BT)
#include "behaviortree_cpp_v3/action_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <string>
#include <vector>
#include <fstream>    // dosya okumak için (waypoints.txt)
#include <sstream>    // satır parse etmek için

namespace robotaxi_bt {
// 3-) Inheritance
// Inheritance from BT::SyncActionNode
class InitMissionAction : public BT::SyncActionNode {

  // 4-) Constructor
  //  Parent class need name and config in constructor
public:
  InitMissionAction(const std::string &name,
                    const BT::NodeConfiguration &config)
      : BT::SyncActionNode(name, config) {}

  // 5-) PORTS
  // Define ports for this node
  static BT::PortsList providedPorts() {
    return {
        // GİRİŞ: waypoints.txt dosya yolu
        // Python script bunu üretir:
        //   python3 geojson_parser.py gorev.geojson --waypoints
        BT::InputPort<std::string>("waypoints_file", "waypoints.txt",
          "geojson_parser.py --waypoints ile uretilen dosya yolu"),

        // ÇIKIŞLAR: BT blackboard'a yazılacak veriler
        BT::OutputPort<std::vector<geometry_msgs::msg::PoseStamped>>("goals"),
        BT::OutputPort<std::string>("current_goal_type"),
        BT::OutputPort<std::string>("passenger_served")};
  }

  // 6-) Tick
  // Override empty tick method from parent class.
  // It will return Success , Failure or Running
  BT::NodeStatus tick() override;
};

} // namespace robotaxi_bt

#endif // INIT_MISSION_ACTION_HPP

/*
 YAPISI:

 1) header guard     — dosyanın iki kez include edilmesini önler
 2) libraries        — ihtiyacımız olan kütüphaneler
 3) Inheritance      — SyncActionNode'dan miras alma
 4) Constructor      — name ve config parametreleri
 5) Ports            — BT ile veri alışverişi:
                         GİRİŞ:  waypoints_file (dosya yolu)
                         ÇIKIŞ:  goals (koordinat listesi)
                                 current_goal_type (nokta tipi)
                                 passenger_served (servis durumu)
 6) tick()           — asıl iş burada yapılır (cpp dosyasında)
*/