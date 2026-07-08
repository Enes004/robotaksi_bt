// InitMissionAction — GeoJSON'dan gelen waypoints.txt dosyasını okur
//
// NOT: Bu eski node korunmuştur. Yeni segment_bt.xml için
//      LoadMission node'u kullanılır. Bu node sadece geriye dönük
//      uyumluluk ve test amaçlıdır.
//
// BT.CPP v3 uyumlu.

#ifndef INIT_MISSION_ACTION_HPP
#define INIT_MISSION_ACTION_HPP

#include "behaviortree_cpp_v3/action_node.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <string>
#include <vector>
#include <fstream>
#include <sstream>

namespace robotaxi_bt {

class InitMissionAction : public BT::SyncActionNode {
public:
  InitMissionAction(const std::string &name,
                    const BT::NodeConfiguration &config)
      : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts() {
    return {
        BT::InputPort<std::string>("waypoints_file", "waypoints.txt",
          "geojson_parser.py --waypoints ile uretilen dosya yolu"),
        BT::OutputPort<std::vector<geometry_msgs::msg::PoseStamped>>("goals"),
        BT::OutputPort<std::string>("current_goal_type"),
        BT::OutputPort<std::string>("passenger_served")};
  }

  BT::NodeStatus tick() override;
};

}  // namespace robotaxi_bt

#endif  // INIT_MISSION_ACTION_HPP