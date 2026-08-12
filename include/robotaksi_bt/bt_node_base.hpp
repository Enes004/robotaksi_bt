// ============================================================================
// bt_node_base.hpp — ROS2 Node Paylaşım Altyapısı
//
// Tüm BT node'larının ROS2 publisher/subscriber oluşturabilmesi için
// ortak rclcpp::Node::SharedPtr erişimi sağlar.
//
// KULLANIM:
//   main.cpp'de Blackboard'a ROS node'u ekle:
//     blackboard->set("ros_node", ros_node_ptr);
//
//   Herhangi bir BT node'unda:
//     auto ros = getRosNode();         // rclcpp::Node::SharedPtr
//     auto pub = ros->create_publisher<...>("/topic", 10);
//
// BT.CPP v3 uyumlu.
// ============================================================================
#ifndef BT_NODE_BASE_HPP
#define BT_NODE_BASE_HPP

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/condition_node.h"
#include <string>

namespace robotaksi_bt {

// ─── Ortak logger fonksiyonu ───
// Tüm node'lar bu fonksiyonla log basar.
// Kullanım: RCLCPP_INFO(btLogger(), "mesaj");
inline rclcpp::Logger btLogger()
{
  return rclcpp::get_logger("segment_bt");
}

// ─── ROS Node'a Erişim ───
// Blackboard'dan "ros_node" anahtarıyla paylaşılan rclcpp::Node'u alır.
// main.cpp'de blackboard->set("ros_node", node) ile set edilmeli.
//
// Kullanım (herhangi bir BT node içinde):
//   auto ros = getRosNode(config());
//   auto pub = ros->create_publisher<std_msgs::msg::Bool>("/topic", 10);
inline rclcpp::Node::SharedPtr getRosNode(const BT::NodeConfiguration& config)
{
  rclcpp::Node::SharedPtr node;
  config.blackboard->get("ros_node", node);
  if (!node) {
    RCLCPP_ERROR(btLogger(),
      "Blackboard'da 'ros_node' bulunamadı! main.cpp'de set edilmeli.");
  }
  return node;
}

}  // namespace robotaksi_bt

#endif  // BT_NODE_BASE_HPP
