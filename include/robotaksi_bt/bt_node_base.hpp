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
// globalRootBlackboard(): Kök (root) blackboard'a global erişim sağlar.
//   Çok katmanlı SubTree yapılarında _autoremap'in koptuğu durumlarda
//   bu fonksiyon üzerinden doğrudan kök blackboard'a yazılır/okunur.
//   main.cpp'de tree oluşturulduktan hemen sonra set edilmeli:
//     globalRootBlackboard() = tree.rootBlackboard();
//
// BT.CPP v3 uyumlu.
// ============================================================================
#ifndef BT_NODE_BASE_HPP
#define BT_NODE_BASE_HPP

#include "rclcpp/rclcpp.hpp"
#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/condition_node.h"
#include "robotaksi_bt/segment_graph.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/utils.h>
#include <tf2/LinearMath/Quaternion.h>
#include <string>
#include <mutex>
#include <cmath>
#include <map>
#include <iostream>
#include <cstdio>

namespace robotaksi_bt {

// ─── Topic Sabitleri ───
// BT tarafından publish edilen topic isimleri burada tanımlanır.
// twist_mux mimarisinde BT, /cmd_vel'e DEĞİL /cmd_vel_bt'ye yazar;
// twist_mux önceliğe göre gerçek /cmd_vel'i üretir.
constexpr const char* kCmdVelTopic = "/cmd_vel_bt";

// ─── Ortak logger fonksiyonu ───
// Tüm node'lar bu fonksiyonla log basar.
// Kullanım: RCLCPP_INFO(btLogger(), "mesaj");
inline rclcpp::Logger btLogger()
{
  return rclcpp::get_logger("segment_bt");
}

// ─── Global Kök Blackboard Erişimi ───
// Çok katmanlı SubTree yapılarında _autoremap zinciri koptuğunda,
// node'lar kendi config().blackboard'ları yerine bu global pointer
// üzerinden kök blackboard'a erişir.
//
// main.cpp'de tree oluşturulduktan sonra mutlaka set edilmeli:
//   globalRootBlackboard() = tree.rootBlackboard();
inline BT::Blackboard::Ptr& globalRootBlackboard()
{
  static BT::Blackboard::Ptr bb;
  return bb;
}

// ─── Paylaşılan SegmentGraph Singleton ───
// LoadMission ve ReplanRoute aynı graph nesnesini kullanır.
// loadFromGeoJSON sadece LoadMission'da, ilk tick'te çağrılır.
// Diğer node'lar (ReplanRoute) doğrudan planRoute/findNearestNode
// çağırabilir — grafın yüklü olduğu varsayılır.
inline SegmentGraph& globalSegmentGraph()
{
  static SegmentGraph graph;
  return graph;
}

// ─── Çalışma Zamanı İstatistikleri (Singleton) ───
// BT bittikten sonra tek satırlık özet raporu basar.
// Kullanım:
//   Segment eşleştiğinde:  RunStats::instance().recordSegmentType(type);
//   BT bittikten sonra:    RunStats::instance().print();
class RunStats {
public:
  static RunStats& instance() {
    static RunStats inst;
    return inst;
  }

  /// Her eşleşen segment tipini kaydet.
  void recordSegmentType(const std::string& type) {
    type_counts_[type]++;
    if (start_time_ == std::chrono::steady_clock::time_point{}) {
      start_time_ = std::chrono::steady_clock::now();
    }
  }

  /// BT bittikten sonra özet raporu stdout'a bas.
  void print() const {
    int total = 0;
    for (const auto& [t, c] : type_counts_) total += c;

    double elapsed_sec = 0.0;
    if (start_time_ != std::chrono::steady_clock::time_point{}) {
      elapsed_sec = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - start_time_).count();
    }

    // ── kutu çizgisi yardımcıları ──────────────────────────────
    constexpr int kWidth = 38;  // iç genişlik (║ ile ║ arası)
    auto line = [&](const std::string& label, int val) {
      // "║ LANE_FOLLOW:          20 ║" formatı
      std::string left  = " " + label + ":";
      std::string right = std::to_string(val);
      int padding = kWidth - static_cast<int>(left.size()) -
                    static_cast<int>(right.size()) - 1;
      if (padding < 1) padding = 1;
      std::cout << "\u2551" << left << std::string(padding, ' ') << right << " \u2551\n";
    };

    std::cout << "\n";
    std::cout << "\u2554" << std::string(kWidth, '\u2550') << "\u2557\n";
    std::cout << "\u2551" << "      SEGMENT \u00d6ZET RAPORU"
              << std::string(kWidth - 26, ' ') << "\u2551\n";
    std::cout << "\u2560" << std::string(kWidth, '\u2550') << "\u2563\n";

    if (type_counts_.empty()) {
      std::string msg = " (segment kaydedilmedi)";
      int pad = kWidth - static_cast<int>(msg.size());
      std::cout << "\u2551" << msg << std::string(std::max(pad, 0), ' ') << "\u2551\n";
    } else {
      for (const auto& [t, c] : type_counts_) {
        line(t, c);
      }
      std::cout << "\u2560" << std::string(kWidth, '\u2550') << "\u2563\n";
      line("TOPLAM", total);
      // Süre satırı
      char buf[32];
      std::snprintf(buf, sizeof(buf), "%.1f sn", elapsed_sec);
      std::string left  = " Sure:";
      std::string right = buf;
      int padding = kWidth - static_cast<int>(left.size()) -
                    static_cast<int>(right.size()) - 1;
      if (padding < 1) padding = 1;
      std::cout << "\u2551" << left << std::string(padding, ' ') << right << " \u2551\n";
    }

    std::cout << "\u255a" << std::string(kWidth, '\u2550') << "\u255d\n\n";
  }

private:
  RunStats() = default;
  RunStats(const RunStats&) = delete;
  RunStats& operator=(const RunStats&) = delete;

  std::map<std::string, int> type_counts_;
  std::chrono::steady_clock::time_point start_time_{};
};

// ─── ROS Node'a Erişim ───
// Önce config.blackboard'dan dener; bulamazsa globalRootBlackboard()'a
// fallback yapar (SubTree sınırı geçen autoremap hatalarına karşı güvence).
//
// Kullanım (herhangi bir BT node içinde):
//   auto ros = getRosNode(config());
//   auto pub = ros->create_publisher<std_msgs::msg::Bool>("/topic", 10);
inline rclcpp::Node::SharedPtr getRosNode(const BT::NodeConfiguration& config)
{
  rclcpp::Node::SharedPtr node;
  config.blackboard->get("ros_node", node);
  if (!node && globalRootBlackboard()) {
    globalRootBlackboard()->get("ros_node", node);
  }
  if (!node) {
    RCLCPP_ERROR(btLogger(),
      "Blackboard'da 'ros_node' bulunamadı! main.cpp'de set edilmeli.");
  }
  return node;
}

// ─── Odometry Veri Sağlayıcı (Singleton) ───
// Tüm BT node'larının erişebileceği paylaşılan pose/twist önbelleği.
// GPS fix gelmeden veri olmayabilir — tüm getter'lar bu durumda false döner.
//
// Kullanım:
//   main.cpp'de init:
//     OdometryProvider::instance().init(ros_node);
//
//   Herhangi bir BT node'unda:
//     double x, y, yaw;
//     if (OdometryProvider::instance().getPose(x, y, yaw)) { ... }
class OdometryProvider {
public:
  static OdometryProvider& instance() {
    static OdometryProvider inst;
    return inst;
  }

  /// ROS subscription'ı başlat. main.cpp'de ros_node oluşturulduktan sonra çağır.
  void init(rclcpp::Node::SharedPtr node,
            const std::string& topic = "/odometry/gnss_corrected")
  {
    if (sub_) return;  // zaten init edilmiş
    node_ = node;
    sub_ = node->create_subscription<nav_msgs::msg::Odometry>(
      topic, rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {
        std::lock_guard<std::mutex> lock(mtx_);
        last_msg_ = *msg;
        last_stamp_ = node_->now();
        has_data_ = true;
      });
    RCLCPP_INFO(btLogger(), "OdometryProvider: '%s' topic'ine abone olundu.", topic.c_str());
  }

  /// Mevcut pozisyonu al. false = henüz hiç veri gelmedi.
  bool getPose(double& x, double& y, double& yaw) const {
    std::lock_guard<std::mutex> lock(mtx_);
    if (!has_data_) return false;
    x = last_msg_.pose.pose.position.x;
    y = last_msg_.pose.pose.position.y;
    tf2::Quaternion q(
      last_msg_.pose.pose.orientation.x,
      last_msg_.pose.pose.orientation.y,
      last_msg_.pose.pose.orientation.z,
      last_msg_.pose.pose.orientation.w);
    yaw = tf2::getYaw(q);
    return true;
  }

  /// Mevcut doğrusal hızı al. false = henüz hiç veri gelmedi.
  bool getLinearSpeed(double& speed) const {
    std::lock_guard<std::mutex> lock(mtx_);
    if (!has_data_) return false;
    double vx = last_msg_.twist.twist.linear.x;
    double vy = last_msg_.twist.twist.linear.y;
    speed = std::sqrt(vx * vx + vy * vy);
    return true;
  }

  /// Son mesajın yaşını kontrol et. false = hiç veri gelmedi VEYA çok eski.
  bool isFresh(double max_age_sec = 1.0) const {
    std::lock_guard<std::mutex> lock(mtx_);
    if (!has_data_ || !node_) return false;
    double age = (node_->now() - last_stamp_).seconds();
    return age <= max_age_sec;
  }

private:
  OdometryProvider() = default;
  OdometryProvider(const OdometryProvider&) = delete;
  OdometryProvider& operator=(const OdometryProvider&) = delete;

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
  nav_msgs::msg::Odometry last_msg_;
  rclcpp::Time last_stamp_{0, 0, RCL_ROS_TIME};
  bool has_data_ = false;
  mutable std::mutex mtx_;
};

}  // namespace robotaksi_bt

#endif  // BT_NODE_BASE_HPP
