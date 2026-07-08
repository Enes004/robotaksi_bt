// ============================================================================
// Teknofest 2026 Robotaksi — Segment Graf Veri Yapısı
//
// Haritayı yönlü graf olarak modelleyen sınıf.
//   - Düğümler (Node): kavşak, tünel ağzı, durak, park girişi vb.
//   - Kenarlar (Segment): iki düğüm arasında tek yönde, tek şeritte yol parçası.
//
// Kullanım:
//   SegmentGraph graph;
//   graph.loadFromYAML("segment_map.yaml");
//   auto route = graph.planRoute({"N_START", "N_DURAK_1", "N_PARK_IN"});
// ============================================================================
#ifndef SEGMENT_GRAPH_HPP
#define SEGMENT_GRAPH_HPP

#include <string>
#include <vector>
#include <map>
#include <unordered_map>
#include <queue>
#include <limits>
#include <fstream>
#include <sstream>
#include <cmath>

namespace robotaxi_bt {

// ─── Veri Yapıları ───

struct GraphNode {
  std::string id;
  double x = 0.0;
  double y = 0.0;
};

struct Segment {
  std::string id;
  std::string from_node;        // kaynak düğüm id
  std::string to_node;          // hedef düğüm id
  std::string type;             // LANE_FOLLOW, INTERSECTION, ROUNDABOUT, TUNNEL, PASSENGER_STOP, LANE_CHANGE, PARKING
  std::string lane;             // right / left (opsiyonel)
  std::string meta;             // ek veri: exit_node, mission id, vb.
  double cost = 1.0;            // kenar ağırlığı (mesafe veya süre)

  // Segment bitiş hedefi (Pose2D basitleştirilmiş)
  double goal_x = 0.0;
  double goal_y = 0.0;
  double goal_yaw = 0.0;
};

// ─── Route: planlanmış segment dizisi ───

struct Route {
  std::vector<Segment> segments;
  double total_cost = 0.0;

  size_t size() const { return segments.size(); }
  bool empty() const { return segments.empty(); }

  const Segment& at(size_t index) const { return segments.at(index); }
};

// ─── Segment Graf Sınıfı ───

class SegmentGraph {
public:
  SegmentGraph() = default;

  // ──────────────────────────────────────────────
  // YAML'dan graf yükleme
  // Basitleştirilmiş YAML parser (sadece segment_map.yaml formatı)
  // ──────────────────────────────────────────────
  bool loadFromYAML(const std::string& filepath) {
    std::ifstream file(filepath);
    if (!file.is_open()) {
      return false;
    }

    std::string line;
    std::string current_section;  // "nodes" veya "segments"

    while (std::getline(file, line)) {
      // Yorum ve boş satırları atla
      auto trimmed = trim(line);
      if (trimmed.empty() || trimmed[0] == '#') continue;

      // Bölüm tespiti
      if (trimmed == "nodes:") { current_section = "nodes"; continue; }
      if (trimmed == "segments:") { current_section = "segments"; continue; }

      if (current_section == "nodes") {
        parseNodeLine(trimmed);
      } else if (current_section == "segments") {
        parseSegmentLine(trimmed);
      }
    }

    // Segment cost'larını düğüm mesafelerinden hesapla
    computeSegmentCosts();
    return !nodes_.empty();
  }

  // ──────────────────────────────────────────────
  // Rota planlama: sıralı waypoint'lerden geçen en kısa segment dizisi
  //
  // waypoint_node_ids: ["N_START", "N_DURAK_1", "N_PARK_IN"]
  // Her ardışık çift arasında Dijkstra çalıştırır ve birleştirir.
  // ──────────────────────────────────────────────
  Route planRoute(const std::vector<std::string>& waypoint_node_ids) const {
    Route full_route;

    for (size_t i = 0; i + 1 < waypoint_node_ids.size(); ++i) {
      auto sub = dijkstra(waypoint_node_ids[i], waypoint_node_ids[i + 1]);
      if (sub.empty()) {
        // Rota bulunamadı — boş dön
        return Route{};
      }
      for (auto& s : sub) {
        full_route.segments.push_back(s);
        full_route.total_cost += s.cost;
      }
    }

    return full_route;
  }

  // ──────────────────────────────────────────────
  // Düğüm/segment erişim
  // ──────────────────────────────────────────────
  const GraphNode* getNode(const std::string& id) const {
    auto it = nodes_.find(id);
    return (it != nodes_.end()) ? &it->second : nullptr;
  }

  size_t nodeCount() const { return nodes_.size(); }
  size_t segmentCount() const { return segments_.size(); }

  const std::vector<Segment>& allSegments() const { return segments_; }

  // En yakın düğümü bul (x,y koordinatına göre)
  std::string findNearestNode(double x, double y) const {
    std::string nearest;
    double min_dist = std::numeric_limits<double>::max();
    for (auto& [id, node] : nodes_) {
      double d = std::hypot(node.x - x, node.y - y);
      if (d < min_dist) {
        min_dist = d;
        nearest = id;
      }
    }
    return nearest;
  }

private:
  std::unordered_map<std::string, GraphNode> nodes_;
  std::vector<Segment> segments_;

  // Adjacency list: node_id → [segment_index, ...]
  std::unordered_map<std::string, std::vector<size_t>> adjacency_;

  // ──────────────────────────────────────────────
  // Dijkstra en kısa yol
  // ──────────────────────────────────────────────
  std::vector<Segment> dijkstra(const std::string& start, const std::string& goal) const {
    if (start == goal) return {};
    if (adjacency_.find(start) == adjacency_.end()) return {};

    // Mesafe ve önceki segment
    std::unordered_map<std::string, double> dist;
    std::unordered_map<std::string, int> prev_seg;  // segment index
    for (auto& [id, _] : nodes_) {
      dist[id] = std::numeric_limits<double>::max();
      prev_seg[id] = -1;
    }
    dist[start] = 0.0;

    // Min-heap: (cost, node_id)
    using PQEntry = std::pair<double, std::string>;
    std::priority_queue<PQEntry, std::vector<PQEntry>, std::greater<PQEntry>> pq;
    pq.push({0.0, start});

    while (!pq.empty()) {
      auto [d, u] = pq.top();
      pq.pop();

      if (u == goal) break;
      if (d > dist[u]) continue;

      auto adj_it = adjacency_.find(u);
      if (adj_it == adjacency_.end()) continue;

      for (size_t seg_idx : adj_it->second) {
        const auto& seg = segments_[seg_idx];
        double new_dist = dist[u] + seg.cost;
        if (new_dist < dist[seg.to_node]) {
          dist[seg.to_node] = new_dist;
          prev_seg[seg.to_node] = static_cast<int>(seg_idx);
          pq.push({new_dist, seg.to_node});
        }
      }
    }

    // Yolu geri izle
    if (dist[goal] == std::numeric_limits<double>::max()) return {};

    std::vector<Segment> path;
    std::string cur = goal;
    while (cur != start && prev_seg[cur] >= 0) {
      path.push_back(segments_[prev_seg[cur]]);
      cur = segments_[prev_seg[cur]].from_node;
    }
    std::reverse(path.begin(), path.end());
    return path;
  }

  // ──────────────────────────────────────────────
  // Basit YAML parser yardımcıları
  // ──────────────────────────────────────────────
  void parseNodeLine(const std::string& line) {
    // Format: N_START: {x: 0.0, y: 0.0}  veya  - {id: N_START, x: 0.0, y: 0.0}
    // Basitleştirilmiş: key: {x: val, y: val}
    auto colon = line.find(':');
    if (colon == std::string::npos) return;

    std::string id = trim(line.substr(0, colon));
    if (id.empty() || id[0] == '-') return;

    GraphNode node;
    node.id = id;
    node.x = extractDouble(line, "x:");
    node.y = extractDouble(line, "y:");

    nodes_[id] = node;
  }

  void parseSegmentLine(const std::string& line) {
    // Format: - {id: S_AB, from: N_A, to: N_B, type: LANE_FOLLOW, ...}
    if (line.find("id:") == std::string::npos) return;

    Segment seg;
    seg.id = extractString(line, "id:");
    seg.from_node = extractString(line, "from:");
    seg.to_node = extractString(line, "to:");
    seg.type = extractString(line, "type:");
    seg.lane = extractString(line, "lane:");
    seg.meta = extractString(line, "meta:");

    if (!seg.id.empty() && !seg.from_node.empty() && !seg.to_node.empty()) {
      size_t idx = segments_.size();
      segments_.push_back(seg);
      adjacency_[seg.from_node].push_back(idx);
    }
  }

  void computeSegmentCosts() {
    for (auto& seg : segments_) {
      auto from_it = nodes_.find(seg.from_node);
      auto to_it = nodes_.find(seg.to_node);
      if (from_it != nodes_.end() && to_it != nodes_.end()) {
        seg.cost = std::hypot(to_it->second.x - from_it->second.x,
                              to_it->second.y - from_it->second.y);
        // Hedef pozisyon = bitiş düğümünün koordinatı
        seg.goal_x = to_it->second.x;
        seg.goal_y = to_it->second.y;
        // Yaw = from→to yönü
        seg.goal_yaw = std::atan2(to_it->second.y - from_it->second.y,
                                   to_it->second.x - from_it->second.x);
      }
      if (seg.cost <= 0.0) seg.cost = 1.0;
    }
  }

  // ──────────────────────────────────────────────
  // String yardımcıları
  // ──────────────────────────────────────────────
  static std::string trim(const std::string& s) {
    auto start = s.find_first_not_of(" \t\r\n");
    if (start == std::string::npos) return "";
    auto end = s.find_last_not_of(" \t\r\n");
    return s.substr(start, end - start + 1);
  }

  static double extractDouble(const std::string& line, const std::string& key) {
    auto pos = line.find(key);
    if (pos == std::string::npos) return 0.0;
    pos += key.size();
    while (pos < line.size() && (line[pos] == ' ' || line[pos] == '\t')) ++pos;
    std::string num;
    while (pos < line.size() && (std::isdigit(line[pos]) || line[pos] == '.' || line[pos] == '-')) {
      num += line[pos++];
    }
    return num.empty() ? 0.0 : std::stod(num);
  }

  static std::string extractString(const std::string& line, const std::string& key) {
    auto pos = line.find(key);
    if (pos == std::string::npos) return "";
    pos += key.size();
    while (pos < line.size() && (line[pos] == ' ' || line[pos] == '\t')) ++pos;
    std::string val;
    while (pos < line.size() && line[pos] != ',' && line[pos] != '}' && line[pos] != ' ') {
      val += line[pos++];
    }
    return val;
  }
};

}  // namespace robotaxi_bt

#endif  // SEGMENT_GRAPH_HPP
