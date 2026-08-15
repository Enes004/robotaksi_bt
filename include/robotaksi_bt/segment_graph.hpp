// ============================================================================
// Teknofest 2026 Robotaksi — Segment Graf Veri Yapısı
//
// Haritayı yönlü graf olarak modelleyen sınıf.
//   - Düğümler (Node): her lanelet bir düğüm.
//   - Kenarlar (Edge): lanelet'ler arası geçiş.
//
// GİRDİ: Haritacı ekibin lanelet YAML dosyası (yaml-cpp ile parse edilir).
//
// Kullanım:
//   SegmentGraph graph;
//   graph.loadFromYAML("harita.yaml");
//   auto route = graph.planRoute({"4","10","16"});
// ============================================================================
#ifndef SEGMENT_GRAPH_HPP
#define SEGMENT_GRAPH_HPP

#include <string>
#include <vector>
#include <map>
#include <unordered_map>
#include <queue>
#include <limits>
#include <cmath>
#include <utility>
#include <fstream>
#include <algorithm>
#include <cctype>

#include <yaml-cpp/yaml.h>
#include <nlohmann/json.hpp>

namespace robotaksi_bt {

// ─── Datum (Sıfır Nokta) ───
// Equirectangular projeksiyonun referans noktası.
// Lokalizasyon ekibiyle netleşince bu iki değeri değiştirin.
constexpr double kDatumLat = 40.7897;   // derece
constexpr double kDatumLon = 29.5090;   // derece

// ─── Projeksiyon Yardımcıları ───

// Enlem/Boylam'ı yerel metre düzlemine çevirir (equirectangular).
// x = doğu yönü (lon farkı), y = kuzey yönü (lat farkı)
inline double lonToMeters(double lon) {
  constexpr double kMetersPerDegLon =
      111320.0 * std::cos(kDatumLat * M_PI / 180.0);
  return (lon - kDatumLon) * kMetersPerDegLon;
}

inline double latToMeters(double lat) {
  constexpr double kMetersPerDegLat = 110540.0;
  return (lat - kDatumLat) * kMetersPerDegLat;
}

// ─── Veri Yapıları ───

struct GraphNode {
  std::string id;
  double x = 0.0;   // metre (yerel düzlem)
  double y = 0.0;   // metre (yerel düzlem)
  std::string type;  // LANE_FOLLOW, INTERSECTION, vb. (loadTypesFromGeoJSON ile dolar)
};

struct Segment {
  std::string id;
  std::string from_node;        // kaynak düğüm id (lanelet_id string)
  std::string to_node;          // hedef düğüm id (lanelet_id string)
  std::string type;             // LANE_FOLLOW, INTERSECTION, ROUNDABOUT, TUNNEL, PASSENGER_STOP, LANE_CHANGE, PARKING
  std::string lane;             // right / left (opsiyonel)
  std::string meta;             // ek veri: exit_node, mission id, vb.
  double cost = 1.0;            // kenar ağırlığı — metre cinsinden path uzunluğu

  // Segment bitiş hedefi (Pose2D basitleştirilmiş)
  double goal_x = 0.0;
  double goal_y = 0.0;
  double goal_yaw = 0.0;

  // Şerit merkez çizgisi noktaları (yerel metre düzleminde).
  // Nav2 FollowPath action'ına verilecek.
  std::vector<std::pair<double, double>> path_xy;
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
  // YAML'dan graf yükleme (yaml-cpp tabanlı)
  //
  // Beklenen format:
  //   nodes:
  //     - lanelet_id: 4
  //       start: [lon, lat]
  //       end:   [lon, lat]
  //       path:  [[lon,lat], ...]
  //       length: ...           # kullanılmıyor
  //       lanelet_properties: {}
  //   edges:
  //     - source: 4
  //       target: 10
  //       cost: ...             # kullanılmıyor — kendi metre cost'unu hesaplıyoruz
  // ──────────────────────────────────────────────
  bool loadFromYAML(const std::string& filepath) {
    YAML::Node root;
    try {
      root = YAML::LoadFile(filepath);
    } catch (const YAML::Exception&) {
      return false;
    }

    nodes_.clear();
    segments_.clear();
    adjacency_.clear();
    lanelet_paths_.clear();

    // ── 1) NODES ──
    if (!root["nodes"] || !root["nodes"].IsSequence()) {
      return false;
    }

    for (const auto& node_yaml : root["nodes"]) {
      std::string lid = std::to_string(node_yaml["lanelet_id"].as<int>());

      // "end" noktasını düğüm pozisyonu olarak al (segment hedefi)
      double end_lon = node_yaml["end"][0].as<double>();
      double end_lat = node_yaml["end"][1].as<double>();

      GraphNode gn;
      gn.id = lid;
      gn.x = lonToMeters(end_lon);
      gn.y = latToMeters(end_lat);
      nodes_[lid] = gn;

      // path noktalarını metre'ye çevirip sakla
      std::vector<std::pair<double, double>> path_meters;
      if (node_yaml["path"] && node_yaml["path"].IsSequence()) {
        for (const auto& pt : node_yaml["path"]) {
          double plon = pt[0].as<double>();
          double plat = pt[1].as<double>();
          path_meters.emplace_back(lonToMeters(plon), latToMeters(plat));
        }
      }
      lanelet_paths_[lid] = std::move(path_meters);

      // TODO: lanelet_properties dolunca buradan segment_type oku.
      // Şu an haritacı ekip boş gönderiyor, dolayısıyla aşağıda
      // varsayılan "LANE_FOLLOW" kullanılıyor.
      // Örnek gelecek format:
      //   lanelet_properties:
      //     segment_type: INTERSECTION
      //     meta: "exit_node:12"
    }

    // ── 2) EDGES ──
    if (!root["edges"] || !root["edges"].IsSequence()) {
      return false;
    }

    for (const auto& edge_yaml : root["edges"]) {
      std::string src = std::to_string(edge_yaml["source"].as<int>());
      std::string tgt = std::to_string(edge_yaml["target"].as<int>());

      // Kaynak düğüm yoksa atla
      if (nodes_.find(src) == nodes_.end() ||
          nodes_.find(tgt) == nodes_.end()) {
        continue;
      }

      Segment seg;
      seg.id = src + "_" + tgt;
      seg.from_node = src;
      seg.to_node = tgt;

      // TODO: lanelet_properties dolunca buradan segment_type oku.
      seg.type = "LANE_FOLLOW";

      // path noktalarından metre cinsinden cost hesapla
      const auto& path = lanelet_paths_[src];
      seg.path_xy = path;

      double path_cost = 0.0;
      for (size_t i = 1; i < path.size(); ++i) {
        double dx = path[i].first - path[i - 1].first;
        double dy = path[i].second - path[i - 1].second;
        path_cost += std::hypot(dx, dy);
      }

      // Path boşsa veya tek noktaysa, düğüm arası düz mesafe kullan
      if (path_cost <= 0.0) {
        const auto& fn = nodes_[src];
        const auto& tn = nodes_[tgt];
        path_cost = std::hypot(tn.x - fn.x, tn.y - fn.y);
      }
      seg.cost = path_cost;

      // Hedef pozisyon = hedef düğümün koordinatı (bitiş noktası)
      seg.goal_x = nodes_[tgt].x;
      seg.goal_y = nodes_[tgt].y;

      // Yaw = son iki path noktasından hesapla; yoksa düğüm arası yön
      if (path.size() >= 2) {
        auto& p1 = path[path.size() - 2];
        auto& p2 = path[path.size() - 1];
        seg.goal_yaw = std::atan2(p2.second - p1.second,
                                   p2.first - p1.first);
      } else {
        seg.goal_yaw = std::atan2(nodes_[tgt].y - nodes_[src].y,
                                   nodes_[tgt].x - nodes_[src].x);
      }

      if (seg.cost <= 0.0) seg.cost = 1.0;

      size_t idx = segments_.size();
      segments_.push_back(std::move(seg));
      adjacency_[src].push_back(idx);
    }

    return !nodes_.empty();
  }

  // ──────────────────────────────────────────────
  // GeoJSON'dan segment tiplerini yükleme (nlohmann::json tabanlı)
  //
  // ÖNEMLİ: Bu fonksiyon loadFromYAML(...)'dan SONRA, çağıran kod
  // (örn. BT node) tarafından AYRICA çağrılmalıdır — loadFromYAML
  // içine gömülü değildir. Sebep: harita ekibi geometriyi
  // (routing_graph.yaml) ve lanelet tiplerini (lanelet_layer.geojson)
  // AYRI dosyalar halinde, AYRI zamanlarda/sürümlerde gönderiyor; biri
  // güncellenirken diğeri sabit kalabiliyor. İki yükleme adımını ayrı
  // tutmak, tipler güncellendiğinde geometriyi (veya tam tersini)
  // yeniden parse etmeden sadece ilgili fonksiyonu çağırmayı mümkün
  // kılıyor.
  //
  // Beklenen format:
  //   { "features": [
  //       { "properties": { "lanelet_id": 224, "lanelet_type": "passenger_stop" }, ... },
  //       ...
  //   ]}
  //
  // lanelet_type küçük harf gelir (örn. "passenger_stop"); Segment::type
  // BÜYÜK HARF bekler (örn. "PASSENGER_STOP", bkz. BT XML'deki
  // IsSegmentType expected="..." karşılaştırması) — burada çevrilir.
  //
  // Eşleştirme mantığı: Her lanelet KENDİ tipini taşır. GeoJSON'daki
  // lanelet_id, segments_ içindeki Segment::from_node alanıyla
  // DOĞRUDAN eşleştirilir. Bir lanelet'ten birden fazla çıkış
  // (segment) olabilir — hepsi aynı tipi alır. Çıkışı olmayan
  // (dead-end) lanelet'lerin tipi de nodes_ üzerinden saklanır,
  // böylece planRoute sonrasında ekstra bilgi çekilebilir.
  //
  // Dosyada eşleşmeyen (geometri var ama tip dosyasında olmayan)
  // lanelet'ler mevcut değerini (loadFromYAML'daki varsayılan
  // "LANE_FOLLOW") korur — hata verilmez.
  // ──────────────────────────────────────────────
  bool loadTypesFromGeoJSON(const std::string& filepath) {
    std::ifstream file(filepath);
    if (!file.is_open()) {
      return false;
    }

    nlohmann::json root;
    try {
      file >> root;
    } catch (const nlohmann::json::exception&) {
      return false;
    }

    if (!root.contains("features") || !root["features"].is_array()) {
      return false;
    }

    // Hızlı erişim için from_node → segment indeksleri haritası oluştur.
    // adjacency_ KULLANILAMAZ çünkü adjacency_ sadece çıkışı olan
    // düğümleri içerir; dead-end lanelet'ler (passenger_stop, parking
    // vb.) orada yer almaz.
    std::unordered_map<std::string, std::vector<size_t>> from_node_index;
    for (size_t i = 0; i < segments_.size(); ++i) {
      from_node_index[segments_[i].from_node].push_back(i);
    }

    for (const auto& feature : root["features"]) {
      if (!feature.contains("properties")) continue;
      const auto& props = feature["properties"];
      if (!props.contains("lanelet_id") || !props.contains("lanelet_type")) continue;
      if (props["lanelet_id"].is_null() || props["lanelet_type"].is_null()) continue;

      std::string lanelet_id = std::to_string(props["lanelet_id"].get<int>());
      std::string type_upper = props["lanelet_type"].get<std::string>();
      std::transform(type_upper.begin(), type_upper.end(), type_upper.begin(),
                     [](unsigned char c) { return std::toupper(c); });

      // 1) Bu lanelet'ten çıkan tüm segmentlerin tipini güncelle
      auto idx_it = from_node_index.find(lanelet_id);
      if (idx_it != from_node_index.end()) {
        for (size_t seg_idx : idx_it->second) {
          segments_[seg_idx].type = type_upper;
        }
      }

      // 2) Düğüm düzeyinde tip bilgisini de sakla (dead-end lanelet'ler
      //    dahil — çıkışı olmasa bile tip sorgulanabilsin diye).
      auto node_it = nodes_.find(lanelet_id);
      if (node_it != nodes_.end()) {
        node_it->second.type = type_upper;
      }
    }

    return true;
  }

  // ──────────────────────────────────────────────
  // Rota planlama: sıralı waypoint'lerden geçen en kısa segment dizisi
  //
  // waypoint_node_ids: ["4", "10", "16"]
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

  const std::unordered_map<std::string, GraphNode>& allNodes() const { return nodes_; }

  // En yakın düğümü bul (x,y koordinatına göre — metre cinsinden)
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

  // Lanelet path noktaları (metre cinsinden) — edge cost hesabı için
  std::unordered_map<std::string, std::vector<std::pair<double, double>>> lanelet_paths_;

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
};

}  // namespace robotaksi_bt

#endif  // SEGMENT_GRAPH_HPP
