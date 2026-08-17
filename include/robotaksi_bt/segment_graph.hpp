// ============================================================================
// Teknofest 2026 Robotaksi — Segment Graf Veri Yapısı
//
// Haritayı YÖNSÜZ graf olarak modelleyen sınıf.
//   - Düğümler (Node): her lanelet bir düğüm = bir segment.
//   - Kenarlar (Edge): lanelet uç noktaları 0.5 m dahilinde ise bağlı.
//
// GİRDİ: Haritacı ekibin 3 katmanlı GeoJSON dosyaları:
//   - lanelet_layer.geojson   (160 lanelet, center_b_id → linestring eşleşmesi)
//   - linestring_layer.geojson (452 çizgi, line_id → koordinatlar)
//
// Kullanım:
//   SegmentGraph graph;
//   graph.loadFromGeoJSON("lanelet_layer.geojson", "linestring_layer.geojson");
//   auto route = graph.planRoute({"1","22"});
// ============================================================================
#ifndef SEGMENT_GRAPH_HPP
#define SEGMENT_GRAPH_HPP

#include <string>
#include <vector>
#include <map>
#include <unordered_map>
#include <unordered_set>
#include <queue>
#include <limits>
#include <cmath>
#include <utility>
#include <fstream>
#include <algorithm>
#include <cctype>
#include <sstream>

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
  std::string type;  // LANE_FOLLOW, INTERSECTION, vb.
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

  // Tüm alt parçaların (center_b_id içindeki line_id'ler) başlangıç ve
  // bitiş noktaları. Bağlantı (komşuluk) kontrolleri sadece bunlara bakılarak yapılır.
  std::vector<std::pair<double, double>> connection_endpoints;
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
  // GeoJSON'dan graf yükleme (nlohmann::json tabanlı)
  //
  // Yeni 3 katmanlı GeoJSON formatı:
  //   lanelet_file:     lanelet_layer.geojson (160 lanelet)
  //   linestring_file:  linestring_layer.geojson (452 çizgi)
  //
  // center_b_id ile linestring eşleşerek her lanelet'in
  // merkez çizgisi (path_xy) elde edilir.
  // ──────────────────────────────────────────────
  bool loadFromGeoJSON(const std::string& lanelet_file,
                       const std::string& linestring_file) {
    nodes_.clear();
    segments_.clear();
    adjacency_.clear();
    segment_index_.clear();

    // ── 1) LİNESTRİNG KATMANI: line_id → koordinat listesi ──
    std::unordered_map<int, std::vector<std::pair<double, double>>> line_coords;
    {
      std::ifstream file(linestring_file);
      if (!file.is_open()) return false;

      nlohmann::json root;
      try { file >> root; } catch (...) { return false; }

      if (!root.contains("features") || !root["features"].is_array())
        return false;

      for (const auto& feat : root["features"]) {
        if (!feat.contains("properties") || !feat.contains("geometry"))
          continue;
        const auto& props = feat["properties"];
        if (!props.contains("line_id") || props["line_id"].is_null())
          continue;

        int line_id = props["line_id"].get<int>();
        const auto& geom = feat["geometry"];

        // MultiLineString: coordinates = [[[lon,lat], ...]]
        if (!geom.contains("coordinates") || !geom["coordinates"].is_array() ||
            geom["coordinates"].empty())
          continue;

        // İlk LineString'i al
        const auto& first_ls = geom["coordinates"][0];
        if (!first_ls.is_array()) continue;

        std::vector<std::pair<double, double>> coords;
        for (const auto& pt : first_ls) {
          if (pt.is_array() && pt.size() >= 2) {
            coords.emplace_back(pt[0].get<double>(), pt[1].get<double>());
          }
        }
        if (!coords.empty()) {
          line_coords[line_id] = std::move(coords);
        }
      }
    }

    // ── 2) LANELET KATMANI: her lanelet → segment ──
    {
      std::ifstream file(lanelet_file);
      if (!file.is_open()) return false;

      nlohmann::json root;
      try { file >> root; } catch (...) { return false; }

      if (!root.contains("features") || !root["features"].is_array())
        return false;

      for (const auto& feat : root["features"]) {
        if (!feat.contains("properties")) continue;
        const auto& props = feat["properties"];

        if (!props.contains("lanelet_id") || props["lanelet_id"].is_null())
          continue;
        if (!props.contains("center_b_id") || props["center_b_id"].is_null())
          continue;

        std::string lid = std::to_string(props["lanelet_id"].get<int>());

        // ── lanelet_type → büyük harf ──
        std::string type_str = "LANE_FOLLOW";  // varsayılan
        if (props.contains("lanelet_type") && !props["lanelet_type"].is_null()) {
          type_str = props["lanelet_type"].get<std::string>();
          std::transform(type_str.begin(), type_str.end(), type_str.begin(),
                         [](unsigned char c) { return std::toupper(c); });
        }

        // ── center_b_id parse: tek "347" veya virgüllü "373,374,375" ──
        std::string cbid_raw = props["center_b_id"].get<std::string>();
        std::vector<int> center_ids;
        {
          std::istringstream ss(cbid_raw);
          std::string token;
          while (std::getline(ss, token, ',')) {
            // Boşlukları temizle
            token.erase(std::remove_if(token.begin(), token.end(),
                        [](unsigned char c) { return std::isspace(c); }),
                        token.end());
            if (!token.empty()) {
              try { center_ids.push_back(std::stoi(token)); }
              catch (...) { /* geçersiz id, atla */ }
            }
          }
        }

        if (center_ids.empty()) continue;

        // ── Koordinatları birleştir (sırayla) ve Uç Noktaları Topla ──
        std::vector<std::pair<double, double>> merged_lonlat;
        std::vector<std::pair<double, double>> endpoints_lonlat;

        for (int cid : center_ids) {
          auto it = line_coords.find(cid);
          if (it == line_coords.end()) continue;
          const auto& lc = it->second;

          if (!lc.empty()) {
            endpoints_lonlat.push_back(lc.front());
            endpoints_lonlat.push_back(lc.back());
          }

          if (!merged_lonlat.empty() && !lc.empty()) {
            // Birleştirme yönünü kontrol et: mevcut son nokta,
            // yeni segmentin başına mı sonuna mı daha yakın?
            auto& last = merged_lonlat.back();
            double d_front = std::hypot(lc.front().first - last.first,
                                        lc.front().second - last.second);
            double d_back = std::hypot(lc.back().first - last.first,
                                       lc.back().second - last.second);
            if (d_back < d_front) {
              // Ters sırada birleştir
              for (auto rit = lc.rbegin(); rit != lc.rend(); ++rit) {
                merged_lonlat.push_back(*rit);
              }
            } else {
              for (const auto& p : lc) {
                merged_lonlat.push_back(p);
              }
            }
          } else {
            for (const auto& p : lc) {
              merged_lonlat.push_back(p);
            }
          }
        }

        if (merged_lonlat.empty()) continue;

        // ── Lon/Lat → Metre dönüşümü ──
        std::vector<std::pair<double, double>> path_meters;
        path_meters.reserve(merged_lonlat.size());
        for (const auto& [lon, lat] : merged_lonlat) {
          path_meters.emplace_back(lonToMeters(lon), latToMeters(lat));
        }

        std::vector<std::pair<double, double>> ep_meters;
        ep_meters.reserve(endpoints_lonlat.size());
        for (const auto& [lon, lat] : endpoints_lonlat) {
          ep_meters.emplace_back(lonToMeters(lon), latToMeters(lat));
        }

        // ── Cost: path noktaları arası öklid mesafe toplamı (metre) ──
        double path_cost = 0.0;
        for (size_t i = 1; i < path_meters.size(); ++i) {
          double dx = path_meters[i].first - path_meters[i - 1].first;
          double dy = path_meters[i].second - path_meters[i - 1].second;
          path_cost += std::hypot(dx, dy);
        }
        if (path_cost <= 0.0) path_cost = 1.0;

        // ── Goal: path'in SON noktası ──
        double gx = path_meters.back().first;
        double gy = path_meters.back().second;

        // ── Goal yaw: son iki noktanın yönü ──
        double gyaw = 0.0;
        if (path_meters.size() >= 2) {
          auto& p1 = path_meters[path_meters.size() - 2];
          auto& p2 = path_meters[path_meters.size() - 1];
          gyaw = std::atan2(p2.second - p1.second, p2.first - p1.first);
        }

        // ── Segment oluştur ──
        Segment seg;
        seg.id = lid;
        seg.from_node = lid;
        seg.to_node = lid;
        seg.type = type_str;
        seg.cost = path_cost;
        seg.goal_x = gx;
        seg.goal_y = gy;
        seg.goal_yaw = gyaw;
        seg.path_xy = std::move(path_meters);
        seg.connection_endpoints = std::move(ep_meters);

        // ── GraphNode oluştur ──
        GraphNode gn;
        gn.id = lid;
        gn.x = gx;
        gn.y = gy;
        gn.type = type_str;
        nodes_[lid] = gn;

        size_t idx = segments_.size();
        segment_index_[lid] = idx;
        segments_.push_back(std::move(seg));
      }
    }

    // ── 3) YÖNSÜZ BAĞLANTI KUR ──
    // İki lanelet bağlı: herhangi bir parçasının ucu (ilk veya son)
    // diğerinin herhangi bir parçasının ucuna 0.5 m'den yakınsa.
    constexpr double kProximityThreshold = 0.5;  // metre

    for (size_t i = 0; i < segments_.size(); ++i) {
      const auto& ep_i = segments_[i].connection_endpoints;
      if (ep_i.empty()) continue;

      for (size_t j = i + 1; j < segments_.size(); ++j) {
        const auto& ep_j = segments_[j].connection_endpoints;
        if (ep_j.empty()) continue;

        bool connected = false;
        for (const auto& pi : ep_i) {
          for (const auto& pj : ep_j) {
            double d = std::hypot(pi.first - pj.first, pi.second - pj.second);
            if (d < kProximityThreshold) {
              connected = true;
              break;
            }
          }
          if (connected) break;
        }

        if (connected) {
          adjacency_[segments_[i].id].push_back(j);
          adjacency_[segments_[j].id].push_back(i);
        }
      }
    }

    return !segments_.empty();
  }

  // ──────────────────────────────────────────────
  // Rota planlama: sıralı waypoint'lerden geçen en kısa segment dizisi
  //
  // waypoint_node_ids: ["1", "22"]
  // Her ardışık çift arasında Dijkstra çalıştırır ve birleştirir.
  // Rota bulunduktan sonra yön düzeltmesi yapılır.
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

    // Rota sonrası yön düzeltme
    if (!full_route.segments.empty()) {
      fixRouteDirections(full_route.segments);
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
  // path_xy üzerindeki TÜM noktalara bakarak en yakın lanelet'i bulur
  // (görev noktaları yolun ortasına denk gelebiliyor).
  std::string findNearestNode(double x, double y) const {
    std::string nearest;
    double min_dist = std::numeric_limits<double>::max();
    for (const auto& seg : segments_) {
      for (const auto& [px, py] : seg.path_xy) {
        double d = std::hypot(px - x, py - y);
        if (d < min_dist) {
          min_dist = d;
          nearest = seg.id;
        }
      }
    }
    return nearest;
  }

private:
  std::unordered_map<std::string, GraphNode> nodes_;
  std::vector<Segment> segments_;

  // Adjacency list: node_id → [segment_index, ...] (YÖNSÜZ)
  std::unordered_map<std::string, std::vector<size_t>> adjacency_;

  // Segment id → segments_ index (hızlı erişim)
  std::unordered_map<std::string, size_t> segment_index_;

  // ──────────────────────────────────────────────
  // İki uç nokta arasındaki mesafe (metre)
  // ──────────────────────────────────────────────
  static double endpointDist(const std::pair<double,double>& a,
                             const std::pair<double,double>& b) {
    return std::hypot(a.first - b.first, a.second - b.second);
  }

  // ──────────────────────────────────────────────
  // Dijkstra en kısa yol (YÖNSÜZ graf)
  //
  // Her segment bir düğüm. Komşuluk adjacency_ ile tanımlanır.
  // Cost = geçilen (hedef) segment'in kendi cost'u.
  // ──────────────────────────────────────────────
  std::vector<Segment> dijkstra(const std::string& start,
                                const std::string& goal) const {
    if (start == goal) return {};
    if (segment_index_.find(start) == segment_index_.end()) return {};
    if (segment_index_.find(goal) == segment_index_.end()) return {};

    // Mesafe ve önceki düğüm
    std::unordered_map<std::string, double> dist;
    std::unordered_map<std::string, std::string> prev;
    for (const auto& seg : segments_) {
      dist[seg.id] = std::numeric_limits<double>::max();
      prev[seg.id] = "";
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

      for (size_t neighbor_idx : adj_it->second) {
        const auto& neighbor = segments_[neighbor_idx];
        double new_dist = dist[u] + neighbor.cost;
        if (new_dist < dist[neighbor.id]) {
          dist[neighbor.id] = new_dist;
          prev[neighbor.id] = u;
          pq.push({new_dist, neighbor.id});
        }
      }
    }

    // Yolu geri izle
    if (dist[goal] == std::numeric_limits<double>::max()) return {};

    std::vector<Segment> path;
    std::string cur = goal;
    while (!cur.empty() && cur != start) {
      auto idx_it = segment_index_.find(cur);
      if (idx_it == segment_index_.end()) break;
      path.push_back(segments_[idx_it->second]);
      cur = prev[cur];
    }
    // Başlangıç segment'ini de ekle
    {
      auto idx_it = segment_index_.find(start);
      if (idx_it != segment_index_.end()) {
        path.push_back(segments_[idx_it->second]);
      }
    }
    std::reverse(path.begin(), path.end());
    return path;
  }

  // ──────────────────────────────────────────────
  // Rota sonrası yön düzeltme
  //
  // Yönsüz graftan gelen rota segmentlerinin path_xy yönü
  // trafik akış yönünü göstermeyebilir. Ardışık segment
  // çiftlerinin bağlantı uçlarına bakarak gerektiğinde
  // path_xy'yi ters çevirir.
  //
  // Mantık: segment A'dan segment B'ye geçiliyorsa,
  // A'nın B'ye DEĞEN ucu A'nın çıkış noktası olmalı.
  // Eğer A'nın "başlangıç" ucu B'ye değiyorsa, A ters
  // okunmalı demektir.
  // ──────────────────────────────────────────────
  static void fixRouteDirections(std::vector<Segment>& route) {
    if (route.size() < 2) return;

    constexpr double kThreshold = 0.5;

    // İlk segment için: ikinci segmente değen uç çıkış olmalı.
    // İlk segment'in hangi ucu ikinci segment'in herhangi bir ucuna yakın?
    {
      auto& A = route[0];
      const auto& B = route[1];
      if (A.path_xy.size() >= 2 && B.path_xy.size() >= 2) {
        // A'nın sonu → B'nin herhangi bir ucu
        double d_back_front = endpointDist(A.path_xy.back(), B.path_xy.front());
        double d_back_back  = endpointDist(A.path_xy.back(), B.path_xy.back());
        // A'nın başı → B'nin herhangi bir ucu
        double d_front_front = endpointDist(A.path_xy.front(), B.path_xy.front());
        double d_front_back  = endpointDist(A.path_xy.front(), B.path_xy.back());

        double min_back = std::min(d_back_front, d_back_back);
        double min_front = std::min(d_front_front, d_front_back);

        // Eğer A'nın başı B'ye daha yakınsa → A ters çevrilmeli
        if (min_front < min_back && min_front < kThreshold) {
          std::reverse(A.path_xy.begin(), A.path_xy.end());
          recalcGoal(A);
        }
      }
    }

    // Ortadaki ve son segmentler: önceki segment'in çıkışına göre yönlendir
    for (size_t i = 1; i < route.size(); ++i) {
      const auto& prev_seg = route[i - 1];
      auto& cur = route[i];

      if (prev_seg.path_xy.empty() || cur.path_xy.empty()) continue;

      const auto& prev_exit = prev_seg.path_xy.back();

      double d_front = endpointDist(cur.path_xy.front(), prev_exit);
      double d_back  = endpointDist(cur.path_xy.back(), prev_exit);

      // Önceki segment'in çıkışı cur'un giriş noktası olmalı
      // Eğer cur'un sonu daha yakınsa → cur ters çevrilmeli
      if (d_back < d_front) {
        std::reverse(cur.path_xy.begin(), cur.path_xy.end());
        recalcGoal(cur);
      }
    }
  }

  // ──────────────────────────────────────────────
  // Segment'in goal_x/goal_y/goal_yaw'ını path_xy'den yeniden hesapla
  // ──────────────────────────────────────────────
  static void recalcGoal(Segment& seg) {
    if (seg.path_xy.empty()) return;
    seg.goal_x = seg.path_xy.back().first;
    seg.goal_y = seg.path_xy.back().second;
    if (seg.path_xy.size() >= 2) {
      auto& p1 = seg.path_xy[seg.path_xy.size() - 2];
      auto& p2 = seg.path_xy[seg.path_xy.size() - 1];
      seg.goal_yaw = std::atan2(p2.second - p1.second,
                                 p2.first - p1.first);
    }
  }
};

}  // namespace robotaksi_bt

#endif  // SEGMENT_GRAPH_HPP
