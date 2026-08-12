// ============================================================================
// segment_loop_nodes.hpp — Segment Döngüsü Node'ları
//
// Bu dosyadaki node'lar segment listesi üzerinde iterasyon yapar:
//   - HasMoreSegments : segment kaldı mı? (Condition)
//   - IsSegmentType   : aktif segment tipi kontrolü (Condition)
//   - IsTrafficControlActive : tur≥2 mi? (Condition)
//   - AdvanceSegment  : seg_index++ (Action)
//   - ClearHandledFlags : latch bayraklarını sıfırla (Action)
//
// BAĞIMLILIK: YOK — Saf mantık, harici sensör/harita gerektirmez.
// DURUM: ✅ Tam implemente — başka kimseye ihtiyaç yok.
//
// BT.CPP v3 uyumlu.
// ============================================================================
#ifndef SEGMENT_LOOP_NODES_HPP
#define SEGMENT_LOOP_NODES_HPP

#include "robotaksi_bt/bt_node_base.hpp"
#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/condition_node.h"
#include <string>

namespace robotaksi_bt {

// ─────────────────────────────────────────────
// HasMoreSegments (Condition)
//
// Segment döngüsünde "daha segment var mı?" kontrolü yapar.
// seg_index < route_size ise SUCCESS (evet, var), değilse FAILURE (bitti).
//
// XML kullanımı:
//   <HasMoreSegments seg_index="{seg_index}" route_size="{route_size}" />
//
// DriveAllSegments ağacında Repeat döngüsünün başında çağrılır.
// FAILURE dönünce Repeat durur → tüm segmentler tamamlandı.
// ─────────────────────────────────────────────
class HasMoreSegments : public BT::ConditionNode {
public:
  HasMoreSegments(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config) {}

  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<int>("seg_index", "Şu anki segment indeksi"),
      BT::InputPort<int>("route_size", "Toplam segment sayısı")
    };
  }

  BT::NodeStatus tick() override;
};

// ─────────────────────────────────────────────
// IsSegmentType (Condition)
//
// Aktif segmentin tipini kontrol eder.
// ExecuteSegment ağacında Fallback+Sequence deseniyle switch/case gibi çalışır:
//   <IsSegmentType seg_type="{seg_type}" expected="LANE_FOLLOW" />
//   <IsSegmentType seg_type="{seg_type}" expected="INTERSECTION" />
//   ...
//
// seg_type == expected ise SUCCESS → o segment alt-ağacı çalışır.
// ─────────────────────────────────────────────
class IsSegmentType : public BT::ConditionNode {
public:
  IsSegmentType(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config) {}

  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<std::string>("seg_type", "Aktif segment tipi"),
      BT::InputPort<std::string>("expected", "Beklenen tip (LANE_FOLLOW, INTERSECTION, ...)")
    };
  }

  BT::NodeStatus tick() override;
};

// ─────────────────────────────────────────────
// IsTrafficControlActive (Condition)
//
// Şartname kuralı: Tur 1'de sahada trafik ışığı/tabela YOKTUR.
// Tur ≥ 2 ise trafik kontrolleri aktiftir.
//
// SafetyReflexes içinde K5-K7 katmanlarını kapılar:
//   <IsTrafficControlActive tour="{tour}" />
//   SUCCESS (tur≥2) → ışık/tabela kontrolleri çalışır
//   FAILURE (tur=1) → atlanır
// ─────────────────────────────────────────────
class IsTrafficControlActive : public BT::ConditionNode {
public:
  IsTrafficControlActive(const std::string& name, const BT::NodeConfiguration& config)
    : BT::ConditionNode(name, config) {}

  static BT::PortsList providedPorts() {
    return {
      BT::InputPort<int>("tour", "Tur numarası (1, 2 veya 3)")
    };
  }

  BT::NodeStatus tick() override;
};

// ─────────────────────────────────────────────
// AdvanceSegment (SyncAction)
//
// Segment döngüsünde bir sonraki segmente geçer: seg_index++
// BidirectionalPort kullanır — aynı portu hem okur hem yazar.
//
// XML kullanımı:
//   <AdvanceSegment seg_index="{seg_index}" />
//
// SegmentLoop'un sonunda, ExecuteSegment başarılı olduktan sonra çağrılır.
// ─────────────────────────────────────────────
class AdvanceSegment : public BT::SyncActionNode {
public:
  AdvanceSegment(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts() {
    return {
      BT::BidirectionalPort<int>("seg_index", "Artırılacak segment indeksi")
    };
  }

  BT::NodeStatus tick() override;
};

// ─────────────────────────────────────────────
// ClearHandledFlags (SyncAction)
//
// Her yeni segmente girildiğinde çağrılır.
// Latch bayraklarını sıfırlar — aynı segment içinde bir tabela/olay
// sonsuz döngüye girmesin diye "handled" bayrakları kullanılır.
// Yeni segment = tüm bayraklar false.
//
// Örnek akış:
//   Segment A → DUR tabelası → handled_stop_sign = true (tekrar tetiklenmez)
//   Segment B → ClearHandledFlags → handled_stop_sign = false (tekrar tetiklenebilir)
// ─────────────────────────────────────────────
class ClearHandledFlags : public BT::SyncActionNode {
public:
  ClearHandledFlags(const std::string& name, const BT::NodeConfiguration& config)
    : BT::SyncActionNode(name, config) {}

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override;
};

}  // namespace robotaksi_bt

#endif  // SEGMENT_LOOP_NODES_HPP
