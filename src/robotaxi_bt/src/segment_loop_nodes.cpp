// ============================================================================
// segment_loop_nodes.cpp — Segment Döngüsü Node İmplementasyonları
//
// Bu dosyadaki TÜM node'lar tam implemente — harici bağımlılık YOK.
// ============================================================================

#include "robotaxi_bt/segment_loop_nodes.hpp"

namespace robotaxi_bt {

// ═══════════════════════════════════════════════════════════════
// HasMoreSegments::tick()
//
// MANTIK:
//   seg_index < route_size → SUCCESS (daha segment var)
//   seg_index >= route_size → FAILURE (tüm segmentler bitti)
//
// BT AKIŞI:
//   DriveAllSegments ağacında Repeat(-1) içindeki ilk node.
//   FAILURE dönünce Repeat durur, Inverter ile SUCCESS'a çevrilir,
//   Fallback tamamlanır → "tüm segmentler bitti" anlamına gelir.
//
// NOT: Bu bir Condition node — RUNNING dönemez, yan etkisi yoktur.
// ═══════════════════════════════════════════════════════════════
BT::NodeStatus HasMoreSegments::tick()
{
  // Port'lardan değerleri oku
  int seg_index = 0;
  int route_size = 0;

  // getInput<T>(port_adı, değişken) — Blackboard'dan okur
  // Başarısızsa (port yok/tip uyumsuz) değişken değişmez
  getInput("seg_index", seg_index);
  getInput("route_size", route_size);

  // Saf karşılaştırma — hiçbir state değiştirmez
  bool has_more = (seg_index < route_size);

  // RCLCPP_DEBUG: sadece debug seviyesinde log (üretimde görünmez)
  RCLCPP_DEBUG(btLogger(), "HasMoreSegments: %d / %d → %s",
               seg_index, route_size, has_more ? "DEVAM" : "BİTTİ");

  // Condition node dönüş kuralı:
  //   true  → SUCCESS (koşul sağlandı)
  //   false → FAILURE (koşul sağlanmadı)
  return has_more ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

// ═══════════════════════════════════════════════════════════════
// IsSegmentType::tick()
//
// MANTIK:
//   seg_type == expected → SUCCESS (bu tip doğru)
//   seg_type != expected → FAILURE (bu tip değil)
//
// BT AKIŞI:
//   ExecuteSegment ağacında Fallback > Sequence deseniyle kullanılır:
//
//   <Fallback name="SegmentSwitch">
//     <Sequence>
//       <IsSegmentType seg_type="{seg_type}" expected="LANE_FOLLOW" />
//       <SubTree ID="Seg_LaneFollow" />   ← sadece LANE_FOLLOW ise çalışır
//     </Sequence>
//     <Sequence>
//       <IsSegmentType seg_type="{seg_type}" expected="INTERSECTION" />
//       <SubTree ID="Seg_Intersection" /> ← sadece INTERSECTION ise çalışır
//     </Sequence>
//     ...
//   </Fallback>
//
//   Bu desen C++'daki switch/case ile eşdeğerdir:
//     LANE_FOLLOW  → Seg_LaneFollow
//     INTERSECTION → Seg_Intersection
//     ROUNDABOUT   → Seg_Roundabout
//     vb.
// ═══════════════════════════════════════════════════════════════
BT::NodeStatus IsSegmentType::tick()
{
  std::string seg_type, expected;
  getInput("seg_type", seg_type);
  getInput("expected", expected);

  // Büyük/küçük harf duyarlı karşılaştırma
  bool match = (seg_type == expected);

  RCLCPP_DEBUG(btLogger(), "IsSegmentType: '%s' == '%s' → %s",
               seg_type.c_str(), expected.c_str(), match ? "EVET" : "HAYIR");

  return match ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

// ═══════════════════════════════════════════════════════════════
// IsTrafficControlActive::tick()
//
// MANTIK:
//   tour >= 2 → SUCCESS (trafik kontrolleri aktif)
//   tour < 2  → FAILURE (Tur 1'de trafik kontrolleri kapalı)
//
// ŞARTNAME KURALI:
//   Tur 1'de sahada trafik ışığı ve tabelalar fiziksel olarak YOKTUR.
//   Bu yüzden Tur 1'de algılama katmanları (K5-K7) atlanmalıdır.
//   Aksi halde "fantom" algılamalar yanlış davranışa sebep olur.
//
// BT AKIŞI:
//   SafetyReflexes içinde Fallback + Inverter deseniyle kapı görevi yapar:
//
//   <Fallback>
//     <Inverter><IsTrafficControlActive tour="{tour}" /></Inverter>
//     <ReactiveSequence name="TrafficControlLayers">
//       ... K5/K6/K7 katmanları ...
//     </ReactiveSequence>
//   </Fallback>
//
//   Tur 1 (FAILURE):
//     → Inverter → SUCCESS → Fallback biter → katmanlar atlanır ✓
//   Tur 2+ (SUCCESS):
//     → Inverter → FAILURE → Fallback ikinci çocuğa geçer → katmanlar çalışır ✓
// ═══════════════════════════════════════════════════════════════
BT::NodeStatus IsTrafficControlActive::tick()
{
  int tour = 1;
  getInput("tour", tour);

  bool active = (tour >= 2);

  RCLCPP_DEBUG(btLogger(), "IsTrafficControlActive: tur=%d → %s",
               tour, active ? "AKTİF" : "PASİF");

  return active ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

// ═══════════════════════════════════════════════════════════════
// AdvanceSegment::tick()
//
// MANTIK:
//   1. seg_index'i Blackboard'dan oku (getInput)
//   2. 1 artır
//   3. Blackboard'a geri yaz (setOutput)
//
// PORT TİPİ: BidirectionalPort<int>
//   Hem InputPort hem OutputPort gibi davranır.
//   Aynı Blackboard anahtarını okuyup yazabilir.
//   XML'de: <AdvanceSegment seg_index="{seg_index}" />
//   {seg_index} → Blackboard'daki "seg_index" anahtarına bağlı
//
// BT AKIŞI:
//   SegmentLoop'un SON node'u.
//   ExecuteSegment başarılı olduktan sonra çağrılır.
//   Bir sonraki tick'te HasMoreSegments yeni index ile kontrol eder.
// ═══════════════════════════════════════════════════════════════
BT::NodeStatus AdvanceSegment::tick()
{
  int seg_index = 0;

  // Blackboard'dan oku
  getInput("seg_index", seg_index);

  // 1 artır
  seg_index++;

  // Blackboard'a geri yaz — BidirectionalPort sayesinde aynı anahtar
  setOutput("seg_index", seg_index);

  RCLCPP_INFO(btLogger(), "AdvanceSegment: yeni index = %d", seg_index);

  // SyncActionNode — her zaman SUCCESS veya FAILURE
  return BT::NodeStatus::SUCCESS;
}

// ═══════════════════════════════════════════════════════════════
// ClearHandledFlags::tick()
//
// MANTIK:
//   Blackboard'daki tüm "handled_*" bayraklarını false'a çeker.
//
// NEDEN GEREKLİ?
//   ReactiveSequence her tick'te TÜM çocukları baştan kontrol eder.
//   Bir DUR tabelası algılandığında StopAndProceed çalışır ve
//   handled_stop_sign = true yapar. Eğer bu bayrak sıfırlanmazsa,
//   aynı segment içinde tabela tekrar görüldüğünde tekrar durulur
//   → sonsuz döngü!
//
//   Bayrak sıfırlama SEGMENTler arasında yapılır:
//     SegmentLoop başı → ClearHandledFlags → bayraklar temiz
//     Segment içi → tabela tetiklenir → bayrak true olur (bir kez)
//     Sonraki segment → ClearHandledFlags → bayraklar yine temiz
//
// BLACKBOARD ERİŞİMİ:
//   config().blackboard doğrudan Blackboard nesnesine erişim sağlar.
//   Port tanımlamadan herhangi bir anahtarı okuyup yazabilirsiniz.
//   Bu özel bir durum — normalde port kullanmak tercih edilir.
// ═══════════════════════════════════════════════════════════════
BT::NodeStatus ClearHandledFlags::tick()
{
  // config().blackboard → Blackboard'a doğrudan erişim (port'suz)
  auto bb = config().blackboard;

  // Tüm latch bayraklarını sıfırla
  bb->set("handled_stop_sign", false);
  bb->set("handled_traffic_light", false);
  bb->set("handled_road_sign", false);
  bb->set("handled_pedestrian_crossing", false);

  RCLCPP_DEBUG(btLogger(), "ClearHandledFlags: tüm bayraklar sıfırlandı");
  return BT::NodeStatus::SUCCESS;
}

}  // namespace robotaxi_bt
