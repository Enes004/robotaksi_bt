# Segment BT — Groot2 Şemaları (Düzeltilmiş)

> Bu doküman `segment_bt.xml` dosyasındaki tüm alt-ağaçları Groot2'de görüneceği
> hiyerarşik yapıda gösterir. **Düzeltilmiş versiyondur** — Roundabout, PassengerStop,
> HandleStaticObstacle ve ValidateTurn hataları giderilmiştir.

---

## 1. MainTree (Ana Ağaç)

```mermaid
graph TD
    M["→ Sequence: Mission"]
    M --> S1["📝 Script<br/>cruise=0.60<br/>intersection=0.30<br/>roundabout=0.30<br/>tunnel=0.30<br/>parking=0.20"]
    M --> LM["⚡ LoadMission<br/>geojson_file, tour<br/>→ route, route_size, seg_index"]
    M --> FB["? Fallback: DriveWithRecovery"]
    FB --> DAS["🌲 SubTree: DriveAllSegments"]
    FB --> REC["🌲 SubTree: Recovery"]
    M --> FM["🌲 SubTree: FinishMission"]

    style M fill:#2d5a27,color:#fff
    style S1 fill:#4a4a8a,color:#fff
    style LM fill:#8a4a2d,color:#fff
    style FB fill:#5a5a2d,color:#fff
    style DAS fill:#2d4a5a,color:#fff
    style REC fill:#5a2d2d,color:#fff
    style FM fill:#2d4a5a,color:#fff
```

---

## 2. DriveAllSegments (Segment Döngüsü)

```mermaid
graph TD
    F["? Fallback"]
    F --> REP["🔄 Repeat: num_cycles=-1"]
    REP --> SEQ["→ Sequence: SegmentLoop"]
    SEQ --> HMS["❓ HasMoreSegments<br/>seg_index, route_size"]
    SEQ --> GCS["⚡ GetCurrentSegment<br/>route, seg_index<br/>→ seg_type, seg_goal, seg_meta"]
    SEQ --> ES["🌲 SubTree: ExecuteSegment"]
    SEQ --> AS["⚡ AdvanceSegment<br/>seg_index++"]
    F --> INV["🔃 Inverter"]
    INV --> HMS2["❓ HasMoreSegments"]

    style F fill:#5a5a2d,color:#fff
    style REP fill:#6a3a6a,color:#fff
    style SEQ fill:#2d5a27,color:#fff
    style HMS fill:#4a6a2d,color:#fff
    style GCS fill:#8a4a2d,color:#fff
    style ES fill:#2d4a5a,color:#fff
    style AS fill:#8a4a2d,color:#fff
    style INV fill:#6a3a6a,color:#fff
    style HMS2 fill:#4a6a2d,color:#fff
```

---

## 3. ExecuteSegment (Segment Router — Switch/Case)

```mermaid
graph TD
    FB["? Fallback: SegmentSwitch"]
    
    FB --> S1["→ Seq"]
    S1 --> C1["❓ IsSegmentType<br/>expected=LANE_FOLLOW"]
    S1 --> T1["🌲 Seg_LaneFollow"]

    FB --> S2["→ Seq"]
    S2 --> C2["❓ IsSegmentType<br/>expected=INTERSECTION"]
    S2 --> T2["🌲 Seg_Intersection"]

    FB --> S3["→ Seq"]
    S3 --> C3["❓ IsSegmentType<br/>expected=ROUNDABOUT"]
    S3 --> T3["🌲 Seg_Roundabout"]

    FB --> S4["→ Seq"]
    S4 --> C4["❓ IsSegmentType<br/>expected=TUNNEL"]
    S4 --> T4["🌲 Seg_Tunnel"]

    FB --> S5["→ Seq"]
    S5 --> C5["❓ IsSegmentType<br/>expected=PASSENGER_STOP"]
    S5 --> T5["🌲 Seg_PassengerStop"]

    FB --> S6["→ Seq"]
    S6 --> C6["❓ IsSegmentType<br/>expected=LANE_CHANGE"]
    S6 --> T6["🌲 Seg_LaneChange"]

    FB --> S7["→ Seq"]
    S7 --> C7["❓ IsSegmentType<br/>expected=PARKING"]
    S7 --> T7["🌲 Seg_Parking"]

    style FB fill:#5a5a2d,color:#fff
    style C1 fill:#4a6a2d,color:#fff
    style C2 fill:#4a6a2d,color:#fff
    style C3 fill:#4a6a2d,color:#fff
    style C4 fill:#4a6a2d,color:#fff
    style C5 fill:#4a6a2d,color:#fff
    style C6 fill:#4a6a2d,color:#fff
    style C7 fill:#4a6a2d,color:#fff
    style T1 fill:#2d4a5a,color:#fff
    style T2 fill:#2d4a5a,color:#fff
    style T3 fill:#2d4a5a,color:#fff
    style T4 fill:#2d4a5a,color:#fff
    style T5 fill:#2d4a5a,color:#fff
    style T6 fill:#2d4a5a,color:#fff
    style T7 fill:#2d4a5a,color:#fff
```

---

## 4. SafeDrive (Güvenli Hareket Primitifi)

```mermaid
graph TD
    RS["⟳ ReactiveSequence: SafeDrive"]
    RS --> SR["🌲 SubTree: SafetyReflexes"]
    RS --> FLS["⚡ FollowLaneSegment<br/>goal=seg_goal"]

    style RS fill:#8a2d2d,color:#fff
    style SR fill:#2d4a5a,color:#fff
    style FLS fill:#8a4a2d,color:#fff
```

---

## 5. SafetyReflexes (Güvenlik Katmanı)

```mermaid
graph TD
    RS["⟳ ReactiveSequence: SafetyReflexes"]

    RS --> FB1["? Fallback: 1-Acil Durdurma"]
    FB1 --> INV1["🔃 Inverter"]
    INV1 --> ESR["❓ EmergencyStopRequested"]
    FB1 --> KR["🔄 KeepRunningUntilFailure"]
    KR --> SV1["⚡ StopVehicle"]

    RS --> FB2["? Fallback: 2-Yaya"]
    FB2 --> INV2["🔃 Inverter"]
    INV2 --> PA["❓ PedestrianAhead"]
    FB2 --> SEQ2["→ Sequence"]
    SEQ2 --> SV2["⚡ StopVehicle"]
    SEQ2 --> WFC1["⚡ WaitForClear<br/>hazard=pedestrian"]

    RS --> FB3["? Fallback: 3-Dinamik Engel"]
    FB3 --> INV3["🔃 Inverter"]
    INV3 --> DOA["❓ DynamicObstacleAhead"]
    FB3 --> SEQ3["→ Sequence"]
    SEQ3 --> SV3["⚡ StopVehicle"]
    SEQ3 --> WFC2["⚡ WaitForClear<br/>hazard=dynamic"]

    RS --> FB4["? Fallback: 4-Statik Engel"]
    FB4 --> INV4["🔃 Inverter"]
    INV4 --> SOL["❓ StaticObstacleInLane"]
    FB4 --> HSO["🌲 HandleStaticObstacle"]

    style RS fill:#8a2d2d,color:#fff
    style FB1 fill:#aa0000,color:#fff
    style FB2 fill:#aa6600,color:#fff
    style FB3 fill:#aa4400,color:#fff
    style FB4 fill:#886600,color:#fff
```

---

## 6. HandleStaticObstacle (✅ DÜZELTİLDİ)

> **Düzeltme:** `CalculateLaneChange` sonrası `FollowLaneSegment` eklendi — artık hesaplanan hedefe gerçekten gidiliyor.

```mermaid
graph TD
    FB["? Fallback: HandleStaticObstacle"]
    FB --> SEQ1["→ Sequence: Sakınma"]
    SEQ1 --> ASA["❓ AvoidanceSpaceAvailable<br/>free_lane"]
    SEQ1 --> CLC["⚡ CalculateLaneChange<br/>current_pose, target_lane → seg_goal"]
    SEQ1 --> FLS["⚡ FollowLaneSegment<br/>goal=seg_goal 🆕"]
    FB --> SEQ2["→ Sequence: Dur-Bekle"]
    SEQ2 --> SV["⚡ StopVehicle"]
    SEQ2 --> WFC["⚡ WaitForClear<br/>hazard=static"]

    style FB fill:#886600,color:#fff
    style SEQ1 fill:#2d5a27,color:#fff
    style FLS fill:#226622,color:#fff,stroke:#0f0,stroke-width:2px
    style SEQ2 fill:#5a2d2d,color:#fff
```

---

## 7. Seg_LaneFollow (Düz Şerit Takibi)

```mermaid
graph TD
    SEQ["→ Sequence: Seg_LaneFollow"]
    SEQ --> SMS["⚡ SetMaxSpeed<br/>speed=cruise_speed"]
    SEQ --> RMP["⚡ RecordMissionPoint<br/>point=seg_meta, tolerance=1.0"]
    SEQ --> SD["🌲 SubTree: SafeDrive"]

    style SEQ fill:#2d5a27,color:#fff
    style SMS fill:#8a4a2d,color:#fff
    style RMP fill:#4a4a8a,color:#fff
    style SD fill:#2d4a5a,color:#fff
```

---

## 8. Seg_Intersection (Kavşak)

```mermaid
graph TD
    SEQ["→ Sequence: Seg_Intersection"]
    SEQ --> SMS1["⚡ SetMaxSpeed<br/>speed=intersection_speed"]
    SEQ --> FB["? Fallback: Tur Kapısı"]
    FB --> INV["🔃 Inverter"]
    INV --> ITC["❓ IsTrafficControlActive<br/>tour"]
    FB --> SEQ2["→ Sequence"]
    SEQ2 --> HTL["🌲 HandleTrafficLight"]
    SEQ2 --> HSS["🌲 HandleStopSign"]
    SEQ2 --> VT["🌲 ValidateTurn"]
    SEQ --> SD["🌲 SubTree: SafeDrive"]
    SEQ --> SMS2["⚡ SetMaxSpeed<br/>speed=cruise_speed"]

    style SEQ fill:#2d5a27,color:#fff
    style FB fill:#5a5a2d,color:#fff
    style INV fill:#6a3a6a,color:#fff
    style ITC fill:#4a6a2d,color:#fff
    style HTL fill:#2d4a5a,color:#fff
    style HSS fill:#2d4a5a,color:#fff
    style VT fill:#2d4a5a,color:#fff
    style SD fill:#2d4a5a,color:#fff
```

---

## 9. HandleTrafficLight (Işık Yönetimi)

```mermaid
graph TD
    FB["? Fallback: HandleTrafficLight"]
    FB --> INV1["🔃 Inverter"]
    INV1 --> TLA["❓ TrafficLightAhead<br/>→ light_color"]
    FB --> SEQ1["→ Sequence"]
    SEQ1 --> FB2["? Fallback: Kırmızı mı?"]
    FB2 --> INV2["🔃 Inverter"]
    INV2 --> ILR["❓ IsLightRed<br/>color=light_color"]
    FB2 --> SEQ2["→ Sequence"]
    SEQ2 --> SASL["⚡ StopAtStopLine<br/>tolerance=5.0m"]
    SEQ2 --> WFGL["⚡ WaitForGreenLight"]
    SEQ1 --> POG["⚡ ProceedOnGreen<br/>max_react_sec=5.0"]

    style FB fill:#aa0000,color:#fff
    style SEQ1 fill:#2d5a27,color:#fff
    style FB2 fill:#5a5a2d,color:#fff
    style SASL fill:#8a4a2d,color:#fff
    style WFGL fill:#8a4a2d,color:#fff
    style POG fill:#226622,color:#fff
```

---

## 10. HandleStopSign & ValidateTurn (✅ DÜZELTİLDİ)

> **Düzeltme:** `ValidateTurn`'de `ReplanRoute` artık `ForceSuccess` ile sarılı — başarısız olsa bile segment loop crash etmez.

```mermaid
graph TD
    subgraph HandleStopSign
        FB1["? Fallback"]
        FB1 --> INV1["🔃 Inverter"]
        INV1 --> SSA["❓ StopSignAhead"]
        FB1 --> SAP["⚡ StopAndProceed"]
    end

    subgraph ValidateTurn
        FB2["? Fallback"]
        FB2 --> INV2["🔃 Inverter"]
        INV2 --> TCS["❓ TurnConflictsWithSigns"]
        FB2 --> FS["🔃 ForceSuccess 🆕"]
        FS --> RR["⚡ ReplanRoute<br/>reason=sign_conflict"]
    end

    style FB1 fill:#5a5a2d,color:#fff
    style FB2 fill:#5a5a2d,color:#fff
    style SAP fill:#8a4a2d,color:#fff
    style FS fill:#226622,color:#fff,stroke:#0f0,stroke-width:2px
    style RR fill:#aa4400,color:#fff
```

---

## 11. Seg_Roundabout (✅ DÜZELTİLDİ)

> **Düzeltme:** `ExecuteRoundabout` yerine `SafeDrive` SubTree kullanılıyor — artık dönel kavşak içinde de güvenlik refleksleri çalışır.

```mermaid
graph TD
    SEQ["→ Sequence: Seg_Roundabout"]
    SEQ --> SMS1["⚡ SetMaxSpeed<br/>speed=roundabout_speed"]
    SEQ --> YAR["⚡ YieldAtRoundabout"]
    SEQ --> SD["🌲 SubTree: SafeDrive 🆕"]
    SEQ --> SMS2["⚡ SetMaxSpeed<br/>speed=cruise_speed"]

    style SEQ fill:#2d5a27,color:#fff
    style YAR fill:#aa6600,color:#fff
    style SD fill:#226622,color:#fff,stroke:#0f0,stroke-width:2px
```

---

## 12. Seg_Tunnel (Tünel)

```mermaid
graph TD
    SEQ["→ Sequence: Seg_Tunnel"]
    SEQ --> SMS1["⚡ SetMaxSpeed<br/>speed=tunnel_speed"]
    SEQ --> TH1["⚡ TurnHeadlights<br/>state=on"]
    SEQ --> SD["🌲 SubTree: SafeDrive"]
    SEQ --> TH2["⚡ TurnHeadlights<br/>state=off"]
    SEQ --> SMS2["⚡ SetMaxSpeed<br/>speed=cruise_speed"]

    style SEQ fill:#2d5a27,color:#fff
    style TH1 fill:#aaaa00,color:#000
    style SD fill:#2d4a5a,color:#fff
    style TH2 fill:#666600,color:#fff
```

---

## 13. Seg_PassengerStop (✅ DÜZELTİLDİ)

> **Düzeltme:** `CheckStopAccuracy` FAILURE dönerse pozisyon düzeltme döngüsü eklendi.

```mermaid
graph TD
    SEQ["→ Sequence: Seg_PassengerStop"]
    SEQ --> SD["🌲 SubTree: SafeDrive"]
    SEQ --> SV["⚡ StopVehicle"]
    SEQ --> FB["? Fallback: EnsureStopAccuracy 🆕"]
    FB --> CSA1["❓ CheckStopAccuracy<br/>tolerance=1.0m"]
    FB --> SEQ2["→ Sequence: Düzeltme"]
    SEQ2 --> FLS["⚡ FollowLaneSegment<br/>goal=seg_goal"]
    SEQ2 --> SV2["⚡ StopVehicle"]
    SEQ2 --> CSA2["❓ CheckStopAccuracy<br/>tolerance=1.0m"]
    SEQ --> DW["⚡ Dwell<br/>min=15sn, max=20sn"]
    SEQ --> SPE["⚡ SignalPassengerEvent<br/>event_type=seg_meta"]

    style SEQ fill:#2d5a27,color:#fff
    style SD fill:#2d4a5a,color:#fff
    style SV fill:#aa0000,color:#fff
    style FB fill:#226622,color:#fff,stroke:#0f0,stroke-width:2px
    style CSA1 fill:#4a6a2d,color:#fff
    style DW fill:#4a4a8a,color:#fff
    style SPE fill:#8a4a2d,color:#fff
```

---

## 14. Seg_LaneChange (Şerit Değişimi)

```mermaid
graph TD
    SEQ["→ Sequence: Seg_LaneChange"]
    SEQ --> CLC["⚡ CalculateLaneChange<br/>current_pose, target_lane → seg_goal"]
    SEQ --> SD["🌲 SubTree: SafeDrive"]

    style SEQ fill:#2d5a27,color:#fff
    style CLC fill:#8a4a2d,color:#fff
    style SD fill:#2d4a5a,color:#fff
```

---

## 15. Seg_Parking (Park)

```mermaid
graph TD
    SEQ["→ Sequence: Seg_Parking"]
    SEQ --> SMS["⚡ SetMaxSpeed<br/>speed=parking_speed"]
    SEQ --> SD["🌲 SubTree: SafeDrive"]
    SEQ --> RPER["⚡ RecordParkEntryReached"]
    SEQ --> FPS["⚡ FindParkingSlot<br/>→ slot_pose"]
    SEQ --> EP["⚡ ExecuteParking<br/>slot=slot_pose<br/>time_limit=180sn"]

    style SEQ fill:#2d5a27,color:#fff
    style SMS fill:#8a4a2d,color:#fff
    style SD fill:#2d4a5a,color:#fff
    style RPER fill:#4a4a8a,color:#fff
    style FPS fill:#aa6600,color:#fff
    style EP fill:#226622,color:#fff
```

---

## 16. Recovery & FinishMission

```mermaid
graph TD
    subgraph Recovery
        SEQ1["→ Sequence: Recovery"]
        SEQ1 --> IS["❓ IsStuck"]
        SEQ1 --> BU["⚡ BackUp<br/>dist=0.30m, speed=0.05"]
        SEQ1 --> SP["⚡ Spin<br/>dist=1.57rad"]
    end

    subgraph FinishMission
        SEQ2["→ Sequence: FinishMission"]
        SEQ2 --> SV["⚡ StopVehicle"]
    end

    style SEQ1 fill:#5a2d2d,color:#fff
    style IS fill:#4a6a2d,color:#fff
    style BU fill:#8a4a2d,color:#fff
    style SP fill:#8a4a2d,color:#fff
    style SEQ2 fill:#2d5a27,color:#fff
    style SV fill:#aa0000,color:#fff
```

---

## Düzeltme Özeti

| # | Sorun | Düzeltme | Diyagram |
|---|---|---|---|
| 1 | Roundabout'ta SafeDrive eksik | `SafeDrive` SubTree eklendi | §11 |
| 2 | PassengerStop pozisyon düzeltme yok | `EnsureStopAccuracy` Fallback eklendi | §13 |
| 3 | HandleStaticObstacle hareket eksik | `FollowLaneSegment` eklendi | §6 |
| 4 | ValidateTurn ReplanRoute crash riski | `ForceSuccess` sarmalı eklendi | §10 |

---

## Groot2'de Açma Talimatı

1. `segment_bt.xml` dosyasını Groot2'de açın: **File → Open**
2. Format: `BTCPP_format="4"` — Groot2 bunu native olarak destekler
3. `TreeNodesModel` bloğu XML içinde gömülü → custom node'lar port'larıyla görünecektir
4. Sol panelde alt-ağaç listesinden her SubTree'ye çift tıklayarak detayına inebilirsiniz
5. 🆕 işaretli node'lar düzeltme ile eklenen yeni elemanlardır
