# Robotaksi — Segment-Tabanlı Behaviour Tree Mimarisi

**Teknofest 2026 Robotaksi-Binek Otonom Araç Yarışması**
Hazırlayan: BT / Karar Verme Sorumlusu — *(senin için taslak)*
Referanslar: Şartname v1.0 (02.01.2026), `general_driving_bt.xml`, `segment_bt.xml`

---

## 0. Bu doküman ne anlatıyor?

Takım, haritayı **segment segment** ele almak ve **her şeridi ayrı bir segment** olarak işlemek istiyor; tekrar eden yol parçalarında aynı alt-ağacı yeniden kullanmak (DRY) istiyor. Bu doküman:

1. Mevcut ağacın neyi doğru, neyi eksik yaptığını,
2. Haritanın şartnameye göre **doğru** yorumunu,
3. "Her şerit ayrı segment" fikrinin teknik karşılığını (yönlü şerit grafı),
4. Tekrar kullanılabilir alt-ağaçlarla kurulan yeni BT mimarisini,
5. Şartnamedeki **puanlı kuralların** hangi node'a düştüğünü,
6. Groot2'de açılacak `segment_bt.xml` dosyasının yapısını

anlatır. Amaç sadece kod değil; *neden böyle kurulduğunu* anlayarak geliştirebilmen.

---

## 1. Mevcut ağacın (`general_driving_bt.xml`) analizi

### İyi olan
- Tabela davranışları doğru ayrıştırılmış: `RoundaboutAction`, `TunnelAction`, `BusStopAction`, `LaneRegulationAction`, `MandatoryTurnAction`, `ForbiddenTurnAction`, `NoEntryAction` — bunlar şartname tabela listesiyle birebir örtüşüyor.
- `current_vertex` / `next_vertex` blackboard değişkenleri **graf fikrinin** zaten kafanda olduğunu gösteriyor. Segment mimarisi bunun doğal devamı.
- En dışta `RecoveryNode` + `BackUp`/`Spin` kurtarma mantığı sağlam.

### Eksik / takımın istemediği yön
| Konu | Mevcut ağaç | Sorun |
|---|---|---|
| Yapı | Tek `ReactiveFallback`, "serbest dolaşım" | Rota yok; her şey reaktif tetikleniyor |
| Şerit | Şerit kavramı yok | Takım "her şerit ayrı segment" istiyor |
| Rota kaynağı | `NavigateToPose free_roam` (Nav2 global planner) | Takım planlamayı BT'nin yönetmesini istiyor |
| Tur farkı | Yok | Tur 1 statik (ışık/tabela yok), Tur 2-3 dinamik |
| Puanlı kurallar | Çoğu yok | Yolcu 15-20 sn, ışık toleransı/5 sn, 1 m hassasiyet, 3 dk park, dik park girmemiş |
| Yolcu/Park | `BusStopAction` var ama süre yok; park yok | Şartname puanının büyük kısmı burada |

Sonuç: Mevcut ağaç "reaktif sürücü davranışı kütüphanesi" olarak değerli ama **görev planı** ve **segment iskeleti** yok. Yeni mimari bu node'ların çoğunu yeniden kullanır, sadece onları bir segment iskeletine oturtur.

---

## 2. Haritanın doğru okunması (Şartname Şekil 1)

`implementation_plan.md`'deki ilk yorumda iki şey yanlış/eksikti. Şartnamedeki tabela listesi + parkur görseli + puanlama tablolarını birlikte okuyunca harita şöyle:

```
        ┌───────────────────────────────────────────────┐
        │   blok    blok    blok          ┌── PARK ──┐    │
        │  ┌────┐  ┌────┐  ┌────┐         │ ▟ ▟ ▟ ▟  │    │  PARK = dik (perpendicular) park,
        │  │    │  │    │  │    │         │ (dik park) │    │         sağ üst köşe (P-3a tabelası)
        │  └────┘  └────┘  └────┘         └────────────┘   │
        │  ┌────┐         ┌────┐  ┌────┐                  │
   [D]──┤  │TÜN │   ( O ) │    │  │    │ ──[D]            │  ( O ) = DÖNEL KAVŞAK (ortadaki
        │  │EL  │  roundabout                             │           yeşil daire, TT-37)
        │  └────┘         └────┘  └────┘                  │  TÜNEL  = sol blok (B-49a, +200 ops.)
        │  ┌────┐  ┌────┐  ┌────┐  ┌────┐                 │
        │  │    │  │    │  │    │  │    │                 │  [D] = DURAK (B-22) → yolcu
        │  └────┘  └────┘  └────┘  └────┘                 │        al/bırak noktası adayı
        │       [D]            [D]                        │
        └───────────────────────────────────────────────┘
```

**Düzeltilen yorumlar:**

| Eleman | İlk (yanlış) yorum | Doğru yorum (şartname) |
|---|---|---|
| `D` işaretleri | "Başlangıç/durak noktası?" | **DURAK (B-22)** — yolcu indirme/bindirme noktası. GeoJSON'daki `gorev_n` noktaları buralarda işaretlenir. |
| Yeşil daire | Belirsiz | **Dönel kavşak** — tabela **TT-37 "Ada Etrafında Dönünüz"**. |
| Sol blok | "Tünel (tek/çift?)" | **Tünel (B-49a)**. Geçiş **opsiyonel** ve tek seferlik **+200 puan**. |
| Sağ üst | "Park alanı" | Doğru, ama **dik park**. En az 3 araç kapasiteli; P-3a tabelası olan ve önü boş slota park. |
| Yollar | "Manhattan grid" | Doğru: ızgara + çevre yolu. **Her yol 2 yönlü/çok şeritli** → graf yönlü kenarlarla modellenir. |

> **Kritik:** Şartname "Aksi belirtilmediği sürece trafik soldan akar" diyor. Bu, grafın **yönlü** olmasını ve şerit seçimini doğrudan etkiler. Yanlış yön = **Ters Yön İhlali** (diskalifiye riski). Bu yüzden şeritleri yönlü kenar yapmak sadece bir tercih değil, ihlalden kaçınmanın yolu.

---

## 3. "Her şerit ayrı segment" → Yönlü Şerit Grafı

Takımın istediği şey, formel olarak şu: **harita = yönlü graf (directed graph).**

- **Düğüm (node/vertex):** karar noktası — kavşak, dönel kavşak giriş/çıkışı, tünel ağzı, durak, park girişi.
- **Kenar (segment):** iki düğüm arasında **tek yönde**, **tek şeritte** ilerlenen yol parçası.

Bunun pratik sonuçları:

1. **İki yönlü bir yol = en az 2 segment** (her yön bir kenar). Yönde 2 şerit varsa = 4 segment.
2. **Şerit değişimi de bir kenardır** (`LANE_CHANGE` tipi). Engelden sakınma veya B-50 şerit düzenleme tabelası bu kenarı tetikler.
3. **Yön bilgisi grafta gömülü** olduğu için araç asla ters yöne giremez (graf zaten izin vermez).
4. **Tekrar eden parçalar = aynı segment TİPİ.** Bütün düz şerit kenarları `LANE_FOLLOW` tipindedir ve **aynı alt-ağacı** (`Seg_LaneFollow`) çalıştırır. Takımın "kopyala-yapıştır / aynı kodu kullan" dediği şey budur: kod tekrarı yok, **tip → tek alt-ağaç** eşlemesi var.

### Örnek: bir yol parçasının segmentlere ayrılması

```
  Kavşak_A  ════════════ iki yönlü, 2 şerit/yön ════════════  Kavşak_B

  Yönlü graf karşılığı (4 segment):
    S_AB_1 : A→B  (sağ şerit)    tip=LANE_FOLLOW
    S_AB_2 : A→B  (sol şerit)    tip=LANE_FOLLOW
    S_BA_1 : B→A  (sağ şerit)    tip=LANE_FOLLOW
    S_BA_2 : B→A  (sol şerit)    tip=LANE_FOLLOW
    + şerit değişimi gerekiyorsa:  S_AB_1→S_AB_2  tip=LANE_CHANGE
```

Dördü de **aynı** `Seg_LaneFollow` alt-ağacını kullanır; sadece `seg_goal` (kenarın bitiş düğümü) ve şerit kimliği farklıdır.

---

## 4. Temel mimari kararlar

### Karar 1 — Rotayı BT yönetir, Nav2 değil
Görev başında GeoJSON'daki noktalar (`start`, `gorev_n`, `park_giris`) okunur; önceden hazırlanmış **segment haritası** (graf) üzerinde, sırasıyla geçilmesi gereken noktalardan geçen **en kısa yasal rota** çıkarılır. Sonuç: `route = [S_start, S_AB_1, S_INT_B, ... , S_PARK]` gibi bir **segment dizisi.** BT bu diziyi sırayla yürütür. Bu, takımın "Nav2 global planner'a güvenmeyelim" isteğini **tam** karşılar.

### Karar 2 — Nav2'yı tamamen atma; hareketi soyutla
Takımın endişesi haklı ama Nav2'yı komple silmek, costmap + engel sakınma + lokal kontrolü sıfırdan yazmak demektir; finale yetişmez. Çözüm: hareketi **tek bir soyut node** arkasına koymak →

> `FollowLaneSegment(goal, max_speed)` — "şu segmentin sonuna, şeridinde kalarak git."

Bu node'un **içini takım istediği gibi doldurur:**
- **Seçenek A (önerilen):** Kameradan şerit takibi (vision lane-following) + lokal engel kontrolü. Şartnamenin "yol geometrisi kamerayla tanınabilir" vurgusuna uyar.
- **Seçenek B:** Kısa mesafe için Nav2 `FollowPath`/`NavigateToPose` (sadece o segment içinde, global planlama BT'de).

BT'nin geri kalanı bu seçimden **habersizdir.** Yani "Nav2 kullanalım mı?" tartışmasını mimariden ayırdık; istediğin zaman içini değiştirirsin, ağaç değişmez.

### Karar 3 — Güvenlik refleksleri küresel ve reaktif
Acil durdurma, yaya, dinamik engel, statik engel — bunlar hangi segmentte olursak olalım **araya girip** mevcut davranışı kesmeli. Bunu, her hareket primitifini saran tekrar kullanılabilir bir **reaktif katmanla** (`SafeDrive` → `SafetyReflexes`) sağlıyoruz. Öncelik sırası (yüksekten düşüğe):

1. **Acil durdurma** (UMS-1 / acil stop talebi) → tam dur.
2. **Yaya / yaya geçidi (B-14a)** → dur, geçene kadar bekle.
3. **Dinamik engel** → yavaşla/dur, geçene kadar bekle (*çarpma = diskalifiye*).
4. **Statik engel** → sakın (şerit değiştir); yer yoksa dur, gerekirse yeniden planla (şartname: yer yoksa o yoldan geçme).

### Karar 4 — Tur kapısı (tek ağaç, üç tur)
Üç tur için ayrı ağaç yazma. Blackboard'da `tour ∈ {1,2,3}` tut. `IsTrafficControlActive` koşulu **Tur ≥ 2** ise ışık/tabela işleme bloğunu açar; Tur 1'de kapatır.

| | Işık/Tabela | Dinamik engel | Yolcu al/bırak | Park |
|---|---|---|---|---|
| **Tur 1** | ❌ (statik) | ❌ (statik engel var) | ✅ (3 nokta) | ✅ |
| **Tur 2** | ✅ | ✅ | ❌ | ✅ |
| **Tur 3** | ✅ | ✅ | ✅ | ✅ |

Yolcu noktaları zaten **rotaya** (segment dizisine) tur bazında eklenir; ışık/tabela ise BT içinde `tour` ile kapılanır.

---

## 5. Segment tipleri ve şartname eşlemesi

Her tip, tekrar kullanılan bir alt-ağaca karşılık gelir. Tip sayısını **7** ile sınırlı tuttuk — fazlası bakımı zorlaştırır.

| Segment tipi | Alt-ağaç | Ne yapar | İlgili şartname kuralı / puan |
|---|---|---|---|
| `LANE_FOLLOW` | `Seg_LaneFollow` | Şeritte kalarak segment sonuna git | Şerit ihlali yok: **+50**; ihlal: **-50/ihlal** |
| `INTERSECTION` | `Seg_Intersection` | Yavaşla → (Tur≥2) ışık/DUR/tabela → dönüşü uygula | Işık: **+60/+40**; tabela: **+50 / -50/ihlal** |
| `ROUNDABOUT` | `Seg_Roundabout` | Boşluk bekle, doğru çıkıştan ayrıl (TT-37) | Tabela uyumu (**-50** ihlali önler) |
| `TUNNEL` | `Seg_Tunnel` | Far aç, yavaş geç, far kapat | Tünelden geçme: **+200** (opsiyonel, 1 kez) |
| `PASSENGER_STOP` | `Seg_PassengerStop` | 1 m hassasiyetle dur, **15-20 sn** bekle | Doğru yolcu işlemi: **+70/nokta** |
| `LANE_CHANGE` | `Seg_LaneChange` | Hedef şeride güvenli geçiş | Şerit ihlali sayılmaz (manevra) |
| `PARKING` | `Seg_Parking` | Giriş noktasından geç (+20), uygun slot bul, **dik park** (≤3 dk) | Giriş: **+20**; park: **+80** |

Ek olarak görev noktası geçişi (1 m içinden geçme = **+30/nokta**) `FollowLaneSegment` ilerlerken `RecordMissionPoint` mantığıyla işaretlenir.

---

## 6. Örnek segment haritası ve rota

### 6.1 Segment haritası (önceden, elle hazırlanır) — `segment_map.yaml`

```yaml
# Düğümler (harita/utm/local frame koordinatları — harita ekibinden gelecek)
nodes:
  N_START:    {x: 0.0,  y: 0.0}
  N_INT_A:    {x: 10.0, y: 0.0}     # kavşak A
  N_INT_B:    {x: 20.0, y: 0.0}     # kavşak B
  N_RND:      {x: 15.0, y: -8.0}    # dönel kavşak merkezi
  N_TUN_IN:   {x: 3.0,  y: -5.0}    # tünel girişi
  N_TUN_OUT:  {x: 3.0,  y: -12.0}   # tünel çıkışı
  N_DURAK_1:  {x: 25.0, y: -2.0}    # DURAK (gorev_1)
  N_PARK_IN:  {x: 28.0, y: 6.0}     # park_giris

# Segmentler = YÖNLÜ kenarlar (her şerit / her yön ayrı satır)
segments:
  - {id: S_START_A,  from: N_START,  to: N_INT_A, type: LANE_FOLLOW, lane: right}
  - {id: S_A_B_R,    from: N_INT_A,  to: N_INT_B, type: LANE_FOLLOW, lane: right}
  - {id: S_A_B_L,    from: N_INT_A,  to: N_INT_B, type: LANE_FOLLOW, lane: left}
  - {id: S_B_A_R,    from: N_INT_B,  to: N_INT_A, type: LANE_FOLLOW, lane: right}  # ters yön ayrı kenar
  - {id: S_INT_A,    from: N_INT_A,  to: N_INT_A, type: INTERSECTION}
  - {id: S_TUNNEL,   from: N_TUN_IN, to: N_TUN_OUT, type: TUNNEL, bonus: 200}
  - {id: S_RND,      from: N_INT_B,  to: N_RND,   type: ROUNDABOUT, exit: N_DURAK_1}
  - {id: S_DURAK_1,  from: N_RND,    to: N_DURAK_1, type: PASSENGER_STOP, mission: gorev_1}
  - {id: S_PARK,     from: N_DURAK_1, to: N_PARK_IN, type: PARKING}
  - {id: S_LC_AB,    from: S_A_B_R,  to: S_A_B_L, type: LANE_CHANGE}   # şerit→şerit
```

> Bu YAML bir kez hazırlanır, **bütün turlarda** kullanılır. Koordinatlar harita ekibinden gelir (bkz. §10 açık sorular).

### 6.2 Çalışma anında rota oluşumu

`LoadMission`, GeoJSON'daki sıralı noktalardan (`start → gorev_1 → gorev_2 → gorev_3 → park_giris`) ve segment grafından, **en kısa yasal** segment dizisini çıkarır:

```
# Tur 3 örnek rota (yolcu + dinamik + park):
route = [ S_START_A, S_INT_A, S_A_B_R, S_RND, S_DURAK_1,
          S_A_B_R(geri-leg), S_DURAK_2, ... , S_PARK ]

# Tur 1 örnek rota (statik, ışık/tabela pasif, 3 yolcu):
route = [ S_START_A, S_DURAK_1, S_DURAK_2, S_DURAK_3, S_PARK ]
```

BT bu diziyi indeksleyerek (`seg_index`) tek tek yürütür. Tünel rotaya **opsiyonel** olarak eklenir (yalnızca +200 değer kazandıracaksa ve güvenliyse).

---

## 7. BT mimarisi — alt-ağaç katmanları

```
MainTree
├── LoadMission                       # GeoJSON + graf → route, seg_index=0
├── [Recovery sarmalı]
│   └── DriveAllSegments              # rota bitene kadar döngü
│       └── (her segment)
│           └── ExecuteSegment        # SWITCH: seg_type → ilgili Seg_*
│               ├── Seg_LaneFollow ───┐
│               ├── Seg_Intersection  │
│               ├── Seg_Roundabout    │ hepsi hareket için
│               ├── Seg_Tunnel        ├─► SafeDrive'ı çağırır
│               ├── Seg_PassengerStop │
│               ├── Seg_LaneChange    │
│               └── Seg_Parking ──────┘
└── FinishMission                     # final raporu / dur

SafeDrive  (her hareketli segmentin kullandığı tek primitif)
└── ReactiveSequence
    ├── SafetyReflexes                # acil > yaya > dinamik > statik (her tick)
    └── FollowLaneSegment             # asıl hareket (vision veya Nav2 — içi takıma ait)
```

**Neden bu yapı?**
- `ExecuteSegment` bir **switch-case**: `IsSegmentType` koşullarıyla doğru alt-ağaca dallanır. Yeni tip eklemek = bir satır.
- Bütün `Seg_*` alt-ağaçları, hareketi **`SafeDrive`** üzerinden yapar. Yani güvenlik refleksleri **otomatik olarak her yerde** geçerli; tek yerde tanımlı, her segmentte etkili (DRY).
- `Seg_LaneFollow` haritadaki onlarca düz şerit için **aynı** alt-ağaç → takımın "tekrar eden parçalar için aynı kod" isteği.

### 7.1 Önemli alt-ağaçların davranışı

**`Seg_Intersection`** (kavşak):
```
Sequence
├── SetMaxSpeed(yavaş)
├── Fallback                          # Tur 1'de tümü atlanır
│   ├── Inverter(IsTrafficControlActive)   # Tur1 → SUCCESS, blok atlanır
│   └── Sequence
│       ├── HandleTrafficLight        # kırmızı→toleransta dur→yeşil bekle→5sn'de kalk
│       ├── HandleStopSign            # DUR (TT-2) → tam dur → devam
│       └── ValidateTurn              # planlanan dönüş tabelayla çelişiyorsa ReplanRoute
├── SafeDrive                         # kavşağı geç / dönüşü uygula
└── SetMaxSpeed(normal)
```

**`HandleTrafficLight`** (ışık puanı kritik):
```
Fallback
├── Inverter(TrafficLightAhead)       # ışık yoksa SUCCESS
└── Sequence
    ├── Fallback                      # kırmızıysa
    │   ├── Inverter(IsLightRed)
    │   └── Sequence
    │       ├── StopAtStopLine(tolerance=5.0)   # şartname: 0<d<5 → +60
    │       └── WaitForGreenLight               # yeşil olana dek RUNNING
    └── ProceedOnGreen(max_react_sec=5.0)        # yeşilde 5 sn içinde kalk → +40
```

**`Seg_PassengerStop`** (yolcu — +70/nokta):
```
Sequence
├── SafeDrive                         # durak noktasına yaklaş
├── StopVehicle
├── CheckStopAccuracy(point, tol=1.0) # 1 m içinde mi?
├── Dwell(min=15, max=20)             # ŞARTNAME: 15-20 sn bekleme
└── SignalPassengerEvent
```

**`Seg_Parking`** (+20 giriş, +80 park):
```
Sequence
├── SetMaxSpeed(çok yavaş)
├── SafeDrive                         # park_giris noktasından geç → +20
├── RecordParkEntryReached
├── FindParkingSlot(slot_pose)        # girişi açık + P-3a var + P-1 yok + önü boş
└── ExecuteParking(slot, time_limit=180)   # dik park, ≤3 dk
```

---

## 8. Şartname puanı → node eşlemesi (kontrol listesi)

Bu tabloyu KTR/sunumda "her kural için bir karşılığı var" diye gösterebilirsin.

| Şartname kuralı | Puan | Karşılayan node/alt-ağaç |
|---|---|---|
| Kırmızı ışıkta toleransta durma | +60 | `HandleTrafficLight` / `StopAtStopLine(5.0)` |
| Yeşilde 5 sn içinde kalkış | +40 | `ProceedOnGreen(5.0)` |
| Park bölgesi noktasından geçme | +20 | `Seg_Parking` / `RecordParkEntryReached` |
| Park görevini tamamlama (dik) | +80 | `ExecuteParking(180)` |
| Şerit ihlali yapmama | +50 | `FollowLaneSegment` (şeritte kalır) |
| Dinamik engele çarpmama | +50 | `SafetyReflexes` (dinamik dal) |
| Statik engelden sakınma | +50 | `SafetyReflexes` / `HandleStaticObstacle` |
| Trafik işaretlerine uyma | +50 | `ValidateTurn`, `Seg_Roundabout`, `HandleStopSign` |
| Görev noktasından geçme | +30/nokta | `RecordMissionPoint` |
| Doğru yolcu al/bırak (15-20 sn) | +70/nokta | `Seg_PassengerStop` / `Dwell(15,20)` |
| Tünelden geçme | +200 | `Seg_Tunnel` |
| Acil durdurma sonrası ≤2 m | (eleme) | `EmergencyStopRequested` → `StopVehicle` |
| Çarpma = diskalifiye | — | `SafetyReflexes` reaktif öncelik |

---

## 9. Custom node sözleşmesi (implementasyon kontratı)

Aşağıdaki node'ların **içini** takım yazacak (ROS2 action/condition). BT sadece bu arayüze güvenir. `segment_bt.xml` içindeki `TreeNodesModel` bu listeyi içerir (Groot'ta görünmesi için).

### Action node'ları
| Node | Girdi (port) | Çıktı | Görev |
|---|---|---|---|
| `LoadMission` | `geojson_file`, `tour` | `route`, `route_size`, `seg_index` | GeoJSON + graf → rota |
| `GetCurrentSegment` | `route`, `seg_index` | `seg_type`, `seg_goal`, `seg_meta` | Aktif segmenti çöz |
| `AdvanceSegment` | `seg_index` (in/out) | — | `seg_index++` |
| `SetMaxSpeed` | `speed` | — | Hız limiti ayarla |
| `FollowLaneSegment` | `goal`, `max_speed` | — | Şeritte segment sonuna sür (RUNNING→SUCCESS) |
| `StopVehicle` | — | — | Kontrollü tam dur |
| `WaitForClear` | `hazard` | — | Tehlike geçene dek RUNNING |
| `StopAtStopLine` | `tolerance` | — | Çizgide toleransta dur |
| `WaitForGreenLight` | `light_color` | — | Yeşil olana dek RUNNING |
| `ProceedOnGreen` | `light_color`, `max_react_sec` | — | Yeşilde hızlı kalkış |
| `StopAndProceed` | — | — | DUR tabelası: tam dur, sonra geç |
| `CalculateLaneChange` | `current_pose`, `target_lane` | `seg_goal` | Şerit değişim hedefi üret |
| `ReplanRoute` | `reason` | `route`, `route_size`, `seg_index` | Rotayı yeniden çıkar |
| `TurnHeadlights` | `state` | — | Far aç/kapat (tünel) |
| `YieldAtRoundabout` | — | — | Dönelde boşluk bekle |
| `ExecuteRoundabout` | `exit_node` | — | Doğru çıkıştan ayrıl |
| `Dwell` | `min_sec`, `max_sec` | — | Belirtilen süre bekle |
| `SignalPassengerEvent` | `event_type` | — | Yolcu al/bırak işaretle |
| `RecordMissionPoint` | `point`, `tolerance` | — | 1 m içinde geçişi say |
| `RecordParkEntryReached` | — | — | Park girişi geçildi |
| `FindParkingSlot` | — | `slot_pose` | Uygun slot bul |
| `ExecuteParking` | `slot`, `time_limit_sec` | — | Dik park manevrası |
| `BackUp`, `Spin` | (Nav2 portları) | — | Kurtarma |

### Condition node'ları
| Node | Girdi | Doğru (SUCCESS) anlamı |
|---|---|---|
| `HasMoreSegments` | `seg_index`, `route_size` | Hâlâ segment var |
| `IsSegmentType` | `seg_type`, `expected` | Tip eşleşti |
| `IsTrafficControlActive` | `tour` | Tur ≥ 2 (ışık/tabela aktif) |
| `EmergencyStopRequested` | — | Acil dur talebi var |
| `PedestrianAhead` | `detected` | Yaya var |
| `DynamicObstacleAhead` | `detected` | Dinamik engel var |
| `StaticObstacleInLane` | `detected` | Şeritte statik engel |
| `AvoidanceSpaceAvailable` | `free_lane` | Sakınma için yer var |
| `TrafficLightAhead` | `light_color` (out) | Işık görülüyor |
| `IsLightRed` | `color` | Işık kırmızı |
| `StopSignAhead` | `detected` | DUR tabelası var |
| `TurnConflictsWithSigns` | `planned_turn`, `detected_signs` | Planlanan dönüş yasak |
| `CheckStopAccuracy` | `point`, `tolerance` | 1 m içinde durdu |
| `IsStuck` | — | Araç sıkıştı |

---

## 10. Groot2'de açma ve sonraki adımlar

### Açma
1. Groot2'yi aç → **File → Open** → `segment_bt.xml`.
2. Dosya **BT.CPP v4** formatında (`BTCPP_format="4"`) ve `TreeNodesModel` bloğu gömülü; bu yüzden custom node'lar "unknown" hatası vermeden, port'larıyla görünür.
3. Sol panelde alt-ağaç listesinden (`Seg_LaneFollow`, `SafeDrive`, …) tek tek inceleyebilir, çift tıklayarak içine girebilirsin.

> Not: Nav2'da çalıştıracaksan, custom node'ları C++ tarafında `factory.registerNodeType<...>()` ile kaydedip `BT::writeTreeNodesModelXML(factory)` çıktısıyla `TreeNodesModel`'i otomatik üretmen daha sağlıklı; bu dosyadaki model elle yazıldığı için port tiplerini kod tarafıyla eşitlemeyi unutma.

### Takıma sorulacaklar (graf hazırlamak için şart)
1. Kavşak / dönel / tünel ağzı / durak / park girişi düğümlerinin **koordinatları** (harita frame). Graf bunsuz çıkmaz.
2. Her yolda **kaç şerit** var? (Segment sayısı buna bağlı.)
3. Dönel kavşağın **kaç çıkışı** var?
4. Tünel **tek mi çift yönlü** mü? (Opsiyonel +200 için risk değerlendirmesi.)
5. "Trafik soldan akar" → şerit numaralandırma yönünü netleştir (sağ/sol şerit tanımı).

### Yapılacaklar sırası
1. ✅ Bu mimariyi takıma onaylat (özellikle "Nav2'yı silme, soyutla" kararını).
2. `segment_map.yaml`'ı gerçek koordinatlarla doldur.
3. `LoadMission` + `GetCurrentSegment` + graf arama (en kısa yasal rota) yaz — **ilk kritik parça budur.**
4. `FollowLaneSegment`'in içini doldur (vision lane-follow veya Nav2 FollowPath).
5. `SafetyReflexes` node'larını gerçek sensör verisine bağla.
6. Puanlı node'ları sırayla tamamla: ışık → yolcu → park (puanın çoğu burada).
7. Simülasyonda (Gazebo/Unity) Tur 1 → Tur 2 → Tur 3 sırayla doğrula.

---

*Bu doküman `segment_bt.xml` ile birlikte teslim edilir. XML, buradaki tüm alt-ağaçları ve node sözleşmesini içerir.*
