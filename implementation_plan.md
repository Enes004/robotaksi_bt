# Segment-Tabanlı BT Mimarisine Geçiş Analizi

## Mevcut Durum vs Takım İsteği

### Senin Mevcut Mimari (Nav2-Bağımlı)
```
ReactiveFallback (her tick soldan sağa taranır)
  P1: DynamicObstacle → Wait
  P2: StaticObstacle → LaneChange / Wait
  P3: TrafficLight → Wait  
  P4: TrafficSigns (10 tabela) → özel eylemler
  P5: GoalReached → PassengerAction
  P6: NavigateThroughPoses → Nav2 global+local planner
```
**Nav2'ye bağımlılık:** Global planner rota çizer, local planner takip eder, BT sadece interrupt/override yapar.

### Takımın İstediği (Segment-Tabanlı)
- Haritayı fiziksel segmentlere böl
- Her segment için hangi davranışın çalışacağını BT belirlesin
- Nav2'nin global/local planner'ına "güvenmeyelim"
- Tekrarlayan yol parçaları için aynı kodu kullan (DRY prensibi)

---

## Harita Analizi — Segmentler

Attığın haritayı inceledim. Parkur şu elemanlardan oluşuyor:

```
┌──────────────────────────────────────────┐
│  ┌───┐ ┌───┐ ┌───┐ ┌───┐                │
│  │   │ │TÜN│ │   │ │   │   [PARK]        │
│  │   │ │EL │ │   │ │   │   ═══════       │
│  └───┘ └───┘ └───┘ └───┘                 │
│  ┌───┐        ┌───┐ ┌───┐                │
│  │   │        │   │ │   │                 │
│  │   │   (●)  │   │ │   │  (●)=Dönel K.  │
│  └───┘  ROND  └───┘ └───┘                │
│  ┌───┐ ┌───┐ ┌───┐ ┌───┐                │
│  │   │ │   │ │   │ │   │                 │
│  └───┘ └───┘ └───┘ └───┘                │
│  D         D         D                   │
└──────────────────────────────────────────┘
```

**Gözlemler:**
1. **Grid yapı** — 3x4 blok, aralarında yollar (Manhattan tarzı)
2. **Dönel kavşak** (roundabout) — ortada
3. **Tünel** — sol üst blokların arasında
4. **Park alanı** — sağ üst köşe (dik park yerleri)
5. **D işaretleri** — alt kenar ve kenarlar (durak/başlangıç noktaları?)
6. **Tekrarlayan yapı** — Yatay ve dikey düz yol segmentleri birbirinin aynısı

---

## Önerim: Hibrit Segment Mimarisi

Takımın "segment" fikri doğru ama "Nav2'yı tamamen devre dışı bırak" fikri **yanlış ve tehlikeli**. Sebebi:

> [!WARNING]  
> Nav2 olmadan her segmentte kendi path planning + obstacle avoidance + costmap kodunuzu yazmanız gerekir. Bu, yarışmaya kadar tamamlanamayacak kadar büyük bir iştir.

**Doğru yaklaşım: Nav2'yı kısa mesafeli "segment şoförü" olarak kullan, ama rotayı BT yönetsin.**

### Segment Tipleri (Haritadan Çıkarılan)

```yaml
segment_types:
  STRAIGHT:        # Düz yol (iki kavşak arası)
    nav2: true     # Nav2 NavigateToPose ile git
    speed: normal
    
  ROUNDABOUT:      # Dönel kavşak girişi → çıkışı  
    nav2: true     # Nav2 ama özel waypoint zinciri
    speed: slow
    
  TUNNEL:          # Tünel girişi → çıkışı
    nav2: true
    speed: slow
    lights: on     # Far aç
    
  INTERSECTION:    # Kavşak (tabela/ışık kontrolü)
    nav2: true
    check_signs: true
    check_lights: true
    
  PARKING_APPROACH: # Park alanına yaklaşma
    nav2: true
    speed: slow
    
  PARKING_EXECUTE:  # Park manevrası
    nav2: false    # Manuel cmd_vel ile park
    
  PASSENGER_STOP:   # Yolcu indirme/bindirme
    nav2: false    # Dur, bekle, devam et
```

### Harita Üzerindeki Fiziksel Segmentler

```
Segment ID  | Tip            | Başlangıç    | Bitiş        | Tekrar Eden?
------------|----------------|--------------|--------------|-------------
S_H1        | STRAIGHT       | Kavşak_A     | Kavşak_B     | ✅ (yatay düz)
S_H2        | STRAIGHT       | Kavşak_B     | Kavşak_C     | ✅ (yatay düz)
S_H3        | STRAIGHT       | Kavşak_C     | Kavşak_D     | ✅ (yatay düz)
S_V1        | STRAIGHT       | Kavşak_A     | Kavşak_E     | ✅ (dikey düz)
S_V2        | STRAIGHT       | Kavşak_E     | Kavşak_I     | ✅ (dikey düz)
...         | ...            | ...          | ...          |
S_TUN       | TUNNEL         | Tünel_Giriş  | Tünel_Çıkış  | ❌ (tek)
S_RND       | ROUNDABOUT     | RND_Giriş    | RND_Çıkış    | ❌ (tek)
S_PARK      | PARKING_APPROACH| Park_Giriş  | Park_Alanı   | ❌ (tek)
S_PARK_EXE  | PARKING_EXECUTE | Park_Slotu  | Park_Bitti   | ❌ (tek)
```

> [!IMPORTANT]
> **Tekrarlayan segmentler:** Düz yol segmentleri (`STRAIGHT`) hep aynı SubTree'yi kullanır. Kavşak segmentleri (`INTERSECTION`) hep aynı SubTree'yi kullanır. Bu, takımın "tekrar için ayrı kod yazmaya gerek yok" isteğini karşılar.

---

## Yeni BT Mimarisi

### Kök Yapı

```xml
<root main_tree_to_execute="MainTree">

  <BehaviorTree ID="MainTree">
    <Sequence>
      <!-- 1. Görev dosyasını oku, segment listesini oluştur -->
      <Action ID="InitMission" 
              waypoints_file="{waypoints_file}"
              segment_list="{segment_list}"
              segment_index="{seg_idx}"/>
      
      <!-- 2. Segment döngüsü -->
      <Repeat num_cycles="-1">
        <Sequence>
          <!-- Tüm segmentler bitti mi? -->
          <Condition ID="HasMoreSegments" 
                     seg_idx="{seg_idx}" 
                     seg_list="{segment_list}"/>
          
          <!-- Aktif segmenti al -->
          <Action ID="GetCurrentSegment"
                  seg_idx="{seg_idx}"
                  seg_list="{segment_list}"
                  current_seg="{current_seg}"
                  seg_type="{seg_type}"
                  seg_goal="{seg_goal}"/>
          
          <!-- Segment Router (switch-case) -->
          <SubTree ID="SegmentRouter" 
                   seg_type="{seg_type}"
                   seg_goal="{seg_goal}"
                   current_seg="{current_seg}"/>
          
          <!-- Segment tamamlandı, index++ -->
          <Action ID="IncrementSegment" seg_idx="{seg_idx}"/>
        </Sequence>
      </Repeat>
    </Sequence>
  </BehaviorTree>
```

### Segment Router (Switch-Case)

```xml
  <BehaviorTree ID="SegmentRouter">
    <Fallback>

      <!-- STRAIGHT: Düz yol navigasyonu -->
      <Sequence>
        <Condition ID="CheckSegType" seg_type="{seg_type}" expected="STRAIGHT"/>
        <SubTree ID="ST_StraightDrive" goal="{seg_goal}"/>
      </Sequence>

      <!-- INTERSECTION: Kavşak (tabela/ışık kontrolü) -->
      <Sequence>
        <Condition ID="CheckSegType" seg_type="{seg_type}" expected="INTERSECTION"/>
        <SubTree ID="ST_Intersection" goal="{seg_goal}"/>
      </Sequence>

      <!-- ROUNDABOUT: Dönel kavşak -->
      <Sequence>
        <Condition ID="CheckSegType" seg_type="{seg_type}" expected="ROUNDABOUT"/>
        <SubTree ID="ST_Roundabout" goal="{seg_goal}"/>
      </Sequence>

      <!-- TUNNEL: Tünel geçişi -->
      <Sequence>
        <Condition ID="CheckSegType" seg_type="{seg_type}" expected="TUNNEL"/>
        <SubTree ID="ST_Tunnel" goal="{seg_goal}"/>
      </Sequence>

      <!-- PASSENGER_STOP: Yolcu al/bırak -->
      <Sequence>
        <Condition ID="CheckSegType" seg_type="{seg_type}" expected="PASSENGER_STOP"/>
        <SubTree ID="ST_PassengerStop" goal="{seg_goal}"/>
      </Sequence>

      <!-- PARKING_APPROACH + EXECUTE -->
      <Sequence>
        <Condition ID="CheckSegType" seg_type="{seg_type}" expected="PARKING"/>
        <SubTree ID="ST_Parking" goal="{seg_goal}"/>
      </Sequence>

    </Fallback>
  </BehaviorTree>
```

### Örnek SubTree: ST_StraightDrive (Tekrar Kullanılır!)

```xml
  <BehaviorTree ID="ST_StraightDrive">
    <!-- Bu SubTree HER düz yol segmentinde çalışır -->
    <RecoveryNode number_of_retries="3">
      <ReactiveFallback>
        
        <!-- P1: Dinamik engel → bekle -->
        <Sequence>
          <Condition ID="DynamicObstacleDetected"/>
          <Action ID="WaitForClear"/>
        </Sequence>

        <!-- P2: Statik engel → sakın -->
        <Sequence>
          <Condition ID="StaticObstacleDetected"/>
          <Action ID="AvoidStaticObstacle"/>
        </Sequence>

        <!-- P3: Normal navigasyon (Nav2 KISA MESAFE) -->
        <Action ID="NavigateToPose" goal="{goal}"/>

      </ReactiveFallback>
      
      <!-- Recovery -->
      <Sequence>
        <Action ID="BackUp" backup_dist="0.3"/>
        <Action ID="Spin" spin_dist="1.57"/>
      </Sequence>
    </RecoveryNode>
  </BehaviorTree>
```

### Örnek SubTree: ST_Intersection (Kavşaklarda)

```xml
  <BehaviorTree ID="ST_Intersection">
    <Sequence>
      <!-- Hız düşür -->
      <Action ID="SetMaxSpeed" speed="0.3"/>
      
      <!-- Işık var mı kontrol et -->
      <Fallback>
        <Sequence>
          <Condition ID="TrafficLightDetected" color="{light_color}"/>
          <SubTree ID="HandleTrafficLight" color="{light_color}"/>
        </Sequence>
        <AlwaysSuccess/> <!-- Işık yoksa geç -->
      </Fallback>
      
      <!-- Tabela kontrol et -->
      <Fallback>
        <Sequence>
          <Condition ID="TrafficSignDetected" sign="{sign_type}"/>
          <SubTree ID="HandleTrafficSign" sign="{sign_type}"/>
        </Sequence>
        <AlwaysSuccess/>
      </Fallback>
      
      <!-- Kavşaktan geç -->
      <Action ID="NavigateToPose" goal="{goal}"/>
      
      <!-- Hız normale dön -->
      <Action ID="SetMaxSpeed" speed="0.5"/>
    </Sequence>
  </BehaviorTree>
```

---

## Segment Haritası (YAML Config)

Haritayı bir kez tanımla, tüm turlar için kullan:

```yaml
# segment_map.yaml — Parkurun fiziksel yapısı
# Kavşak noktaları (map frame koordinatları)
nodes:
  N_START:  {x: 0.0,  y: 0.0,  name: "Başlangıç"}
  N_A:      {x: 5.0,  y: 0.0,  name: "Kavşak A"}
  N_B:      {x: 10.0, y: 0.0,  name: "Kavşak B"}
  N_C:      {x: 15.0, y: 0.0,  name: "Kavşak C"}
  # ... tüm kavşak noktaları

# Segment tanımları (kenarlar)
edges:
  - id: E_AB
    from: N_A
    to: N_B
    type: STRAIGHT
    length: 5.0
    bidirectional: true   # İki yönlü kullanılabilir
    
  - id: E_TUN
    from: N_TUN_IN
    to: N_TUN_OUT
    type: TUNNEL
    length: 8.0
    bidirectional: false  # Tek yön
    
  - id: E_RND
    from: N_RND_IN
    to: N_RND_OUT
    type: ROUNDABOUT
    waypoints: [wp1, wp2, wp3]  # İç waypoint zinciri
    
  - id: E_PARK
    from: N_PARK_ENTRY
    to: N_PARK_AREA
    type: PARKING
```

**Runtime'da:** GeoJSON'dan gelen görev noktaları + bu segment haritası → `SegmentPlanner` en kısa/uygun rota üzerindeki segment sırasını çıkarır.

---

## Kritik Avantajlar

| Özellik | Eski (Senin) | Yeni (Segment) |
|---------|-------------|-----------------|
| Rota kaynağı | Nav2 global planner | BT segment zinciri |
| Nav2 rolü | Tam kontrol | Segment içi kısa mesafe şoförü |
| Tekrar eden kod | Her yol ayrı düşünülür | Aynı SubTree tekrar kullanılır |
| Tünel/Kavşak | Reaktif (tespit edince) | Proaktif (segment tipinden bilir) |
| Debug | "Nav2 kayboldu" | "Segment 7'de takıldı" |
| Tabela yönetimi | Her yerde dinle | Sadece INTERSECTION segmentlerinde |

---

## Takıma Söylenecekler

> [!IMPORTANT]
> **Takıma net söyle:**
> 1. ✅ Haritayı segment segment böleriz — her kavşak arası bir segment
> 2. ✅ Tekrarlayan segmentler aynı SubTree'yi kullanır (kod tekrarı yok)
> 3. ✅ Rotayı BT yönetir, Nav2 sadece "şu noktaya git" der
> 4. ⚠️ Nav2'yı tamamen kaldırmak **yanlış** — obstacle avoidance, costmap, local planning hâlâ gerekli
> 5. ✅ Her segment tipi için özel davranış tanımlanır (tünel, kavşak, park, düz yol)

## Açık Sorular

> [!IMPORTANT]
> **Takıma sor:**
> 1. Haritanın koordinatları belli mi? Kavşak noktalarının (x,y) değerleri lazım
> 2. Segment haritasını kim çıkaracak? (Harita ekibinden koordinatlar lazım)
> 3. Tünel tek yönlü mü çift yönlü mü?
> 4. Dönel kavşağın kaç çıkışı var?
> 5. "D" işaretleri tam olarak ne? (Durak mı, başlangıç noktası mı?)

## Sonraki Adımlar

1. **Takımla anlaş** → Bu hibrit yaklaşımı kabul ettir
2. **Segment haritasını çıkar** → Kavşak koordinatları + kenar tanımları
3. **SegmentPlanner yazılır** → GeoJSON + segment_map → segment dizisi
4. **SubTree'ler yazılır** → 6-7 adet (STRAIGHT, INTERSECTION, ROUNDABOUT, TUNNEL, PASSENGER, PARKING, INIT)
5. **Custom BT node'ları** → CheckSegType, GetCurrentSegment, IncrementSegment, SegmentPlanner
