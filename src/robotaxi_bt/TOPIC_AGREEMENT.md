# TeamIMU-AV — Robotaxi ROS 2 Topic & Interface Agreement

Bu doküman, **Robotaxi Behavior Tree (BT)** modülü ile **Algı (Perception)**, **Sürüş Kontrol (Control)**, **Navigasyon (Nav2)** ve **Planlama (Planning)** ekipleri arasındaki ROS 2 topic, mesaj türleri ve interface anlaşmasını içermektedir.

---

## 🛰️ Published Topics (Behavior Tree Tarafından Yayınlananlar)

BT modülü tarafından araç kontrolü ve durum bilgilendirmesi için dışarı yayınlanan topic'ler:

| Topic Adı | Mesaj Türü (`std_msgs`/`geometry_msgs`) | Açıklama |
| :--- | :--- | :--- |
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Aracı acil durdurma veya hız sıfırlama komutları |
| `/vehicle/max_speed` | `std_msgs/msg/Float64` | Ağacın dinamik olarak belirlediği maks. hız hedefi (m/s) |
| `/vehicle/headlights` | `std_msgs/msg/Bool` | Far aç/kapat komutu (`true`: açık [Tünel modu], `false`: kapalı) |
| `/bt/current_state` | `std_msgs/msg/String` | Behavior Tree'nin anlık durumu ve aktif görevi |

---

## 📥 Subscribed Topics (Behavior Tree Tarafından Dinlenenler)

BT modülünün karar verme mekanizmasında kullandığı sensör ve algı verileri:

| Topic Adı | Mesaj Türü | Hedef BT Node | Açıklama |
| :--- | :--- | :--- | :--- |
| `/perception/traffic_light` | `std_msgs/msg/String` veya Custom | `IsTrafficLightRed` | Trafik ışığı durumu (`RED`, `GREEN`, `YELLOW`) |
| `/perception/road_signs` | `std_msgs/msg/String` | `CheckSign` | Algılanan trafik levhaları (`STOP`, `PARK`, `SPEED_LIMIT`, vb.) |
| `/perception/obstacle` | `std_msgs/msg/Bool` | `IsPathClear` | Ön engel/yaya tespiti |
| `/odom` veya `/current_pose` | `geometry_msgs/msg/PoseStamped` | `GetCurrentSegment`, `CheckStopAccuracy` | Aracın anlık konum bilgisi |

---

## 🎯 Nav2 Action Interfaces (Navigasyon Entegrasyonu)

BT modülünün Nav2 haritası ve yol planlayıcı ile haberleşme arayüzleri:

| Action Server | Action Türü | BT Node | Açıklama |
| :--- | :--- | :--- | :--- |
| `navigate_to_pose` | `nav2_msgs/action/NavigateToPose` | `ExecuteSegment` | Belirtilen hedef segmente/noktaya otonom sürüş |
| `navigate_through_poses` | `nav2_msgs/action/NavigateThroughPoses` | `FollowPath` | Çoklu waypoint takip komutu |

---

## 📌 Şartname Uyum Notları (Teknofest 2026)

1. **Tünel Segmenti:** Tünel algılandığında veya harita segmentine girildiğinde `/vehicle/headlights` -> `true` basılır. (Ceza puanı engelleme)
2. **Durak Beklemeleri:** `Dwell` node'u yolcu indirme/bindirme noktalarında varsayılan **15-20 saniye** sayacı çalıştırır.
3. **Durma Hassasiyeti:** Durak ve park alanlarında duruş doğrulama `CheckStopAccuracy` node'u üzerinden `/odom` verisine göre denetlenir.

---
*Son Güncelleme: 2026-08-06*
