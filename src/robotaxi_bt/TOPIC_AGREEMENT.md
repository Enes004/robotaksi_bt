# Topic Agreement

## bt

### istenilen
- `/perception/traffic_light` (`std_msgs/msg/String`) — Trafik ışığı durumu (`RED`, `GREEN`, `YELLOW`)
- `/perception/road_signs` (`std_msgs/msg/String`) — Algılanan trafik levhaları (`STOP`, `PARK`, `SPEED_LIMIT`, vb.)
- `/perception/obstacle` (`std_msgs/msg/Bool`) — Ön engel / yaya tespiti
- `/odom` veya `/current_pose` (`geometry_msgs/msg/PoseStamped`) — Aracın anlık konum bilgisi

### verilen
- `/cmd_vel` (`geometry_msgs/msg/Twist`) — Aracı acil durdurma / hız sıfırlama komutu
- `/vehicle/max_speed` (`std_msgs/msg/Float64`) — Ağacın belirlediği maks. hız hedefi (m/s)
- `/vehicle/headlights` (`std_msgs/msg/Bool`) — Far aç/kapat komutu (`true`: açık [Tünel modu], `false`: kapalı)
- `/bt/current_state` (`std_msgs/msg/String`) — BT anlık durum ve görev bilgisi

---

## localization

### istenilen
- 

### verilen
- `/odom` veya `/current_pose` (`geometry_msgs/msg/PoseStamped`) — Konum verisi (BT ve Nav2 tarafından kullanılır)

---

## perception

### istenilen
- 

### verilen
- `/perception/traffic_light` (`std_msgs/msg/String`)
- `/perception/road_signs` (`std_msgs/msg/String`)
- `/perception/obstacle` (`std_msgs/msg/Bool`)

---

## nav2

### istenilen
- `/odom` (`geometry_msgs/msg/PoseStamped`)

### verilen
- `navigate_to_pose` Action Server (`nav2_msgs/action/NavigateToPose`)
- `navigate_through_poses` Action Server (`nav2_msgs/action/NavigateThroughPoses`)

---

## controller

### istenilen
- `/vehicle/max_speed` (`std_msgs/msg/Float64`)
- `/cmd_vel` (`geometry_msgs/msg/Twist`)

### verilen
- 

---

## route_planner

### istenilen
- 

### verilen
- `mission.json` / GeoJSON rotası veya hedef waypoint listesi
