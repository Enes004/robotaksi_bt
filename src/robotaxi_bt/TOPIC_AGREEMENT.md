# Topic Agreement

## bt

### istenilen
- `/perception/traffic_light` (`std_msgs/msg/String`) — Trafik ışığı durumu (RED, GREEN, YELLOW)
- `/perception/road_signs` (`std_msgs/msg/String`) — Algılanan trafik levhaları (STOP, PARK, SPEED_LIMIT)
- `/perception/obstacle` (`std_msgs/msg/Bool`) — Ön engel / yaya algılama bilgisi
- `/current_pose` (`geometry_msgs/msg/PoseStamped`) veya `/odom` (`nav_msgs/msg/Odometry`) — Aracın anlık konum verisi
- `/mission/waypoints` veya GeoJSON — Yarışma durak ve rota noktaları

### verilen
- `/vehicle/max_speed` (`std_msgs/msg/Float64`) — Hedef maksimum hız sınırı komutu
- `/vehicle/headlights` (`std_msgs/msg/Bool`) — Far aç/kapat komutu
- `/cmd_vel` (`geometry_msgs/msg/Twist`) — Acil durma / hız sıfırlama komutu

---

## localization

### istenilen

### verilen

---

## perception

### istenilen

### verilen

---

## nav2

### istenilen

### verilen

---

## controller

### istenilen

### verilen

---

## route_planner

### istenilen

### verilen
