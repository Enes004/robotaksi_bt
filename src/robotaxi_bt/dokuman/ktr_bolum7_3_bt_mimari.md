# 7.3. Nav2 Uyumlu Davranış Ağacı (Behavior Tree) Mimarisi ve Rota Planlama

Otonom sürüş sistemlerinde rota planlama ve karar alma mekanizmaları, yalnızca bir başlangıç ve
bitiş noktası arasında yol oluşturmakla sınırlı kalmayan; çevresel faktörlere, anlık sensör
verilerine ve görev hedeflerine bağlı olarak milisaniyeler içinde değişebilen dinamik bir
süreçtir. Bu bağlamda, klasik Sonlu Durum Makineleri (FSM) yerine ROS 2 ekosistemi içerisinde
otonom navigasyon kabiliyeti sunan, güçlü, esnek ve modüler Davranış Ağacı (Behavior Tree - BT)
mimarisi ve NAV2 çerçevesi kullanılmıştır.

Sistemimizde NAV2; önceden hazırlanmış LeGO-LOAM haritası, lokalizasyon modülleri ve waypoint
temelli rota takibi ile entegre çalışmaktadır. Aracın asıl karar merkezi ise Groot2 ile tasarlanan
XML tabanlı özel karar alma düğümleri (TreeNodesModel) üzerinden yönetilmektedir.

**Waypoint Tabanlı Rota Tanımı (NavigateThroughPoses):** Planlama sürecinde görev öncesinde
belirlenmiş waypoint (geçiş) noktaları sisteme dizi olarak verilir. Davranış ağacımızdaki
NavigateThroughPoses eylem düğümü (Action Node), aracın bu noktalar (Başlangıç → gorev_1 →
gorev_2 → park_giris) arasında küresel bir rota (Global Path) çizmesini sağlar.

**Yerel Planlama ve Engel Kaçınma:** Küresel planlayıcı tarafından belirlenen rota takip
edilirken, aracın çevresel koşullara uyum sağlayabilmesi için yerel planlama (Local Planner)
aşaması devreye girer. Engel tespiti ve kaçınma işlemleri NAV2'nin katmanlı maliyet haritası
(costmap) altyapısı ile BT düğümleri üzerinden eşzamanlı yürütülür. LiDAR ve kamera verileri
haritaya işlenirken, BT üzerindeki koşul düğümlerimiz (Condition Nodes) bu güncellemeleri anlık
olarak dinler.

**Trafik Kurallarına Dinamik Adaptasyon:** Klasik sürüşün aksine, sistemimiz her bir trafik
tabelası ve ışığı için özel BT düğümleri barındırır. Yeni bir kural algılandığında (örneğin DUR
tabelası veya Yaya Geçidi), sistem Nav2'nin hız ve yön komutlarına mikro düzeyde müdahale eder
veya gerekirse yeni bir küresel planlama (Replanning) tetikleyerek alternatif bir güzergah
oluşturur.

---

## 7.3.1. Karar Ağacı

Karar ağacı, otonom aracın tüm sürüş senaryolarını hiyerarşik ve modüler şekilde yönetmek
amacıyla tasarlanmıştır. Geçmiş yıllarda kullanılan Sonlu Durum Makinelerinin (FSM) aksine, bu
yıl sistemimiz ROS 2 Nav2 ekosistemiyle tam entegre çalışan bir Davranış Ağacı (Behavior Tree -
BT) mimarisi kullanmaktadır. Araç, Groot2 ile tasarlanan XML tabanlı özel karar alma düğümleri
(Condition ve Action Node'ları) sayesinde sensör verilerini eşzamanlı işleyerek anlık reaksiyonlar
gösterir.

BT'nin temel çalışma mantığı **öncelik tabanlı reaktif döngü** üzerine kuruludur. Her tick'te
(yaklaşık 10 Hz) sistemin durumu soldan sağa değerlendirilir; ilk SUCCESS dönen dal o döngüde
kazanır ve diğer dallar atlanır. Bu sayede güvenlik kuralları her zaman navigasyondan önce
işlenir.

### Öncelik Sırası (P1 → P6)

| Öncelik | Düğüm | Tetikleyici | Eylem |
|---------|-------|------------|-------|
| **P1** | DynamicObstacleCondition | Önde hareketli engel | Araç bekler (Wait) |
| **P2** | StaticObstacleCondition | Önde statik engel | Şerit değiştir veya çift yolda bekle |
| **P3** | TrafficLightCondition | Kırmızı ışık | Araç bekler (Wait) |
| **P4** | TrafficSigns Fallback | 10 farklı tabela | Tabela tipine özel eylem |
| **P5** | GoalReachedCondition | Yolcu noktasına ulaşıldı | PassengerAction (15-20 sn dur) |
| **P6** | NavigateThroughPoses | Yukarıdakilerin hiçbiri yoksa | Normal navigasyon |

---

### 7.3.1.1. Boşta Durum

Boşta durumu, aracın sistemleri başlatıldıktan sonra operasyonel hedeflerini almayı beklediği
aşamadır. Mimarimizde bu durum, **SingleTrigger** dekoratörü kullanılarak optimize edilmiştir.
Görev koordinatlarını içeren GeoJSON dosyasından üretilen `waypoints.txt` dosyası diskten yalnızca
bir kez okunur; böylece gereksiz I/O operasyonlarının önüne geçilerek sistemin sürüşe en düşük
gecikmeyle hazır olması sağlanır.

Yarışma öncesinde hakem tarafından sağlanan GeoJSON dosyası, `geojson_parser.py` Python betiği
aracılığıyla `waypoints.txt` formatına dönüştürülür. Bu dosya; her waypoint için isim, tip, (x,y)
koordinatı ve quaternion yön bilgisini içerir. `InitMissionAction` C++ düğümü bu dosyayı okuyarak
blackboard'a yazar:

- **`{goals}`** → Sıralı waypoint listesi: [start, gorev_1, gorev_2, park_giris]
- **`{current_goal_type}`** → İlk hedefin tipi ("gorev")
- **`{passenger_served}`** → Başlangıçta boş string (""), servis yapıldıkça güncellenir

---

### 7.3.1.2. Sürüş Durumu

Aracın hedeflerine doğru hareket ettiği ana operasyon modudur. Sistemimizde bu eylem, Nav2 tabanlı
**NavigateThroughPoses** eylem düğümü ile kontrol edilir. Araç, başlangıç noktasından başlayarak,
sırasıyla yolcu bindirme/indirme noktalarına (gorev_1, gorev_2) ve son olarak otopark girişine
(park_giris) doğru yönlendirilir.

Aracın o anki hedef tipi, BT içerisindeki **`{current_goal_type}`** blackboard değişkeni
üzerinden anlık olarak takip edilir. Bu değer `InitMissionAction` tarafından başlangıçta yazılır;
her yolcu servisinin tamamlanmasının ardından `PassengerAction` düğümü tarafından güncellenir
(gorev_1 → gorev_2 → park_giris sırasıyla).

---

### 7.3.1.3. Düz Yol

Araç, düz bir güzergahta herhangi bir engel veya trafik işaretiyle karşılaşmadığı sürece Nav2'nin
yerel planlayıcısı (Local Planner) devrededir. Şerit çizgileri maliyet haritasında (costmap)
kısıt olarak tutulur.

Bu esnada BT'nin ReactiveFallback yapısı ~10 Hz hızında tüm P1-P5 koşullarını sürekli kontrol
ederken, hiçbiri tetiklenmezse NavigateThroughPoses (P6) aktif kalır ve araç optimum hız
profilini koruyarak kesintisiz ilerler.

**Şerit Düzenleme (LaneRegulation):** Bazı trafik tabelaları (belirli yön tabelaları) aracın
şerit konumunu düzenlemesini zorunlu kılar. Bu durumda **LaneRegulationCondition** düğümü
`{sign_lane_reg}` blackboard anahtarını dinler; tabela algılandığında **LaneRegulationAction**
devreye girerek geçici bir hedef (`{temp_goal}`) hesaplar ve **DriveToGoal** eylemi bu geçici
hedefe yönelim sağlar. İşlem tamamlandığında NavigateThroughPoses orijinal rotaya devam eder.

---

### 7.3.1.4. Trafik Lambaları (ve Levhaları)

Trafik lambaları ve çevresel levhalar, YOLO algı modelinden gelen verilerle Blackboard'a yansıtılır
ve özel Condition (Koşul) düğümleriyle işlenir:

**Trafik Lambası (P3 Önceliği):** `TrafficLightCondition` düğümü `{detected_light}` anahtarını
dinler. Algılanan değer:
- **"RED"** → Araç `Wait` komutuyla tam durdurulur; kırmızı süresince beklenir (şartname +60 puan)
- **"GREEN"** → Condition FAILURE döner, araç harekete devam eder (5 sn içinde → şartname +40 puan)
- **Sarı ışık** → Sistemimizde sarı ışık için BT'de ayrı bir dal tanımlı değildir; bu durum
  kamera algı pipeline'ında, ışığa olan mesafe ve araç hızı parametreleri göz önünde bulundurularak
  değerlendirilen bir geçiş durumu olarak ele alınır ve güvenlik marjı çerçevesinde yönetilir.

**DUR Tabelası (TT-2):** `StopSignCondition` aktifleştiğinde araç `StopSignAction` ile tam duruş
gerçekleştirir. Çift durmayı engellemek için C++ içinde durum bayrağı (flag) kullanılır ve araç
yasal bekleme süresini (3 sn) tamamlar.

**Tünel Geçişi (B-49a):** `TunnelCondition` tetiklendiğinde `TunnelAction` devreye girer.
Bu eylem; önce `static bool tunnel_used` bayrağını kontrol eder (tünel yalnızca tek seferlik
+200 ek puan sağlar, şartname gereği), ardından Nav2 maksimum hız parametresini tünel içi güvenlik
şartlarına göre düşürür, geçişi tamamlar, çıkışta hızı normale döndürür ve bayrağı işaretler.

---

### 7.3.1.5. Dönüşler

Mecburi yön veya dönüş yasağı tabelaları, aracın topolojik hedeflerini dinamik olarak değiştirir.

**Dönüş Yasağı (TT-26a/b):** `ForbiddenTurnCondition` algılandığında, `ForbiddenTurnAction`
yasak dönüş yönünü maliyet haritasına (costmap) "lethal" (geçilemez) kısıt olarak ekler. Ardından
Nav2 global planlayıcısı mevcut `{current_vertex}`'ten alternatif bir `{next_vertex}` için yeniden
planlama (replanning) yapar.

**Mecburi Yön (TT-35a-h, 7 varyant):** `MandatoryTurnCondition` algılandığında
`MandatoryTurnAction` devreye girer. Tabela kodu parse edilir ("TT-35X:yön" formatında), belirlenen
yöne göre Nav2 yeniden planlanır.

**Girilmez Yol (TT-4):** `NoEntryCondition` aktifleştiğinde `NoEntryAction` hedef yolu costmap
üzerinde kapatır ve alternatif güzergah hesaplanır.

---

### 7.3.1.6. Durak Durumu

Araç B-22 Durak tabelası ile karşılaştığında **BusStopCondition** aktifleşir ve araç durak cebine
kontrollü bir şekilde yanaşır (`BusStopAction`).

Bu nokta aynı zamanda bir görev (yolcu) noktası ise, BT'deki öncelik yapısı şu şekilde çalışır:

- **Tick N:** P4 (Tabela Fallback) → `BusStopAction` çalışır (B-22 kuralı uygulanır)
- **Tick N+1:** Tabela artık görüş alanında değil → P4 FAILURE → P5 (GoalReachedCondition)
  SUCCESS → `PassengerAction` çalışır (15-20 sn yolcu servisi)

Bu yapı sayesinde hem durak kuralı hem de yolcu servisi doğru sırayla ve eksiksiz uygulanır;
her iki puan da alınır.

---

### 7.3.1.7. Park Durumu

P-3a Park Yeri tabelası algılandığında **ParkSignCondition** tetiklenir ve **ParkingAction** eylemi
başlatılır. Lidar ve odometri sensörlerinden alınan lokalizasyon verileri kullanılarak, aracın
şerit çizgileri içerisinde kalacak şekilde otopark alanına girmesi sağlanır. Otopark girişi
(park_giris), aynı zamanda görev zincirinin son halkası olarak değerlendirilir.

Puanlama (şartname):
- Başarılı park (şerit içinde) → **+80 puan**
- Eksik park (bir tekerlek dışarıda) → **+20 puan**
- Uygun olmayan yere park → **-20 puan**

---

### 7.3.1.8. Engelden Kaçınma

Yol üzerindeki engeller Lidar üzerinden alınan `{detected_obstacle}` ve `{is_static_obstacle}`
verileriyle **iki ayrı öncelik katmanında** yönetilir:

**P1 — Dinamik Engeller:** `DynamicObstacleCondition` hareketli bir engel tespit ettiğinde BT'nin
en yüksek öncelikli (P1) dalı SUCCESS döner. Bu, tüm diğer işlemleri (navigasyon, tabela
tepkileri dahil) geçici olarak askıya alır. Araç `Wait` eylemiyle yerinde bekler; güvenli takip
mesafesini korur. Engel ortadan kalktığında Condition FAILURE döner ve sistem normal akışa döner.

**P2 — Statik Engeller ve Bypass:** `StaticObstacleCondition` sabit bir engel tespit ettiğinde
P2 dalı devreye girer. Bu aşamada iki senaryo değerlendirilir:

1. **Çift yol senaryosu (`TwoWayRoadCondition`):** Yol iki yönlüyse karşı şeridi beklemek gerekir;
   `Wait` eylemiyle beklenir, karşıdan araç geçince yol serbest kalır.

2. **Şerit değiştirme senaryosu:** Tek yönlü yolda `CalculateLaneChangeAction` mevcut konum
   (`{current_pose}`) üzerinden geçici bir ara hedef (`{temp_goal}`) hesaplar. Ardından
   `DriveToGoal` eylemi bu geçici hedefe yönelir. Engel aşıldıktan sonra `NavigateThroughPoses`
   orijinal rotaya devam eder.

**TT-36a/b (Sağdan/Soldan Geçiniz):** Bu tabelalar P4 (Tabela Fallback) katmanında işlenir.
`PassByCondition` aktifleştiğinde `PassByAction`, engelin geçilmemesi gereken tarafına costmap
üzerinde geçici "lethal" bölge ekler; Nav2 planlayıcısı güvenli rotayı otomatik hesaplar.

---

### 7.3.2. Anormal Durumların Karar Ağacında İşlenmesi

Aracın beklenmedik durumlarda görevine devam edebilmesi için Nav2 Progress Checker tabanlı
**IsStuck** (Mahsur Kalma) koşulu izlenmektedir. Bu mekanizma `nav2_params.yaml` içindeki
`progress_checker_plugin` parametreleriyle yapılandırılmıştır:

- `required_movement_radius: 0.5 m` — belirtilen sürede bu mesafe kaytedilmezse mahsur sayılır
- `movement_time_allowance: 10.0 sn` — değerlendirme penceresi

Araç belirli bir süre içinde hedeflenen hareketi gerçekleştiremezse **RecoveryNode** devreye girer.
Sırasıyla **BackUp** (0.3 m, 0.05 m/s ile geri çekilme) ve **Spin** (1.57 rad ≈ 90°, yerinde
dönerek yeni açı arama) eylemleriyle aracın kilitlendiği noktadan otonom olarak kurtulması
sağlanır. Bu kurtarma manevrası, yolcu durakları arasındaki dar alanlarda maksimum **3 kez**
denenecek şekilde (`number_of_retries="3"`) optimize edilmiştir.

---

## Notlar (İç Kullanım — KTR'ye Eklenmeyecek)

- `general_driving_bt.xml` → Geliştirme aşaması dosyası; `round3_bt_final.xml` ile tam senkronize
  değil. KTR referansı olarak `round3_bt_final.xml` kullanılmalıdır.
- `PassengerAction` C++ implementasyonu tamamlanmalı: `{current_goal_type}` ve
  `{passenger_served}` blackboard güncellemeleri zorunlu.
- Sarı ışık için ileride ayrı bir BT Condition dalı eklenebilir.
