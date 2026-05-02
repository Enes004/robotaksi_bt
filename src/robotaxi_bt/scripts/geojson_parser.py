#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
╔══════════════════════════════════════════════════════════════════╗
║  TEKNOFEST 2026 Robotaksi — GeoJSON Görev Dosyası Ayrıştırıcı  ║
║  Yarışma günü GEOJSON → YAML / Okunabilir Çıktı / Nav2 Format  ║
╚══════════════════════════════════════════════════════════════════╝

Kullanım:
  # Okunabilir özet (terminal'e bas)
  python3 geojson_parser.py gorev.geojson

  # YAML waypoints dosyası oluştur
  python3 geojson_parser.py gorev.geojson --yaml

  # Nav2 PoseStamped formatında (map frame) çıktı
  python3 geojson_parser.py gorev.geojson --nav2

  # Tüm çıktıları aynı anda üret
  python3 geojson_parser.py gorev.geojson --yaml --nav2

  # Özel datum noktası (varsayılan: dosyadaki ilk "start" noktası)
  python3 geojson_parser.py gorev.geojson --nav2 --datum-lat 40.790343 --datum-lon 29.509014

  # GPS→UTM dönüşümü yerine doğrudan lat/lon çıktısı
  python3 geojson_parser.py gorev.geojson --raw-gps

Şartname Formatı (Sayfa 6-7):
  {
    "type": "FeatureCollection",
    "features": [
      {
        "type": "Feature",
        "properties": { "name": "start|gorev_N|park_giris", "description": "...", "nokta_id": 0 },
        "geometry": { "type": "Point", "coordinates": [longitude, latitude] }
      }
    ]
  }

Koordinat Sırası (GeoJSON Standardı): [longitude, latitude]
GPS Formatı (Şartname): Decimal Latitude ve Longitude
"""

import json
import sys
import os
import math
import argparse
from datetime import datetime
from pathlib import Path

# ─────────────────────────────────────────────────────────────────
# Opsiyonel bağımlılıklar — yoksa temel özellikler yine çalışır
# ─────────────────────────────────────────────────────────────────
try:
    import yaml
    HAS_YAML = True
except ImportError:
    HAS_YAML = False

try:
    from pyproj import Transformer
    HAS_PYPROJ = True
except ImportError:
    HAS_PYPROJ = False


# ═══════════════════════════════════════════════════════════════════
# ANSI Renk Kodları (terminal çıktısı güzelleştirme)
# ═══════════════════════════════════════════════════════════════════
class C:
    HEADER  = '\033[95m'
    BLUE    = '\033[94m'
    CYAN    = '\033[96m'
    GREEN   = '\033[92m'
    YELLOW  = '\033[93m'
    RED     = '\033[91m'
    BOLD    = '\033[1m'
    DIM     = '\033[2m'
    RESET   = '\033[0m'
    UNDERLINE = '\033[4m'


# ═══════════════════════════════════════════════════════════════════
# GÖREV NOKTASI SINIFI
# ═══════════════════════════════════════════════════════════════════
class MissionPoint:
    """Tek bir GEOJSON Feature'ından parse edilen görev noktası."""

    # Bilinen nokta tipleri ve BT'deki karşılıkları
    KNOWN_TYPES = {
        'start':      {'bt_type': 'start',      'label': '🏁 Başlangıç',    'color': C.GREEN},
        'gorev':      {'bt_type': 'gorev',       'label': '📍 Görev Noktası', 'color': C.YELLOW},
        'park_giris': {'bt_type': 'park_giris',  'label': '🅿️  Park Giriş',   'color': C.BLUE},
    }

    def __init__(self, feature: dict):
        props = feature.get('properties', {})
        geom  = feature.get('geometry', {})

        self.name        = props.get('name', 'bilinmiyor')
        self.description = props.get('description', '')
        self.nokta_id    = props.get('nokta_id', 0)

        # GeoJSON standardı: [longitude, latitude]
        coords = geom.get('coordinates', [0.0, 0.0])
        self.longitude = coords[0]
        self.latitude  = coords[1]

        # Tip belirleme
        self._determine_type()

        # UTM dönüşüm sonuçları (sonradan doldurulur)
        self.utm_x = None
        self.utm_y = None
        self.local_x = None  # datum'a göre relatif
        self.local_y = None
        self.heading_rad = None  # Bir sonraki noktaya yön açısı

    def _determine_type(self):
        """name alanından nokta tipini belirle."""
        name_lower = self.name.lower()
        if name_lower == 'start':
            self.point_type = 'start'
        elif name_lower == 'park_giris':
            self.point_type = 'park_giris'
        elif name_lower.startswith('gorev'):
            self.point_type = 'gorev'
        else:
            self.point_type = 'unknown'

    @property
    def type_info(self):
        return self.KNOWN_TYPES.get(self.point_type, {
            'bt_type': 'unknown', 'label': '❓ Bilinmiyor', 'color': C.RED
        })

    @property
    def bt_goal_type(self):
        """BT blackboard {current_goal_type} değeri."""
        return self.type_info['bt_type']

    def __repr__(self):
        return f"MissionPoint({self.name}: lat={self.latitude}, lon={self.longitude})"


# ═══════════════════════════════════════════════════════════════════
# GPS → LOKAL KOORDİNAT DÖNÜŞÜMÜ
# ═══════════════════════════════════════════════════════════════════
class CoordinateConverter:
    """GPS (WGS84) → Lokal map frame dönüşümü.

    İki mod:
      1. pyproj varsa: UTM projeksiyon (hassas)
      2. pyproj yoksa: Haversine yaklaşımı (yeterli doğrulukta, ~1cm hata < 5km)
    """

    def __init__(self, datum_lat: float, datum_lon: float):
        self.datum_lat = datum_lat
        self.datum_lon = datum_lon

        if HAS_PYPROJ:
            # UTM zone hesapla
            self.utm_zone = int((datum_lon + 180) / 6) + 1
            hemisphere = 'north' if datum_lat >= 0 else 'south'
            utm_crs = f"+proj=utm +zone={self.utm_zone} +{hemisphere} +datum=WGS84"
            self.transformer = Transformer.from_crs("EPSG:4326", utm_crs, always_xy=True)

            # Datum noktasının UTM koordinatları
            self.datum_utm_x, self.datum_utm_y = self.transformer.transform(datum_lon, datum_lat)
            self.method = "UTM"
        else:
            self.method = "Haversine"

    def to_local(self, lat: float, lon: float) -> tuple:
        """GPS koordinatını datum'a göre lokal (x, y) metreye çevir.

        Returns:
            (x, y) — x: doğu yönü (East), y: kuzey yönü (North)
        """
        if HAS_PYPROJ:
            utm_x, utm_y = self.transformer.transform(lon, lat)
            local_x = utm_x - self.datum_utm_x
            local_y = utm_y - self.datum_utm_y
            return local_x, local_y
        else:
            return self._haversine_local(lat, lon)

    def _haversine_local(self, lat: float, lon: float) -> tuple:
        """PyProj olmadan basit Haversine yaklaşımı."""
        R = 6371000.0  # Dünya yarıçapı (metre)

        dlat = math.radians(lat - self.datum_lat)
        dlon = math.radians(lon - self.datum_lon)
        avg_lat = math.radians((lat + self.datum_lat) / 2.0)

        x = R * dlon * math.cos(avg_lat)  # East
        y = R * dlat                        # North

        return x, y


# ═══════════════════════════════════════════════════════════════════
# ANA PARSER SINIFI
# ═══════════════════════════════════════════════════════════════════
class GeoJSONMissionParser:
    """Yarışma GEOJSON dosyasını okur, doğrular ve çeşitli formatlara dönüştürür."""

    # Şartnameye uygun sıralama
    EXPECTED_ORDER = ['start', 'gorev', 'park_giris']

    def __init__(self, filepath: str):
        self.filepath = Path(filepath)
        self.points: list[MissionPoint] = []
        self.raw_data = None
        self.errors: list[str] = []
        self.warnings: list[str] = []
        self.converter: CoordinateConverter | None = None

    def parse(self) -> bool:
        """GeoJSON dosyasını oku ve doğrula.

        Returns:
            True: başarılı, False: kritik hata var
        """
        # 1. Dosya oku
        if not self._read_file():
            return False

        # 2. Feature'ları parse et
        if not self._parse_features():
            return False

        # 3. Doğrulama
        self._validate()

        return len(self.errors) == 0

    def _read_file(self) -> bool:
        """JSON dosyasını oku."""
        if not self.filepath.exists():
            self.errors.append(f"Dosya bulunamadı: {self.filepath}")
            return False

        try:
            with open(self.filepath, 'r', encoding='utf-8') as f:
                self.raw_data = json.load(f)
        except json.JSONDecodeError as e:
            self.errors.append(f"JSON parse hatası: {e}")
            return False
        except Exception as e:
            self.errors.append(f"Dosya okuma hatası: {e}")
            return False

        return True

    def _parse_features(self) -> bool:
        """FeatureCollection içindeki Feature'ları ayrıştır."""
        if not isinstance(self.raw_data, dict):
            self.errors.append("Kök eleman dict olmalı")
            return False

        if self.raw_data.get('type') != 'FeatureCollection':
            self.errors.append(f"Beklenen type: 'FeatureCollection', bulunan: '{self.raw_data.get('type')}'")
            return False

        features = self.raw_data.get('features', [])
        if not features:
            self.errors.append("features listesi boş!")
            return False

        for i, feature in enumerate(features):
            try:
                point = MissionPoint(feature)
                self.points.append(point)
            except Exception as e:
                self.errors.append(f"Feature #{i} parse hatası: {e}")

        return len(self.points) > 0

    def _validate(self):
        """Şartnameye uygunluk kontrolleri."""
        # Start noktası var mı?
        start_points = [p for p in self.points if p.point_type == 'start']
        if not start_points:
            self.errors.append("❌ 'start' noktası bulunamadı!")
        elif len(start_points) > 1:
            self.warnings.append("⚠️  Birden fazla 'start' noktası var")

        # Park girişi var mı?
        park_points = [p for p in self.points if p.point_type == 'park_giris']
        if not park_points:
            self.warnings.append("⚠️  'park_giris' noktası bulunamadı")

        # Görev noktaları
        gorev_points = [p for p in self.points if p.point_type == 'gorev']
        if not gorev_points:
            self.warnings.append("ℹ️  Görev noktası yok (Round 1/2 olabilir)")

        # Bilinmeyen tipler
        unknown_points = [p for p in self.points if p.point_type == 'unknown']
        for up in unknown_points:
            self.warnings.append(f"⚠️  Bilinmeyen nokta tipi: '{up.name}'")

        # Koordinat geçerliliği
        for p in self.points:
            if not (-90 <= p.latitude <= 90):
                self.errors.append(f"❌ Geçersiz latitude ({p.name}): {p.latitude}")
            if not (-180 <= p.longitude <= 180):
                self.errors.append(f"❌ Geçersiz longitude ({p.name}): {p.longitude}")
            # Türkiye koordinat aralığı kontrolü
            if not (35.5 <= p.latitude <= 42.5 and 25.5 <= p.longitude <= 45.0):
                self.warnings.append(f"⚠️  '{p.name}' koordinatı Türkiye dışında görünüyor "
                                     f"(lat={p.latitude}, lon={p.longitude})")

    def compute_local_coordinates(self, datum_lat: float = None, datum_lon: float = None):
        """GPS → lokal koordinat dönüşümü yap.

        datum verilmezse 'start' noktası origin kabul edilir.
        """
        if datum_lat is None or datum_lon is None:
            start_pts = [p for p in self.points if p.point_type == 'start']
            if start_pts:
                datum_lat = start_pts[0].latitude
                datum_lon = start_pts[0].longitude
            else:
                datum_lat = self.points[0].latitude
                datum_lon = self.points[0].longitude

        self.converter = CoordinateConverter(datum_lat, datum_lon)

        for p in self.points:
            p.local_x, p.local_y = self.converter.to_local(p.latitude, p.longitude)

        # Noktalar arası yön açılarını hesapla
        self._compute_headings()

    def _compute_headings(self):
        """Ardışık noktalar arası yön açısını hesapla (radyan, map frame).

        Şartname: 'Aracın yönünü ifade eder'
        """
        for i in range(len(self.points)):
            if i < len(self.points) - 1:
                dx = self.points[i+1].local_x - self.points[i].local_x
                dy = self.points[i+1].local_y - self.points[i].local_y
                self.points[i].heading_rad = math.atan2(dy, dx)
            else:
                # Son nokta — önceki heading'i kullan (veya 0)
                if i > 0 and self.points[i-1].heading_rad is not None:
                    self.points[i].heading_rad = self.points[i-1].heading_rad
                else:
                    self.points[i].heading_rad = 0.0

    # ─────────────────────────────────────────────────────────────
    # ÇIKTI ÜRETİCİLERİ
    # ─────────────────────────────────────────────────────────────

    def get_mission_order(self) -> list[MissionPoint]:
        """BT sırasına göre nokta listesi: start → gorev_1..N → park_giris."""
        starts = [p for p in self.points if p.point_type == 'start']
        gorevs = sorted([p for p in self.points if p.point_type == 'gorev'],
                        key=lambda p: p.name)
        parks  = [p for p in self.points if p.point_type == 'park_giris']
        others = [p for p in self.points if p.point_type == 'unknown']
        return starts + gorevs + parks + others

    def print_summary(self):
        """Terminal'e renkli, okunabilir özet bas."""
        ordered = self.get_mission_order()

        print()
        print(f"{C.BOLD}{C.CYAN}╔══════════════════════════════════════════════════════════════════╗{C.RESET}")
        print(f"{C.BOLD}{C.CYAN}║  TEKNOFEST 2026 Robotaksi — Görev Dosyası Özeti                 ║{C.RESET}")
        print(f"{C.BOLD}{C.CYAN}╚══════════════════════════════════════════════════════════════════╝{C.RESET}")
        print()
        print(f"  {C.DIM}Dosya:{C.RESET}  {self.filepath.name}")
        print(f"  {C.DIM}Tarih:{C.RESET}  {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
        print(f"  {C.DIM}Nokta:{C.RESET}  {len(self.points)} adet")
        if self.converter:
            print(f"  {C.DIM}Dönüş:{C.RESET}  {self.converter.method} (Zone {getattr(self.converter, 'utm_zone', 'N/A')})")
        print()

        # Hata / Uyarılar
        if self.errors:
            print(f"  {C.RED}{C.BOLD}HATALAR:{C.RESET}")
            for e in self.errors:
                print(f"    {C.RED}{e}{C.RESET}")
            print()

        if self.warnings:
            print(f"  {C.YELLOW}{C.BOLD}UYARILAR:{C.RESET}")
            for w in self.warnings:
                print(f"    {C.YELLOW}{w}{C.RESET}")
            print()

        # Görev Akışı
        print(f"  {C.BOLD}Görev Akışı (BT Sırası):{C.RESET}")
        print(f"  {'─' * 60}")

        for i, p in enumerate(ordered):
            info = p.type_info
            arrow = "  →  " if i < len(ordered) - 1 else "  ■  "

            # Temel bilgi satırı
            print(f"  {info['color']}{C.BOLD}{info['label']}{C.RESET}", end="")
            print(f"  {C.DIM}({p.name}){C.RESET}")

            # GPS koordinatları
            print(f"      GPS:   {p.latitude:.6f}°N, {p.longitude:.6f}°E")

            # Lokal koordinatlar (hesaplandıysa)
            if p.local_x is not None:
                print(f"      Lokal: x={p.local_x:+.3f}m, y={p.local_y:+.3f}m", end="")
                if p.heading_rad is not None:
                    heading_deg = math.degrees(p.heading_rad)
                    print(f"  θ={heading_deg:.1f}°", end="")
                print()

            # Mesafe bir sonraki noktaya
            if i < len(ordered) - 1 and p.local_x is not None:
                next_p = ordered[i + 1]
                if next_p.local_x is not None:
                    dist = math.sqrt((next_p.local_x - p.local_x)**2 +
                                     (next_p.local_y - p.local_y)**2)
                    print(f"      {C.DIM}↓ {dist:.1f}m mesafe{C.RESET}")

            # BT bilgisi
            print(f"      {C.DIM}BT goal_type: \"{p.bt_goal_type}\"{C.RESET}")

            if i < len(ordered) - 1:
                print(f"      {C.DIM}│{C.RESET}")

        print(f"  {'─' * 60}")
        print()

        # Noktalar arası toplam mesafe
        if ordered[0].local_x is not None:
            total_dist = 0
            for i in range(len(ordered) - 1):
                total_dist += math.sqrt(
                    (ordered[i+1].local_x - ordered[i].local_x)**2 +
                    (ordered[i+1].local_y - ordered[i].local_y)**2
                )
            print(f"  {C.BOLD}Toplam rota mesafesi (kuş uçuşu):{C.RESET} {total_dist:.1f}m")
            print()

    def to_yaml_dict(self) -> dict:
        """Nav2/ROS uyumlu YAML sözlüğü üret."""
        ordered = self.get_mission_order()
        waypoints = []

        for i, p in enumerate(ordered):
            wp = {
                'name': p.name,
                'type': p.bt_goal_type,
                'description': p.description,
                'gps': {
                    'latitude': round(p.latitude, 8),
                    'longitude': round(p.longitude, 8),
                },
                'index': i,
            }

            if p.local_x is not None:
                wp['pose'] = {
                    'frame_id': 'map',
                    'position': {
                        'x': round(p.local_x, 4),
                        'y': round(p.local_y, 4),
                        'z': 0.0,
                    },
                    'orientation': self._heading_to_quaternion(p.heading_rad or 0.0),
                }

            waypoints.append(wp)

        result = {
            'mission': {
                'source_file': self.filepath.name,
                'generated_at': datetime.now().isoformat(),
                'coordinate_method': self.converter.method if self.converter else 'raw_gps',
                'datum': {
                    'latitude': self.converter.datum_lat if self.converter else None,
                    'longitude': self.converter.datum_lon if self.converter else None,
                },
                'total_waypoints': len(waypoints),
                'waypoints': waypoints,
            }
        }

        return result

    def save_yaml(self, output_path: str = None) -> str:
        """YAML dosyasına kaydet."""
        if not HAS_YAML:
            # yaml yoksa manuel formatlama
            return self._save_yaml_manual(output_path)

        if output_path is None:
            output_path = str(self.filepath.with_suffix('.yaml'))

        data = self.to_yaml_dict()
        with open(output_path, 'w', encoding='utf-8') as f:
            yaml.dump(data, f, default_flow_style=False, allow_unicode=True, sort_keys=False)

        return output_path

    def _save_yaml_manual(self, output_path: str = None) -> str:
        """PyYAML yoksa manuel YAML formatlama."""
        if output_path is None:
            output_path = str(self.filepath.with_suffix('.yaml'))

        data = self.to_yaml_dict()

        with open(output_path, 'w', encoding='utf-8') as f:
            f.write("# TEKNOFEST 2026 Robotaksi — Görev Waypoints\n")
            f.write(f"# Kaynak: {data['mission']['source_file']}\n")
            f.write(f"# Oluşturulma: {data['mission']['generated_at']}\n\n")

            f.write("mission:\n")
            f.write(f"  source_file: \"{data['mission']['source_file']}\"\n")
            f.write(f"  generated_at: \"{data['mission']['generated_at']}\"\n")
            f.write(f"  coordinate_method: \"{data['mission']['coordinate_method']}\"\n")
            f.write(f"  total_waypoints: {data['mission']['total_waypoints']}\n\n")

            datum = data['mission']['datum']
            f.write("  datum:\n")
            f.write(f"    latitude: {datum['latitude']}\n")
            f.write(f"    longitude: {datum['longitude']}\n\n")

            f.write("  waypoints:\n")
            for wp in data['mission']['waypoints']:
                f.write(f"    - name: \"{wp['name']}\"\n")
                f.write(f"      type: \"{wp['type']}\"\n")
                f.write(f"      description: \"{wp['description']}\"\n")
                f.write(f"      index: {wp['index']}\n")
                f.write(f"      gps:\n")
                f.write(f"        latitude: {wp['gps']['latitude']}\n")
                f.write(f"        longitude: {wp['gps']['longitude']}\n")
                if 'pose' in wp:
                    f.write(f"      pose:\n")
                    f.write(f"        frame_id: \"{wp['pose']['frame_id']}\"\n")
                    f.write(f"        position:\n")
                    f.write(f"          x: {wp['pose']['position']['x']}\n")
                    f.write(f"          y: {wp['pose']['position']['y']}\n")
                    f.write(f"          z: {wp['pose']['position']['z']}\n")
                    f.write(f"        orientation:\n")
                    f.write(f"          x: {wp['pose']['orientation']['x']}\n")
                    f.write(f"          y: {wp['pose']['orientation']['y']}\n")
                    f.write(f"          z: {wp['pose']['orientation']['z']}\n")
                    f.write(f"          w: {wp['pose']['orientation']['w']}\n")
                f.write("\n")

        return output_path

    def print_nav2_poses(self):
        """Nav2 NavigateThroughPoses uyumlu C++ kodu üret."""
        ordered = self.get_mission_order()

        print()
        print(f"{C.BOLD}{C.CYAN}// ═══ Nav2 PoseStamped Listesi (InitMissionAction için) ═══{C.RESET}")
        print(f"{C.DIM}// Otomatik oluşturuldu: {datetime.now().strftime('%Y-%m-%d %H:%M')}{C.RESET}")
        print(f"{C.DIM}// Kaynak: {self.filepath.name}{C.RESET}")
        if self.converter:
            print(f"{C.DIM}// Datum: lat={self.converter.datum_lat}, lon={self.converter.datum_lon}{C.RESET}")
            print(f"{C.DIM}// Yöntem: {self.converter.method}{C.RESET}")
        print()
        print("std::vector<geometry_msgs::msg::PoseStamped> goals_list;")
        print()

        for i, p in enumerate(ordered):
            if p.local_x is None:
                print(f"// ⚠️  '{p.name}': lokal koordinat hesaplanmadı, --nav2 bayrağı ile çalıştırın")
                continue

            quat = self._heading_to_quaternion(p.heading_rad or 0.0)

            print(f"// [{i}] {p.name} — {p.description}")
            print(f"//     GPS: {p.latitude:.6f}°N, {p.longitude:.6f}°E")
            print(f"//     BT goal_type: \"{p.bt_goal_type}\"")
            print("{")
            print("  geometry_msgs::msg::PoseStamped goal;")
            print('  goal.header.frame_id = "map";')
            print(f"  goal.pose.position.x = {p.local_x:.4f};")
            print(f"  goal.pose.position.y = {p.local_y:.4f};")
            print(f"  goal.pose.orientation.x = {quat['x']:.6f};")
            print(f"  goal.pose.orientation.y = {quat['y']:.6f};")
            print(f"  goal.pose.orientation.z = {quat['z']:.6f};")
            print(f"  goal.pose.orientation.w = {quat['w']:.6f};")
            print("  goals_list.push_back(goal);")
            print("}")
            print()

        print(f"// Toplam {len(ordered)} waypoint")
        print()

    def print_raw_gps_table(self):
        """Ham GPS koordinatlarını tablo olarak bas."""
        ordered = self.get_mission_order()

        print()
        print(f"{C.BOLD}{'#':<4} {'İsim':<14} {'Tip':<12} {'Latitude':>12} {'Longitude':>12}  Açıklama{C.RESET}")
        print(f"{'─'*80}")

        for i, p in enumerate(ordered):
            info = p.type_info
            print(f"{info['color']}{i:<4} {p.name:<14} {p.bt_goal_type:<12} "
                  f"{p.latitude:>12.6f} {p.longitude:>12.6f}  {p.description}{C.RESET}")

        print(f"{'─'*80}")
        print()

    @staticmethod
    def _heading_to_quaternion(heading_rad: float) -> dict:
        """Yaw (z ekseni etrafında) → Quaternion dönüşümü.

        ROS2 konvansiyonu: x ileri (East), y sola (North), z yukarı
        """
        qx = 0.0
        qy = 0.0
        qz = math.sin(heading_rad / 2.0)
        qw = math.cos(heading_rad / 2.0)
        return {'x': round(qx, 6), 'y': round(qy, 6),
                'z': round(qz, 6), 'w': round(qw, 6)}

    def save_waypoints_txt(self, output_path: str = None) -> str:
        """InitMissionAction'ın okuyacağı basit waypoints.txt dosyası üret.

        Format (her satır bir waypoint):
          isim tip x y qz qw

        Örnek:
          start start 0.0000 0.0000 -0.8535 0.5211
          gorev_1 gorev -32.3162 -62.9256 0.5253 0.8509
          park_giris park_giris 18.2519 -21.0333 0.6419 0.7668

        C++ tarafı bunu std::ifstream ile satır satır okur.
        """
        ordered = self.get_mission_order()

        if output_path is None:
            output_path = str(self.filepath.with_name('waypoints.txt'))

        with open(output_path, 'w') as f:
            f.write(f"# TEKNOFEST 2026 Robotaksi — Waypoints\n")
            f.write(f"# Kaynak: {self.filepath.name}\n")
            f.write(f"# Format: isim tip x y qz qw\n")
            for p in ordered:
                quat = self._heading_to_quaternion(p.heading_rad or 0.0)
                f.write(f"{p.name} {p.bt_goal_type} "
                        f"{p.local_x:.4f} {p.local_y:.4f} "
                        f"{quat['z']:.6f} {quat['w']:.6f}\n")

        return output_path

    def to_json_summary(self) -> str:
        """Basit JSON özet (diğer araçlarla entegrasyon için)."""
        ordered = self.get_mission_order()
        summary = {
            'source': self.filepath.name,
            'total_points': len(ordered),
            'points': []
        }
        for p in ordered:
            entry = {
                'name': p.name,
                'type': p.bt_goal_type,
                'lat': p.latitude,
                'lon': p.longitude,
            }
            if p.local_x is not None:
                entry['local_x'] = round(p.local_x, 4)
                entry['local_y'] = round(p.local_y, 4)
            summary['points'].append(entry)

        return json.dumps(summary, indent=2, ensure_ascii=False)


# ═══════════════════════════════════════════════════════════════════
# ÖRNEK GEOJSON OLUŞTURUCU (Test amaçlı)
# ═══════════════════════════════════════════════════════════════════
def create_sample_geojson(output_path: str = None) -> str:
    """Şartnamedeki örnek GEOJSON dosyasını oluştur."""
    sample = {
        "type": "FeatureCollection",
        "features": [
            {
                "type": "Feature",
                "properties": {
                    "name": "start",
                    "description": "Araç başlangıç konumu",
                    "nokta_id": 0
                },
                "geometry": {
                    "type": "Point",
                    "coordinates": [29.509014, 40.790343]
                }
            },
            {
                "type": "Feature",
                "properties": {
                    "name": "gorev_1",
                    "description": "1. görev noktası",
                    "nokta_id": 0
                },
                "geometry": {
                    "type": "Point",
                    "coordinates": [29.50861, 40.789785]
                }
            },
            {
                "type": "Feature",
                "properties": {
                    "name": "gorev_2",
                    "description": "2. görev noktası",
                    "nokta_id": 0
                },
                "geometry": {
                    "type": "Point",
                    "coordinates": [29.508726, 40.789949]
                }
            },
            {
                "type": "Feature",
                "properties": {
                    "name": "gorev_3",
                    "description": "3. görev noktası",
                    "nokta_id": 0
                },
                "geometry": {
                    "type": "Point",
                    "coordinates": [29.509082, 40.789635]
                }
            },
            {
                "type": "Feature",
                "properties": {
                    "name": "park_giris",
                    "description": "Araç otopark giriş bölgesi noktası",
                    "nokta_id": 0
                },
                "geometry": {
                    "type": "Point",
                    "coordinates": [29.509223, 40.790149]
                }
            }
        ]
    }

    if output_path is None:
        output_path = "ornek_gorev.geojson"

    with open(output_path, 'w', encoding='utf-8') as f:
        json.dump(sample, f, indent=2, ensure_ascii=False)

    return output_path


# ═══════════════════════════════════════════════════════════════════
# CLI
# ═══════════════════════════════════════════════════════════════════
def main():
    parser = argparse.ArgumentParser(
        description='TEKNOFEST 2026 Robotaksi — GeoJSON Görev Dosyası Ayrıştırıcı',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Örnekler:
  %(prog)s gorev.geojson                    # Okunabilir özet
  %(prog)s gorev.geojson --yaml             # YAML dosyası oluştur
  %(prog)s gorev.geojson --nav2             # Nav2 C++ kodu üret
  %(prog)s gorev.geojson --yaml --nav2      # İkisini birden
  %(prog)s gorev.geojson --raw-gps          # Sadece GPS tablosu
  %(prog)s --sample                         # Örnek GeoJSON dosyası oluştur
  %(prog)s gorev.geojson --json-summary     # JSON özet (pipe/entegrasyon için)
        """)

    parser.add_argument('input', nargs='?', help='GeoJSON dosya yolu')
    parser.add_argument('--yaml', action='store_true',
                        help='YAML waypoints dosyası oluştur')
    parser.add_argument('--yaml-output', '-o', type=str, default=None,
                        help='YAML çıktı dosya yolu (varsayılan: <girdi>.yaml)')
    parser.add_argument('--nav2', action='store_true',
                        help='Nav2 PoseStamped C++ kodu üret')
    parser.add_argument('--raw-gps', action='store_true',
                        help='Ham GPS koordinatları tablosu')
    parser.add_argument('--json-summary', action='store_true',
                        help='JSON özet (pipe/entegrasyon için)')
    parser.add_argument('--sample', action='store_true',
                        help='Örnek GeoJSON dosyası oluştur (Şartname sayfa 6-7)')
    parser.add_argument('--datum-lat', type=float, default=None,
                        help='Datum noktası latitude (varsayılan: start noktası)')
    parser.add_argument('--datum-lon', type=float, default=None,
                        help='Datum noktası longitude (varsayılan: start noktası)')
    parser.add_argument('--waypoints', action='store_true',
                        help='InitMissionAction için waypoints.txt dosyası üret')
    parser.add_argument('--no-color', action='store_true',
                        help='ANSI renk kodlarını devre dışı bırak')

    args = parser.parse_args()

    # Renk desteği
    if args.no_color or not sys.stdout.isatty():
        for attr in dir(C):
            if not attr.startswith('_'):
                setattr(C, attr, '')

    # Örnek dosya oluştur
    if args.sample:
        path = create_sample_geojson()
        print(f"{C.GREEN}✅ Örnek GeoJSON dosyası oluşturuldu: {path}{C.RESET}")
        print(f"{C.DIM}   Şartname Sayfa 6-7'deki formata uygun.{C.RESET}")
        return

    # Girdi dosyası kontrolü
    if not args.input:
        parser.print_help()
        sys.exit(1)

    # ─── Parse ───
    mission = GeoJSONMissionParser(args.input)
    success = mission.parse()

    if not success:
        print(f"\n{C.RED}{C.BOLD}❌ GeoJSON parse başarısız!{C.RESET}")
        for e in mission.errors:
            print(f"  {C.RED}{e}{C.RESET}")
        sys.exit(1)

    # ─── Koordinat dönüşümü ───
    mission.compute_local_coordinates(args.datum_lat, args.datum_lon)

    # ─── Çıktılar ───
    if args.raw_gps:
        mission.print_raw_gps_table()
    elif args.json_summary:
        print(mission.to_json_summary())
    else:
        # Varsayılan: okunabilir özet
        mission.print_summary()

    if args.nav2:
        mission.print_nav2_poses()

    if args.yaml:
        yaml_path = mission.save_yaml(args.yaml_output)
        print(f"{C.GREEN}✅ YAML dosyası kaydedildi: {yaml_path}{C.RESET}")

    if args.waypoints:
        wp_path = mission.save_waypoints_txt()
        print(f"{C.GREEN}✅ Waypoints dosyası kaydedildi: {wp_path}{C.RESET}")
        print(f"{C.DIM}   InitMissionAction bu dosyayı otomatik okuyacak.{C.RESET}")

    # Uyarıları son olarak göster
    if mission.warnings and not args.json_summary:
        print(f"\n{C.YELLOW}⚠️  {len(mission.warnings)} uyarı var (detaylar yukarıda){C.RESET}")


if __name__ == '__main__':
    main()
