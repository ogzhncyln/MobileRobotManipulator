"""
Dört odalı dünyanın TEK GERÇEK KAYNAĞI (single source of truth).

Bu dosyadaki sayılardan üç çıktı üretilir (scripts/generate_world.py):
  1. worlds/four_rooms.world   -> Gazebo'nun yüklediği SDF dünya
  2. maps/four_rooms.pgm/.yaml -> Nav2 map_server'ın yüklediği doluluk haritası
  3. config/world.yaml         -> Beceri sunucusu, görev kontrolcüsü ve
                                  spawner'ın okuduğu anlamsal bilgi
                                  (oda sınırları, kapılar, butonlar, kutular, dock)

Koordinat sistemi: Gazebo dünya çerçevesi == Nav2 'map' çerçevesi (robot,
haritada AMCL'ye verilen başlangıç pozunda doğar).

Yerleşim (üstten görünüm, x sağa, y yukarı):

      y=4 +-----------------+-----------------+
          |  [DOCK]         |   [B] [R] [G]   |
          |                 |                 |
          |     ROOM 1     door_12   ROOM 2   |
          |  (başlangıç)    |                 |
          |   [R] [G] [B]   |                 |
      y=0 +-----------------+-----door_23-----+
          | [R]             |              [G]|
          | [B]  ROOM 4    door_34   ROOM 3[B]|
          | [G]             |              [R]|
          |                 |                 |
     y=-4 +-----------------+-----------------+
         x=0               x=4               x=8

Görev sırası: Room1 -> Room2 -> Room3 -> Room4
"""

import math

WALL_T = 0.10          # Duvar kalınlığı (m)
WALL_H = 0.50          # Duvar yüksekliği (m) - lidar 0.13 m'de, rahatça görür
DOOR_W = 1.00          # Kapı genişliği (m)
DOOR_T = 0.04          # Kapı kanadı kalınlığı
DOOR_H = 0.45
BUTTON_Z = 0.20        # Buton merkezinin yerden yüksekliği
BUTTON_SIZE = (0.03, 0.10, 0.10)   # (duvardan çıkıntı, genişlik, yükseklik)

BIN_OUTER = 0.22       # Renkli kutu dış ölçüsü
BIN_WALL = 0.012
BIN_H = 0.10          # 0.07 -> 0.10: ön yüzde AprilTag için yer

PIECE_SIZE = 0.035     # Renkli küp kenarı

# AprilTag (tag36h11). TOPLAM kenar = beyaz kenar dahil; apriltag_ros 'size' = siyah kare = 0.8 * toplam
BIN_TAG_TOTAL = 0.09   # kutu ön yüzünde (yükseklik 0.10)
DOCK_TAG_TOTAL = 0.14  # dock istasyonu ön yüzünde
DOCK_TAG_ID = 20
DOCK_TAG_Z = 0.10      # etiket merkezinin yüksekliği (kamera yaklaşma pozundan tamamını görebilsin)

COLORS = {
    # isim: (r, g, b)  - Gazebo materyali ve HSV eşikleri bu renklere göre seçildi
    'red':   (0.90, 0.05, 0.05),
    'green': (0.05, 0.75, 0.10),
    'blue':  (0.05, 0.15, 0.90),
}

ROOMS = {
    'room1': {'bounds': (0.0, 4.0, 0.0, 4.0), 'center': (2.0, 2.0), 'floor': (0.85, 0.85, 0.80)},
    'room2': {'bounds': (4.0, 8.0, 0.0, 4.0), 'center': (6.0, 2.0), 'floor': (0.80, 0.84, 0.86)},
    'room3': {'bounds': (4.0, 8.0, -4.0, 0.0), 'center': (6.0, -2.0), 'floor': (0.86, 0.82, 0.80)},
    'room4': {'bounds': (0.0, 4.0, -4.0, 0.0), 'center': (2.0, -2.0), 'floor': (0.82, 0.86, 0.80)},
}
ROOM_ORDER = ['room1', 'room2', 'room3', 'room4']

# Kapılar: hangi duvarda, açıklık aralığı, hangi odadan yaklaşılır (buton o tarafta)
#   wall: ('x', 4.0) -> x=4 doğrusundaki dikey duvar; span: y aralığı
#   approach_dir: robotun butona bakarken yöneldiği birim vektör
DOORS = {
    'door_12': {'rooms': ('room1', 'room2'), 'wall': ('x', 4.0), 'span': (1.5, 2.5),
                'approach_from': 'room1', 'approach_dir': (1.0, 0.0), 'open_offset': (0.0, 1.0)},
    'door_23': {'rooms': ('room2', 'room3'), 'wall': ('y', 0.0), 'span': (5.5, 6.5),
                'approach_from': 'room2', 'approach_dir': (0.0, -1.0), 'open_offset': (1.0, 0.0)},
    'door_34': {'rooms': ('room3', 'room4'), 'wall': ('x', 4.0), 'span': (-2.5, -1.5),
                'approach_from': 'room3', 'approach_dir': (-1.0, 0.0), 'open_offset': (0.0, -1.0)},
}

# Renkli kutular (bin): merkez ve robotun kutuya bakarken yönelimi (yaw)
BINS = {
    'room1': [('red', (1.4, 0.35)), ('green', (2.0, 0.35)), ('blue', (2.6, 0.35))],
    'room2': [('blue', (5.4, 3.65)), ('red', (6.0, 3.65)), ('green', (6.6, 3.65))],
    'room3': [('green', (7.65, -1.4)), ('blue', (7.65, -2.0)), ('red', (7.65, -2.6))],
    'room4': [('red', (0.35, -1.4)), ('blue', (0.35, -2.0)), ('green', (0.35, -2.6))],
}
BIN_APPROACH_YAW = {'room1': -math.pi / 2, 'room2': math.pi / 2, 'room3': 0.0, 'room4': math.pi}

# Şarj istasyonu (Room 1, kuzey duvarı). Robot geri geri girerek yanaşır.
DOCK = {
    'name': 'dock1', 'room': 'room1',
    'station_center': (1.0, 3.89), 'station_size': (0.30, 0.12, 0.30),
    'pose': (1.0, 3.58, -math.pi / 2),        # Şarj olurken base_footprint pozu
    # Nav2 ile gidilen ön-yanaşma pozu: robot istasyona BAKAR (etiketi okumak için),
    # hizalandıktan sonra 180° dönüp geri geri yanaşır
    'staging': (1.0, 2.98, math.pi / 2),
    'charge_radius': 0.10,                    # Bu yarıçap içinde şarj olur
}

ROBOT_START = (1.0, 2.0, 0.0)   # x, y, yaw  (room1)

# Parça (küp) üretimi için yasak bölgeler: (x, y, yarıçap)
SPAWN_WALL_MARGIN = 0.45
SPAWN_MIN_SEPARATION = 0.55


def door_geometry(name):
    """Kapı kanadı merkezi, butonun pozu ve robotun butona bakan ön-pozunu hesapla."""
    d = DOORS[name]
    axis, pos = d['wall']
    a, b = d['span']
    mid = (a + b) / 2.0
    ax, ay = d['approach_dir']
    if axis == 'x':
        panel_center = (pos, mid)
        panel_size = (DOOR_T, DOOR_W)
    else:
        panel_center = (mid, pos)
        panel_size = (DOOR_W, DOOR_T)
    # Robot duvara bakarken (approach_dir), sağ tarafı = approach_dir'i -90° döndür
    rx, ry = ay, -ax
    # Buton: kapı kenarından 0.25 m sağda, duvar yüzeyinde
    edge_offset = DOOR_W / 2.0 + 0.25
    face = pos - (WALL_T / 2.0 + BUTTON_SIZE[0] / 2.0) * (ax if axis == 'x' else ay)
    if axis == 'x':
        bx, by = face, mid + ry * edge_offset
    else:
        bx, by = mid + rx * edge_offset, face
    # Buton linkinin x ekseni duvardan odaya doğru (approach_dir'in tersi)
    button_yaw = math.atan2(-ay, -ax)
    approach_yaw = math.atan2(ay, ax)
    # Nav2 ile gidilecek ön-poz: buton yüzünden 0.65 m geride, duvara bakan
    standoff = 0.65
    pre = (bx - ax * standoff, by - ay * standoff, approach_yaw)
    # Kapıdan geçiş noktaları (her iki tarafta 0.7 m)
    pc = panel_center
    return {
        'panel_center': panel_center, 'panel_size': panel_size,
        'button_pos': (bx, by, BUTTON_Z), 'button_yaw': button_yaw,
        'pre_button_pose': pre,
        'open_offset': d['open_offset'],
        'pass_before': (pc[0] - ax * 0.7, pc[1] - ay * 0.7, approach_yaw),
        'pass_after': (pc[0] + ax * 0.7, pc[1] + ay * 0.7, approach_yaw),
    }


def wall_segments():
    """Duvar parçaları: (x1, y1, x2, y2) merkez çizgileri; kapı boşlukları hariç."""
    segs = []
    # Dış duvarlar
    segs += [(0, 4, 8, 4), (0, -4, 8, -4), (0, -4, 0, 4), (8, -4, 8, 4)]
    # İç duvar x=4: door_12 [1.5,2.5] ve door_34 [-2.5,-1.5] boşlukları
    segs += [(4, -4, 4, -2.5), (4, -1.5, 4, 1.5), (4, 2.5, 4, 4)]
    # İç duvar y=0: [0,4] tam (room1|room4), [4,8] door_23 [5.5,6.5] boşluğu
    segs += [(0, 0, 4, 0), (4, 0, 5.5, 0), (6.5, 0, 8, 0)]
    return segs


def keepout_zones(room):
    """Parçaların doğamayacağı daire bölgeler (x, y, r)."""
    zones = [(*ROOMS[room]['center'], 0.75)]
    for _, (x, y) in BINS[room]:
        zones.append((x, y, 0.75))
    for name, d in DOORS.items():
        if room in d['rooms']:
            g = door_geometry(name)
            zones.append((*g['panel_center'], 1.1))
            if d['approach_from'] == room:
                zones.append((g['pre_button_pose'][0], g['pre_button_pose'][1], 0.75))
                zones.append((g['button_pos'][0], g['button_pos'][1], 0.6))
    if DOCK['room'] == room:
        zones.append((DOCK['staging'][0], DOCK['staging'][1], 0.9))
        zones.append((DOCK['pose'][0], DOCK['pose'][1], 0.6))
    if room == 'room1':
        zones.append((ROBOT_START[0], ROBOT_START[1], 0.6))
    return zones
