#!/usr/bin/env python3
"""
layout.py'deki yerleşimden üç dosya üretir:
  worlds/four_rooms.world, maps/four_rooms.pgm + .yaml, config/world.yaml

Kullanım (paket kaynak klasöründen):
  python3 scripts/generate_world.py
"""

import math
import os
import sys

import numpy as np
import yaml

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
sys.path.insert(0, PKG)
from mrm_simulation import layout as L  # noqa: E402


# ----------------------------------------------------------------------------
# SDF yardımcıları
# ----------------------------------------------------------------------------
def material(rgb, emissive=0.0):
    r, g, b = rgb
    e = f'{r*emissive:.3f} {g*emissive:.3f} {b*emissive:.3f} 1'
    return (f'<material><ambient>{r:.3f} {g:.3f} {b:.3f} 1</ambient>'
            f'<diffuse>{r:.3f} {g:.3f} {b:.3f} 1</diffuse>'
            f'<specular>0.1 0.1 0.1 1</specular><emissive>{e}</emissive></material>')


def box_geom(sx, sy, sz):
    return f'<geometry><box><size>{sx:.4f} {sy:.4f} {sz:.4f}</size></box></geometry>'


def box_cv(name, pose, size, rgb, collision=True, emissive=0.0):
    """Bir link içinde collision + visual çifti."""
    p = ' '.join(f'{v:.4f}' for v in pose)
    s = ''
    if collision:
        s += f'<collision name="{name}_c"><pose>{p}</pose>{box_geom(*size)}</collision>'
    s += f'<visual name="{name}_v"><pose>{p}</pose>{box_geom(*size)}{material(rgb, emissive)}</visual>'
    return s


def walls_model():
    parts = []
    for i, (x1, y1, x2, y2) in enumerate(L.wall_segments()):
        cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
        length = math.hypot(x2 - x1, y2 - y1) + L.WALL_T
        yaw = math.atan2(y2 - y1, x2 - x1)
        parts.append(box_cv(f'wall{i}', (cx, cy, L.WALL_H / 2, 0, 0, yaw),
                            (length, L.WALL_T, L.WALL_H), (0.93, 0.90, 0.82)))
    # Oda zeminleri: sadece görsel, açık pastel tonlar (algılamayı şaşırtmasın diye düşük doygunluk)
    for name, r in L.ROOMS.items():
        x0, x1, y0, y1 = r['bounds']
        parts.append(box_cv(f'floor_{name}', ((x0 + x1) / 2, (y0 + y1) / 2, 0.0005, 0, 0, 0),
                            (x1 - x0, y1 - y0, 0.001), r['floor'], collision=False))
    return (f'<model name="walls"><static>true</static><link name="link">'
            f'{"".join(parts)}</link></model>')


def door_model(name):
    g = L.door_geometry(name)
    px, py = g['panel_center']
    sx, sy = g['panel_size']
    bx, by, bz = g['button_pos']
    bs = L.BUTTON_SIZE
    ox, oy = g['open_offset']
    panel = (f'<link name="panel"><pose>{px} {py} {L.DOOR_H/2 + 0.005} 0 0 0</pose>'
             f'<kinematic>true</kinematic><gravity>false</gravity>'
             f'<inertial><mass>5</mass></inertial>'
             f'{box_cv("panel", (0, 0, 0, 0, 0, 0), (sx, sy, L.DOOR_H), (0.55, 0.35, 0.20))}</link>')
    # Buton: sarı + biraz ışıyan (emissive) - parça renkleriyle (R/G/B) karışmaz
    button = (f'<link name="button"><pose>{bx:.4f} {by:.4f} {bz} 0 0 {g["button_yaw"]:.5f}</pose>'
              f'<kinematic>true</kinematic><gravity>false</gravity>'
              f'<inertial><mass>0.1</mass></inertial>'
              f'{box_cv("button", (0, 0, 0, 0, 0, 0), bs, (0.95, 0.85, 0.05), emissive=0.4)}'
              # Butonun arkasındaki sabit çerçeve (görsel)
              f'<visual name="plate"><pose>{-bs[0]/2:.4f} 0 0 0 0 0</pose>'
              f'{box_geom(0.005, 0.16, 0.16)}{material((0.2, 0.2, 0.2))}</visual></link>')
    plugin = (f'<plugin name="{name}_plugin" filename="libmrm_door_button.so">'
              f'<ros><namespace>/mrm</namespace></ros>'
              f'<door_name>{name}</door_name><robot_model>mrm_robot</robot_model>'
              f'<trigger_link>link5</trigger_link><trigger_offset>0.13 0 0</trigger_offset>'
              f'<open_offset>{ox} {oy} 0</open_offset><open_speed>0.3</open_speed>'
              f'<button_size>{bs[0]} {bs[1]} {bs[2]}</button_size>'
              f'<press_margin>0.02</press_margin><press_depth>0.012</press_depth></plugin>')
    return f'<model name="{name}"><static>false</static>{panel}{button}{plugin}</model>'


def bin_tag_id(room, color):
    """Kutuların etiket kimlikleri: room1..room4 sırasıyla, her odada BINS listesindeki sırayla 0..11."""
    k = 0
    for r, lst in L.BINS.items():
        for c, _ in lst:
            if (r, c) == (room, color):
                return k
            k += 1
    raise KeyError((room, color))


def tag_visual(tag_id, total, _unused, approach_yaw, half_depth, z, origin):
    """
    Yüzeye yapıştırılmış AprilTag: ince bir kutu; dokusu Mrm/Tag<id> materyali.
    Etiket, robotun yaklaşma yönünün TERSİNE (robota) bakar.
    """
    nx, ny = -math.cos(approach_yaw), -math.sin(approach_yaw)   # dışa doğru normal
    px = origin[0] + nx * (half_depth + 0.0015)
    py = origin[1] + ny * (half_depth + 0.0015)
    yaw = math.atan2(ny, nx)
    return (f'<visual name="tag{tag_id}"><pose>{px:.4f} {py:.4f} {z:.4f} 0 0 {yaw:.5f}</pose>'
            f'<geometry><box><size>0.002 {total} {total}</size></box></geometry>'
            f'<material><script><uri>model://mrm_apriltags/materials/scripts</uri>'
            f'<uri>model://mrm_apriltags/materials/textures</uri><name>Mrm/Tag{tag_id}</name>'
            f'</script></material></visual>')


def bin_model(room, idx, color, center):
    x, y = center
    o, t, h = L.BIN_OUTER, L.BIN_WALL, L.BIN_H
    rgb = L.COLORS[color]
    parts = box_cv('bottom', (0, 0, t / 2, 0, 0, 0), (o, o, t), rgb)
    for k, (dx, dy, sx, sy) in enumerate([(o/2 - t/2, 0, t, o), (-o/2 + t/2, 0, t, o),
                                          (0, o/2 - t/2, o, t), (0, -o/2 + t/2, o, t)]):
        parts += box_cv(f'side{k}', (dx, dy, h / 2, 0, 0, 0), (sx, sy, h), rgb)
    parts += tag_visual(bin_tag_id(room, color), L.BIN_TAG_TOTAL, -L.BIN_APPROACH_YAW[room],
                        L.BIN_APPROACH_YAW[room], o / 2, h / 2, (0.0, 0.0))
    return (f'<model name="bin_{room}_{color}"><static>true</static>'
            f'<pose>{x} {y} 0 0 0 0</pose><link name="link">{parts}</link></model>')


def dock_model():
    d = L.DOCK
    sx, sy = d['station_center']
    w, dpt, h = d['station_size']
    px, py, _ = d['pose']
    parts = box_cv('station', (sx, sy, h / 2, 0, 0, 0), (w, dpt, h), (0.25, 0.25, 0.28))
    # Şarj kontakları: etiketin ALTINDA
    parts += box_cv('contacts', (sx, sy - dpt / 2 - 0.003, 0.015, 0, 0, 0), (0.18, 0.006, 0.02),
                    (0.9, 0.6, 0.1), collision=False)
    # Zemin işareti
    parts += box_cv('pad', (px, py, 0.0015, 0, 0, 0), (0.40, 0.45, 0.001), (0.3, 0.3, 0.3),
                    collision=False)
    parts += box_cv('pad_line', (px, py, 0.0022, 0, 0, 0), (0.03, 0.40, 0.001), (1, 1, 1),
                    collision=False)
    # İstasyon ön yüzündeki AprilTag (odaya, -y'ye bakar). approach_yaw = +pi/2 (robot istasyona bakar)
    parts += tag_visual(L.DOCK_TAG_ID, L.DOCK_TAG_TOTAL, None, math.pi / 2, dpt / 2, L.DOCK_TAG_Z, (sx, sy))
    return f'<model name="{d["name"]}"><static>true</static><link name="link">{parts}</link></model>'


def build_world():
    models = [walls_model(), dock_model()]
    models += [door_model(n) for n in L.DOORS]
    for room, bins in L.BINS.items():
        for i, (c, xy) in enumerate(bins):
            models.append(bin_model(room, i, c, xy))
    return f'''<?xml version="1.0"?>
<!-- OTOMATIK URETILDI: scripts/generate_world.py  (elle duzenlemeyin, layout.py'yi duzenleyin) -->
<sdf version="1.6">
  <world name="four_rooms">
    <physics type="ode">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>1000</real_time_update_rate>
    </physics>
    <scene><ambient>0.6 0.6 0.6 1</ambient><background>0.7 0.8 0.9 1</background>
      <shadows>false</shadows></scene>
    <include><uri>model://sun</uri></include>
    <include><uri>model://ground_plane</uri></include>
    <gui><camera name="user_camera"><pose>4 -9 11 0 0.95 1.5708</pose></camera></gui>
    <plugin name="mrm_link_attacher" filename="libmrm_link_attacher.so">
      <ros><namespace>/mrm</namespace></ros>
    </plugin>
    <plugin name="gazebo_ros_state" filename="libgazebo_ros_state.so">
      <ros><namespace>/gazebo</namespace></ros>
      <update_rate>10.0</update_rate>
    </plugin>
    {chr(10).join("    " + m for m in models)}
  </world>
</sdf>
'''


# ----------------------------------------------------------------------------
# Doluluk haritası (Nav2 map_server formatı)
# ----------------------------------------------------------------------------
RES = 0.05
ORIGIN = (-0.5, -4.5)
SIZE = (9.0, 9.0)


def build_map(doors_closed=False):
    w, h = int(SIZE[0] / RES), int(SIZE[1] / RES)
    grid = np.full((h, w), 254, dtype=np.uint8)  # 254 = boş (beyaz)

    def fill_rect(cx, cy, sx, sy):
        x0 = int(math.floor((cx - sx / 2 - ORIGIN[0]) / RES))
        x1 = int(math.ceil((cx + sx / 2 - ORIGIN[0]) / RES))
        y0 = int(math.floor((cy - sy / 2 - ORIGIN[1]) / RES))
        y1 = int(math.ceil((cy + sy / 2 - ORIGIN[1]) / RES))
        x0, x1 = max(0, x0), min(w, x1)
        y0, y1 = max(0, y0), min(h, y1)
        grid[y0:y1, x0:x1] = 0  # 0 = dolu (siyah)

    # Dış duvarların dışını "bilinmeyen" (205) yap
    for j in range(h):
        for i in range(w):
            x = ORIGIN[0] + (i + 0.5) * RES
            y = ORIGIN[1] + (j + 0.5) * RES
            if x < -0.05 or x > 8.05 or y < -4.05 or y > 4.05:
                grid[j, i] = 205
    for x1, y1, x2, y2 in L.wall_segments():
        cx, cy = (x1 + x2) / 2, (y1 + y2) / 2
        if x1 == x2:
            fill_rect(cx, cy, L.WALL_T, abs(y2 - y1) + L.WALL_T)
        else:
            fill_rect(cx, cy, abs(x2 - x1) + L.WALL_T, L.WALL_T)
    # Kutular ve şarj istasyonu: lidar görmese de statik haritada engel olsunlar
    for bins in L.BINS.values():
        for _, (x, y) in bins:
            fill_rect(x, y, L.BIN_OUTER, L.BIN_OUTER)
    sc = L.DOCK['station_center']
    fill_rect(sc[0], sc[1], L.DOCK['station_size'][0], L.DOCK['station_size'][1])
    if doors_closed:
        for n in L.DOORS:
            g = L.door_geometry(n)
            sx, sy = g['panel_size']
            fill_rect(g['panel_center'][0], g['panel_center'][1], max(sx, L.WALL_T), max(sy, L.WALL_T))
    # PGM'de satır 0 = haritanın ÜST kenarı (y max) -> dikey çevir
    return np.flipud(grid)


def write_pgm(path, img):
    h, w = img.shape
    with open(path, 'wb') as f:
        f.write(f'P5\n# MRM four rooms\n{w} {h}\n255\n'.encode())
        f.write(img.tobytes())


# ----------------------------------------------------------------------------
# Anlamsal dünya bilgisi
# ----------------------------------------------------------------------------
def r(v, n=4):
    return [round(float(x), n) for x in v]


def build_semantic():
    doors = {}
    for name, d in L.DOORS.items():
        g = L.door_geometry(name)
        doors[name] = {
            'rooms': list(d['rooms']), 'approach_from': d['approach_from'],
            'panel_center': r(g['panel_center']), 'button_pos': r(g['button_pos']),
            'button_yaw': round(g['button_yaw'], 5), 'button_size': list(L.BUTTON_SIZE),
            'pre_button_pose': r(g['pre_button_pose']),
            'pass_before': r(g['pass_before']), 'pass_after': r(g['pass_after']),
        }
    bins = {}
    for room, lst in L.BINS.items():
        for c, (x, y) in lst:
            bins[f'bin_{room}_{c}'] = {'room': room, 'color': c, 'position': [x, y],
                                      'approach_yaw': round(L.BIN_APPROACH_YAW[room], 5),
                                      'height': L.BIN_H, 'size': L.BIN_OUTER,
                                      'tag_id': bin_tag_id(room, c),
                                      'tag_size': round(L.BIN_TAG_TOTAL * 0.8, 4)}
    rooms = {n: {'bounds': list(v['bounds']), 'center': list(v['center']),
                 'keepout': [r(z) for z in L.keepout_zones(n)]} for n, v in L.ROOMS.items()}
    return {
        'room_order': L.ROOM_ORDER, 'rooms': rooms, 'doors': doors, 'bins': bins,
        'colors': list(L.COLORS.keys()),
        'dock': {'name': L.DOCK['name'], 'room': L.DOCK['room'], 'pose': r(L.DOCK['pose']),
                 'staging': r(L.DOCK['staging']), 'charge_radius': L.DOCK['charge_radius'],
                 # Lidar ile yanaşma referansları (robot -y'ye bakarken):
                 #  sağdaki batı duvarına (x=0) dik mesafe ve arkadaki istasyon yüzüne mesafe
                 'tag_id': L.DOCK_TAG_ID, 'tag_size': round(L.DOCK_TAG_TOTAL * 0.8, 4),
                 # robot merkezi (dock pozunda) ile etiket yüzeyi arası mesafe
                 'tag_to_dock': round(L.DOCK['station_center'][1] - L.DOCK['station_size'][1] / 2
                                      - L.DOCK['pose'][1], 4),
                 'side_wall_dist': round(L.DOCK['pose'][0] - L.WALL_T / 2, 4),
                 'rear_dist': round(L.DOCK['station_center'][1] - L.DOCK['station_size'][1] / 2
                                    - (L.DOCK['pose'][1] + 0.024), 4)},
        'robot_start': r(L.ROBOT_START),
        'piece_size': L.PIECE_SIZE,
        'spawn': {'wall_margin': L.SPAWN_WALL_MARGIN, 'min_separation': L.SPAWN_MIN_SEPARATION},
    }


def main():
    os.makedirs(os.path.join(PKG, 'worlds'), exist_ok=True)
    os.makedirs(os.path.join(PKG, 'maps'), exist_ok=True)
    os.makedirs(os.path.join(PKG, 'config'), exist_ok=True)
    with open(os.path.join(PKG, 'worlds', 'four_rooms.world'), 'w') as f:
        f.write(build_world())
    # İki harita:
    #   four_rooms.*        kapılar AÇIK  -> global costmap (planlayıcı kapıdan geçebilsin)
    #   four_rooms_amcl.*   kapılar KAPALI -> AMCL (lidarın gördüğüyle uyumlu lokalizasyon)
    for name, closed in (('four_rooms', False), ('four_rooms_amcl', True)):
        write_pgm(os.path.join(PKG, 'maps', name + '.pgm'), build_map(closed))
        with open(os.path.join(PKG, 'maps', name + '.yaml'), 'w') as f:
            yaml.safe_dump({'image': name + '.pgm', 'mode': 'trinary', 'resolution': RES,
                            'origin': [ORIGIN[0], ORIGIN[1], 0.0], 'negate': 0,
                            'occupied_thresh': 0.65, 'free_thresh': 0.25}, f, sort_keys=False)
    with open(os.path.join(PKG, 'config', 'world.yaml'), 'w') as f:
        f.write('# OTOMATIK URETILDI: scripts/generate_world.py\n')
        yaml.safe_dump(build_semantic(), f, sort_keys=False, default_flow_style=None)
    print('Uretildi: worlds/four_rooms.world, maps/four_rooms.{pgm,yaml}, config/world.yaml')


if __name__ == '__main__':
    main()
