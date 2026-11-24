#!/usr/bin/env python3
"""
AprilTag (tag36h11) doku görüntüleri ve Gazebo OGRE materyal betiği üretir.

Görüntü: 8x8 hücrelik etiket (siyah çerçeve dahil) + her yanda 1 hücre BEYAZ kenar = 10x10 hücre.
apriltag_ros'un 'size' parametresi SİYAH karenin kenarıdır = toplam kenar * 0.8
"""
import os

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(os.path.dirname(HERE), 'models', 'mrm_apriltags', 'materials')
IDS = list(range(0, 12)) + [20]      # 0-11: kutular, 20: dock
CELL = 40                            # piksel/hücre -> 400x400 görüntü


def main():
    d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    script = []
    for tid in IDS:
        tag = cv2.aruco.generateImageMarker(d, tid, 8 * CELL, borderBits=1)
        img = np.full((10 * CELL, 10 * CELL), 255, np.uint8)
        img[CELL:9 * CELL, CELL:9 * CELL] = tag
        name = f'tag36h11_{tid:02d}.png'
        cv2.imwrite(os.path.join(OUT, 'textures', name), img)
        script.append(f'''material Mrm/Tag{tid}
{{
  technique
  {{
    pass
    {{
      ambient 1 1 1 1
      diffuse 1 1 1 1
      specular 0 0 0 1
      emissive 0.4 0.4 0.4 1
      texture_unit
      {{
        texture {name}
        filtering none
      }}
    }}
  }}
}}
''')
    with open(os.path.join(OUT, 'scripts', 'mrm_apriltags.material'), 'w') as f:
        f.write('\n'.join(script))
    with open(os.path.join(os.path.dirname(OUT), 'model.config'), 'w') as f:
        f.write('<?xml version="1.0"?>\n<model><name>mrm_apriltags</name><version>1.0</version>'
                '<sdf version="1.6">model.sdf</sdf><description>AprilTag dokulari</description></model>\n')
    with open(os.path.join(os.path.dirname(OUT), 'model.sdf'), 'w') as f:
        f.write('<?xml version="1.0"?>\n<sdf version="1.6"><model name="mrm_apriltags"><static>true</static>'
                '<link name="l"/></model></sdf>\n')
    print(f'{len(IDS)} etiket uretildi -> {OUT}')


if __name__ == '__main__':
    main()
