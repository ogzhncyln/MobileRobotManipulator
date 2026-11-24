"""
OpenMANIPULATOR-X için ANALİTİK ters kinematik (IK).

Neden analitik IK?
  Kol 4 serbestlik derecelidir (joint1 yaw + 3 adet pitch eklemi). Genel bir 6-DOF
  hedef pozun (x, y, z, roll, pitch, yaw) çoğuna ulaşamaz. MoveIt'in KDL çözücüsü
  bu yüzden 'position_only_ik' modunda çalışır ve gripper'ın açısını kontrol
  edemez. Biz ise "üstten dik tut", "yatay it" gibi açı kontrollü hareketler
  istiyoruz. Bu kolun geometrisi basit olduğu için kapalı formda çözülebilir:

  1. joint1 (yaw) hedefin yatay düzlemdeki yönünü verir:
        q1 = atan2(y, x - x_j1)
  2. Geriye kalan 3 pitch eklemi, joint1 ile dönen DİKEY DÜZLEMDE bir 3R
     düzlemsel koldur. Uç efektör açısı ψ (yataydan, aşağısı negatif) seçilirse
     bilek noktası W = hedef - L3·(cos ψ, sin ψ) olur ve problem
     klasik 2R (iki uzuvlu) düzlemsel IK'ya indirgenir (kosinüs teoremi).

Uzunluklar URDF'ten (open_manipulator_x.urdf.xacro):
  joint2 -> joint3 : (0.024, 0.128)  => L1 = 0.1302, ofset açısı φ1 = 1.3854 rad
  joint3 -> joint4 : 0.124           => L2
  joint4 -> uç     : 0.126           => L3  (end_effector_link)

Çerçeve: base_footprint (x ileri, z yukarı).
  joint1 ekseni: x = -0.080 ;  joint2 yüksekliği: z = 0.1775
"""

import math

X_J1 = -0.080
Z_J2 = 0.1775
L1 = math.hypot(0.024, 0.128)
PHI1 = math.atan2(0.128, 0.024)
L2 = 0.124
L3 = 0.126

LIMITS = {
    'joint1': (-math.pi * 0.9, math.pi * 0.9),
    'joint2': (-math.pi * 0.57, math.pi * 0.5),
    'joint3': (-math.pi * 0.3, math.pi * 0.44),
    'joint4': (-math.pi * 0.57, math.pi * 0.65),
}
JOINTS = ['joint1', 'joint2', 'joint3', 'joint4']

# Hazır kol pozları (SRDF'teki 'home' ile aynı): gripper tabanın üstünde,
# kamera görüş alanının dışında; taşınan küp de lidar/kamera görüşünü kapatmaz.
HOME = [0.0, -1.0, 0.7, 0.3]


def fk(q):
    """İleri kinematik: eklem açıları -> (x, y, z, ψ) base_footprint'te."""
    q1, q2, q3, q4 = q
    a1 = PHI1 - q2              # L1'in yataydan mutlak açısı
    a2 = -(q2 + q3)             # L2'nin mutlak açısı
    psi = -(q2 + q3 + q4)       # Uç efektör mutlak açısı
    r = L1 * math.cos(a1) + L2 * math.cos(a2) + L3 * math.cos(psi)
    z = L1 * math.sin(a1) + L2 * math.sin(a2) + L3 * math.sin(psi)
    return (X_J1 + r * math.cos(q1), r * math.sin(q1), Z_J2 + z, psi)


def within_limits(q):
    return all(LIMITS[n][0] - 1e-6 <= v <= LIMITS[n][1] + 1e-6 for n, v in zip(JOINTS, q))


def ik(x, y, z, psi, elbow_up=True):
    """Tek bir uç efektör açısı ψ için IK. Çözüm yoksa None döner."""
    q1 = math.atan2(y, x - X_J1)
    r = math.hypot(x - X_J1, y)
    zr = z - Z_J2
    rw = r - L3 * math.cos(psi)
    zw = zr - L3 * math.sin(psi)
    d2 = rw * rw + zw * zw
    c = (d2 - L1 * L1 - L2 * L2) / (2 * L1 * L2)
    if abs(c) > 1.0:
        return None  # Bilek noktası erişim dışında
    g = math.acos(c) * (1 if elbow_up else -1)    # dirsek açısı γ = a1 - a2
    a1 = math.atan2(zw, rw) + math.atan2(L2 * math.sin(g), L1 + L2 * math.cos(g))
    a2 = a1 - g
    q2 = PHI1 - a1
    q3 = -a2 - q2
    q4 = -psi - q2 - q3
    q = [q1, q2, q3, q4]
    return q if within_limits(q) else None


def ik_search(x, y, z, psi_candidates):
    """ψ adaylarını sırayla dener, ilk geçerli çözümü (q, ψ) döndürür."""
    for psi in psi_candidates:
        for elbow in (True, False):
            q = ik(x, y, z, psi, elbow)
            if q is not None:
                return q, psi
    return None, None


# Görevlere göre tercih edilen uç efektör açıları
PSI_TOP_DOWN = [-math.pi / 2 + k * 0.1 for k in range(0, 11)]     # -90° .. -33°
PSI_PLACE = [-math.pi / 2 + k * 0.1 for k in range(0, 16)]        # -90° .. ~0°
PSI_PRESS = [0.0, -0.1, 0.1, -0.2, 0.2, -0.3]                     # yatay itme
