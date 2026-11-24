"""
HEDEF KÜP TAKİBİ ve GÖVDE ÇARPIŞMA KONTROLÜ (ROS'tan bağımsız, birim test edilebilir).

Neden odom çerçevesi?
  * base_footprint robotla birlikte hareket eder: robot sürerken eski bir ölçüm anlamsızlaşır.
  * map (AMCL) yavaş güncellenir ve sıçrayabilir (birkaç cm).
  * odom (tekerlek odometrisi) kısa mesafede ~1-2 mm doğrudur ve SÜREKLİDİR. Durağan bir küp,
    robot hareket etse bile odom'da sabit kalır. Algılayıcı her ölçümü GÖRÜNTÜ ANINDAKİ odom
    dönüşümüyle verdiği için gecikmeli kareler de doğru yere düşer.

Takipçi:
  * Her karede hedefin rengindeki algılardan, tahmine EN YAKIN ve kapı (gate) içinde olanı
    seçer -> komşu küplerle karışmaz.
  * Tahmin = son N ölçümün medyanı (tek tük hatalı ölçümlere dayanıklı).
  * Küp kameranın kör bölgesine (<~0.24 m) girince ölçüm gelmez; tahmin odom'da sabit
    kaldığı için robotun güncel odometrisiyle base çerçevesine çevrilerek kullanılmaya devam eder.
"""

import math
import threading

import numpy as np

# TurtleBot3 Waffle gövdesi (base_footprint çerçevesi, URDF çarpışma kutusu + tekerlekler)
BODY_X_MIN = -0.197
BODY_X_MAX = 0.069
BODY_HALF_W = 0.153        # tekerlek dış yüzü
PIECE_HALF_DIAG = 0.025    # 3.5 cm küpün yarım köşegeni


def to_local(px, py, rx, ry, ryaw):
    """Dünya (odom/map) noktasını robotun (rx, ry, ryaw) çerçevesine çevir."""
    dx, dy = px - rx, py - ry
    c, s = math.cos(ryaw), math.sin(ryaw)
    return c * dx + s * dy, -s * dx + c * dy


def footprint_hit(bx, by, margin=0.02):
    """Robot çerçevesindeki (bx, by) küp merkezi gövdeye 'margin' kadar yakın mı?"""
    m = margin + PIECE_HALF_DIAG
    return (BODY_X_MIN - m <= bx <= BODY_X_MAX + m) and abs(by) <= BODY_HALF_W + m


def pose_conflicts(points, rx, ry, ryaw, margin=0.02):
    """Verilen poza gelindiğinde gövdenin içine/kenarına düşen noktalar."""
    out = []
    for p in points:
        bx, by = to_local(p[0], p[1], rx, ry, ryaw)
        if footprint_hit(bx, by, margin):
            out.append(p)
    return out


def corridor_blocked(target, standoff_dir, dist, others, final_x=0.21, half_w=0.20):
    """
    Hedefe 'standoff_dir' yönünden (hedeften robota doğru birim açı) düz yanaşırken
    gövdenin süpüreceği koridorda başka küp var mı?
    Koridor: robot son pozda (küp önünde final_x) iken ön yüzünden, başlangıç pozunun
    arka yüzüne kadar; yanal ±half_w.
    """
    ux, uy = math.cos(standoff_dir), math.sin(standoff_dir)
    s_min = final_x - BODY_X_MAX - PIECE_HALF_DIAG - 0.02      # robot önü (son pozda)
    s_max = dist - BODY_X_MIN + PIECE_HALF_DIAG + 0.02          # robot arkası (başlangıçta)
    for ox, oy in others:
        dx, dy = ox - target[0], oy - target[1]
        s = dx * ux + dy * uy
        lat = -dx * uy + dy * ux
        if s_min <= s <= s_max and abs(lat) <= half_w:
            return True
    return False


class PieceTracker:
    """Tek bir hedef küpün odom çerçevesinde takibi."""

    def __init__(self, history=9):
        self.lock = threading.Lock()
        self.history = history
        self.active = False
        self.color = None
        self.est = None             # (x, y) odom
        self.samples = []           # (stamp, x, y, base_x)
        self.first = None           # ilk 'kilitli' tahmin (itilme teşhisi için)
        self.last_seen = None
        self.n_obs = 0
        self.gate_init = 0.15
        self.gate_locked = 0.06

    def start(self, color, guess_odom, gate=0.15):
        with self.lock:
            self.active = True
            self.color = color
            self.est = (float(guess_odom[0]), float(guess_odom[1]))
            self.samples = []
            self.first = None
            self.last_seen = None
            self.n_obs = 0
            self.gate_init = gate

    def stop(self):
        with self.lock:
            self.active = False

    def gate(self):
        return self.gate_locked if self.n_obs >= 4 else self.gate_init

    def update(self, stamp, dets):
        """dets: [(color, ox, oy, base_x), ...] tek bir karedeki algılar."""
        with self.lock:
            if not self.active:
                return False
            best = None
            for color, ox, oy, bx in dets:
                if color != self.color:
                    continue
                d = math.hypot(ox - self.est[0], oy - self.est[1])
                if d < self.gate() and (best is None or d < best[0]):
                    best = (d, ox, oy, bx)
            if best is None:
                return False
            self.samples.append((stamp, best[1], best[2], best[3]))
            self.samples = self.samples[-self.history:]
            xs = [s[1] for s in self.samples]
            ys = [s[2] for s in self.samples]
            self.est = (float(np.median(xs)), float(np.median(ys)))
            self.n_obs += 1
            self.last_seen = stamp
            if self.n_obs == 4:
                self.first = self.est
            return True

    def snapshot(self):
        with self.lock:
            return dict(est=self.est, n_obs=self.n_obs, last_seen=self.last_seen,
                        first=self.first, spread=self._spread())

    def _spread(self):
        if len(self.samples) < 3:
            return None
        xs = np.array([s[1] for s in self.samples])
        ys = np.array([s[2] for s in self.samples])
        return float(np.hypot(xs - np.median(xs), ys - np.median(ys)).max())

    def drift(self):
        """İlk kilitli tahminden bu yana tahminin kayması (küp itildi mi?)."""
        with self.lock:
            if self.first is None or self.est is None:
                return 0.0
            return math.hypot(self.est[0] - self.first[0], self.est[1] - self.first[1])
