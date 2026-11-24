import math

from mrm_skills.piece_tracker import (PieceTracker, corridor_blocked, footprint_hit,
                                      pose_conflicts, to_local)


def test_to_local():
    bx, by = to_local(1.0, 1.0, 1.0, 0.0, math.pi / 2)
    assert abs(bx - 1.0) < 1e-9 and abs(by) < 1e-9


def test_footprint():
    assert footprint_hit(0.08, 0.0)           # önde, gövdeye değiyor
    assert not footprint_hit(0.21, 0.0)       # kavrama mesafesi: temas yok
    assert footprint_hit(-0.22, 0.10)         # arka köşe
    assert not footprint_hit(0.0, 0.25)


def test_pose_conflicts():
    pts = [(1.0, 0.0)]
    assert pose_conflicts(pts, 0.95, 0.0, 0.0)
    assert not pose_conflicts(pts, 0.70, 0.0, 0.0)


def test_tracker_gating_and_median():
    t = PieceTracker()
    t.start('red', (1.0, 1.0), gate=0.15)
    # komşu aynı renk küp kapı dışında -> seçilmez; hedef ölçümleri medyanlanır
    for k in range(6):
        t.update(k * 0.1, [('red', 1.3, 1.0, 0.5), ('red', 1.01 + 0.001 * k, 0.99, 0.5),
                           ('blue', 1.0, 1.0, 0.5)])
    s = t.snapshot()
    assert s['n_obs'] == 6
    assert abs(s['est'][0] - 1.0125) < 0.003 and abs(s['est'][1] - 0.99) < 1e-6
    # aykırı ölçüm (kapı içinde ama tek) medyanı bozmaz
    t.update(1.0, [('red', 1.05, 1.0, 0.5)])
    assert abs(t.snapshot()['est'][0] - 1.013) < 0.004
    assert t.drift() < 0.01


def test_corridor():
    # hedef (0,0), robot +x yönünden yanaşıyor (standoff_dir = 0): koridor x∈[~0.1, ~0.75]
    assert corridor_blocked((0, 0), 0.0, 0.5, [(0.35, 0.05)])
    assert not corridor_blocked((0, 0), 0.0, 0.5, [(-0.3, 0.0)])     # hedefin arkası
    assert not corridor_blocked((0, 0), 0.0, 0.5, [(0.35, 0.30)])    # yanda
