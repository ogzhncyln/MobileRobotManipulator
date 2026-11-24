import math
import random

from mrm_skills import arm_ik as K


def test_fk_ik_roundtrip():
    rng = random.Random(0)
    ok = 0
    for _ in range(2000):
        q = [rng.uniform(*K.LIMITS[n]) for n in K.JOINTS]
        x, y, z, psi = K.fk(q)
        for elbow in (True, False):
            sol = K.ik(x, y, z, psi, elbow)
            if sol is not None:
                xs, ys, zs, ps = K.fk(sol)
                assert abs(xs - x) < 1e-6 and abs(ys - y) < 1e-6 and abs(zs - z) < 1e-6
                assert abs(math.remainder(ps - psi, 2 * math.pi)) < 1e-6
                ok += 1
    assert ok > 1000


def test_task_poses_reachable():
    # Yerdeki küpü üstten tutma (küp merkezi 0.0175 m)
    q, psi = K.ik_search(0.20, 0.0, 0.02, K.PSI_TOP_DOWN)
    assert q is not None
    # Kutuya bırakma: kutu merkezi 0.24 m önde, 0.15 m yükseklik
    q, psi = K.ik_search(0.24, 0.0, 0.15, K.PSI_PLACE)
    assert q is not None
    # Butona yatay basma: 0.27 m önde, 0.20 m yükseklik
    q, psi = K.ik_search(0.27, 0.0, 0.20, K.PSI_PRESS)
    assert q is not None
