"""JT128 raycaster with hits on the surface, for the L2 render node (sim only).

~/sim/marsim/jt128_raycast.py counts a ray as a hit once the distance field is within one voxel
(VOX, 0.05 m, from voxel corners): every return lands 0.05-0.1 m in front of its surface, and a ray
grazing an edge within that registers on it. Walls came out ~0.1 m toward the robot and the Gate
L2 course's 0.90 m door ~0.7 m wide, 0.6 m of free cells in the track planner's live grid (d8: no
path through it from an angled approach). A real JT128 returns on the surface (+-2 cm). Here a ray
is a hit only inside an occupied voxel: it closes in by at most half a voxel per step, so a ray
passing an edge at 0.05 m goes by, and the return lies at the voxel it enters (<= 0.05 m in front)."""
import numpy as np

from jt128_raycast import JT128Raycaster, VOX, MAX_R, MIN_R


class SurfaceRaycaster(JT128Raycaster):
    def scan(self, origin, R, iters=128):
        d = self.rays @ R.T
        t = np.full(len(d), MIN_R)
        alive = np.ones(len(d), bool)
        hit = np.zeros(len(d), bool)
        for _ in range(iters):
            a = np.where(alive)[0]
            if not len(a):
                break
            p = origin + d[a] * t[a, None]
            g = np.floor((p - self.lo) / VOX).astype(int)
            inside = np.all((g >= 0) & (g < self.dims), axis=1)
            dist = np.full(len(a), np.inf, np.float32)
            gi = g[inside]
            dist[inside] = self.df[gi[:, 0], gi[:, 1], gi[:, 2]]
            h = dist < VOX * 0.5                       # in an occupied voxel (the field is 0 there)
            hit[a[h]] = True
            step = np.maximum(dist - VOX, VOX * 0.5)   # never past the nearest occupied voxel
            t[a] += np.where(h, 0, step)
            alive[a[h | ~inside]] = False
            alive &= t <= MAX_R
        return origin + d[hit] * t[hit, None]
