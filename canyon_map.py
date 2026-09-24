"""
Χάρτης «πόσο urban canyon είναι εδώ», από τον χάρτη κτηρίων του LiDAR.

Για κάθε σημείο δρόμου κοιτάμε γύρω-γύρω (36 κατευθύνσεις, ανά 10°) και βρίσκουμε
σε κάθε κατεύθυνση πόσο ψηλά «κρύβουν τον ουρανό» τα κτήρια: τη γωνία ανύψωσης
atan(ύψος / απόσταση) του πιο «ψηλού» κτηρίου σε εκείνη την κατεύθυνση.

Μέτρο: ποσοστό των κατευθύνσεων όπου τα κτήρια κρύβουν τον ουρανό πάνω από 15°.
Τα 15° είναι η γωνία αποκοπής δορυφόρων (elevation mask) της βιβλιογραφίας
([G], [D] στο GPS_RESEARCH_REPORT.md): δορυφόροι πιο χαμηλά δεν χρησιμοποιούνται ούτως ή άλλως,
άρα ό,τι κρύβεται πάνω από 15° είναι ουρανός που χάνει ο δέκτης.

  0%  → ανοιχτός ουρανός
 50%  → ο μισός ορίζοντας κρυμμένος (π.χ. κτήρια στη μία πλευρά του δρόμου)
100%  → κτήρια παντού γύρω

Περιορισμός: το LiDAR βλέπει έως ~107 m ύψος (upper_fov 60°), οπότε για τους πολύ ψηλούς
ουρανοξύστες η γωνία υποεκτιμάται — δηλαδή το μέτρο είναι συντηρητικό.

Χωρίς CARLA:
    python canyon_map.py [town10_lidar_map.npz]
"""

import json
import os
import sys

import matplotlib.pyplot as plt
import numpy as np
from PIL import Image

HERE = os.path.dirname(os.path.abspath(__file__))
CELL = 1.0             # m — πλέγμα υψών
MASK_DEG = 15.0        # γωνία αποκοπής δορυφόρων
AZIMUTHS = 36          # κατευθύνσεις γύρω από κάθε σημείο
MAX_DIST = 120.0       # m — πόσο μακριά κοιτάμε
ANTENNA_Z = 2.0        # m — ύψος κεραίας GPS πάνω στο αυτοκίνητο
ROAD_STEP = 3.0        # m — απόσταση ανάμεσα στα σημεία δρόμου που εξετάζουμε


def height_grid(points):
    """Μέγιστο ύψος κτηρίου σε κάθε κελί 1 m × 1 m (0 όπου δεν υπάρχει κτήριο)."""
    x0, y0 = points[:, 0].min(), points[:, 1].min()
    ix = ((points[:, 0] - x0) / CELL).astype(int)
    iy = ((points[:, 1] - y0) / CELL).astype(int)
    grid = np.zeros((ix.max() + 1, iy.max() + 1), dtype=np.float32)
    np.maximum.at(grid, (ix, iy), points[:, 2])
    return grid, x0, y0


def road_points():
    """Σημεία δρόμου σε x/y του CARLA, από το town10_grid.npy (y ανάποδα από το OpenDRIVE)."""
    grid = np.load(os.path.join(HERE, "town10_grid.npy"))
    x0, y0, res = np.load(os.path.join(HERE, "town10_meta.npy"))
    step = int(ROAD_STEP / res)
    rows, cols = np.nonzero(grid[::step, ::step])
    return x0 + cols * step * res, -(y0 + rows * step * res)


def blocked_fraction(rx, ry, grid, gx0, gy0):
    """Για κάθε σημείο δρόμου: ποσοστό κατευθύνσεων με εμπόδιο πάνω από MASK_DEG."""
    angles = np.linspace(0, 2 * np.pi, AZIMUTHS, endpoint=False)
    dists = np.arange(CELL, MAX_DIST, CELL / 2)
    max_elev = np.zeros((len(rx), AZIMUTHS))
    for a, ang in enumerate(angles):
        # όλα τα σημεία κατά μήκος της ακτίνας, για όλα τα σημεία δρόμου μαζί
        px = rx[:, None] + np.cos(ang) * dists[None, :]
        py = ry[:, None] + np.sin(ang) * dists[None, :]
        ix = ((px - gx0) / CELL).astype(int)
        iy = ((py - gy0) / CELL).astype(int)
        inside = (ix >= 0) & (ix < grid.shape[0]) & (iy >= 0) & (iy < grid.shape[1])
        h = np.where(inside, grid[np.clip(ix, 0, grid.shape[0] - 1), np.clip(iy, 0, grid.shape[1] - 1)], 0)
        elev = np.degrees(np.arctan2(h - ANTENNA_Z, dists[None, :]))
        max_elev[:, a] = elev.max(axis=1)
    return (max_elev > MASK_DEG).mean(axis=1)


if __name__ == "__main__":
    path = sys.argv[1] if len(sys.argv) > 1 else os.path.join(HERE, "town10_lidar_map.npz")
    points = np.load(path)["points"]

    grid, gx0, gy0 = height_grid(points)
    rx, ry = road_points()
    blocked = blocked_fraction(rx, ry, grid, gx0, gy0)

    np.savez_compressed(os.path.join(HERE, "town10_canyon.npz"), x=rx, y=ry, blocked=blocked)
    print(f"{len(rx)} σημεία δρόμου")
    for lo, hi in [(0, .25), (.25, .5), (.5, .75), (.75, 1.01)]:
        print(f"  κρυμμένος ορίζοντας {lo:4.0%}–{min(hi, 1):4.0%}: {np.mean((blocked >= lo) & (blocked < hi)):5.1%} των σημείων")

    with open(os.path.join(HERE, "town10_minimap.json")) as f:
        meta = json.load(f)
    image = Image.open(os.path.join(HERE, "town10_minimap.png"))
    ppm, mx, my = meta["pixels_per_meter"], meta["min_x"], meta["min_y"]

    fig, ax = plt.subplots(figsize=(13, 12))
    ax.imshow(image, extent=[mx, mx + image.size[0] / ppm, my + image.size[1] / ppm, my])
    sc = ax.scatter(rx, ry, c=100 * blocked, s=9, cmap="RdYlGn_r", vmin=0, vmax=100)
    plt.colorbar(sc, ax=ax, fraction=0.04, label=f"% ορίζοντα κρυμμένο πάνω από {MASK_DEG:.0f}°")
    ax.set_xlim(mx, mx + image.size[0] / ppm)
    ax.set_ylim(my + image.size[1] / ppm, my)
    ax.set_aspect("equal")
    ax.set_xlabel("CARLA x (m)")
    ax.set_ylabel("CARLA y (m)")
    ax.set_title("Πόσο urban canyon: κρυμμένος ουρανός πάνω από 15° (από LiDAR)")
    plt.tight_layout()
    plt.savefig(os.path.join(HERE, "town10_canyon.png"), dpi=80)
    print("Αποθηκεύτηκαν: town10_canyon.npz, town10_canyon.png")
