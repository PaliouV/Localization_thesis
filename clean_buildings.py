"""
Καθαρίζει το town10_buildings.json (έξοδος του export_buildings.py) και γράφει
το town10_buildings_clean.json με ΜΟΝΟ πραγματικά κτήρια.

Τι κάνει:
  1. Απόλυτη τιμή στις διαστάσεις — το CARLA δίνει κάποιες αρνητικές (π.χ. Block14).
  2. Ενώνει τα κομμάτια κάθε "ProceduralBuilding<N>_Inst_..." σε ένα κτήριο.
  3. Κρατάει ό,τι έχει ύψος > MIN_HEIGHT και βάση > MIN_BASE, εκτός από δέντρα.
  4. Για κάθε κτήριο γράφει και την απόσταση από τον πλησιέστερο δρόμο (road_dist).

Τα όρια (5 m, 50 m²) ΔΕΝ είναι από βιβλιογραφία — διαλέχτηκαν από το ιστόγραμμα
υψών: κάτω από ~3 m είναι κάγκελα, κλιματιστικά, τέντες, πανό κ.λπ.

Δεν χρειάζεται CARLA:
    python clean_buildings.py
"""

import json
import math
import os
import re
from collections import defaultdict

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
MIN_HEIGHT = 5.0     # m
MIN_BASE = 50.0      # m²
JUNK = re.compile(r"InstancedFoliageActor")   # δέντρα που το CARLA βάζει στα "Buildings"


def corners(b):
    """Οι 4 γωνίες της βάσης του κουτιού, σε x/y του CARLA."""
    c, s = math.cos(math.radians(b["yaw"])), math.sin(math.radians(b["yaw"]))
    ex, ey = b["extent_x"], b["extent_y"]
    return [(b["x"] + c * px - s * py, b["y"] + s * px + c * py)
            for px, py in [(ex, ey), (ex, -ey), (-ex, -ey), (-ex, ey)]]


def merge(name, pieces):
    """Ένα κουτί (ευθυγραμμισμένο με τους άξονες) που περικλείει όλα τα κομμάτια."""
    pts = [p for b in pieces for p in corners(b)]
    xs, ys = [p[0] for p in pts], [p[1] for p in pts]
    bottom = min(b["z"] - b["extent_z"] for b in pieces)
    top = max(b["z"] + b["extent_z"] for b in pieces)
    return dict(id=pieces[0]["id"], name=name, pieces=len(pieces),
                x=(min(xs) + max(xs)) / 2, y=(min(ys) + max(ys)) / 2, z=(bottom + top) / 2,
                extent_x=(max(xs) - min(xs)) / 2, extent_y=(max(ys) - min(ys)) / 2,
                extent_z=(top - bottom) / 2, yaw=0.0, height=top - bottom, top_z=top)


def road_points():
    """Σημεία δρόμου σε x/y του CARLA, από το town10_grid.npy (y ανάποδα από το OpenDRIVE)."""
    grid = np.load(os.path.join(HERE, "town10_grid.npy"))
    x0, y0, res = np.load(os.path.join(HERE, "town10_meta.npy"))
    rows, cols = np.nonzero(grid[::4, ::4])          # κάθε 1 m αρκεί
    return x0 + cols * 4 * res, -(y0 + rows * 4 * res)


def road_dist(b, rx, ry):
    """Απόσταση από την άκρη του κουτιού ως το πλησιέστερο σημείο δρόμου (0 αν ακουμπάει)."""
    c, s = math.cos(math.radians(b["yaw"])), math.sin(math.radians(b["yaw"]))
    dx, dy = rx - b["x"], ry - b["y"]
    u, v = c * dx + s * dy, -s * dx + c * dy
    out_u = np.maximum(np.abs(u) - b["extent_x"], 0)
    out_v = np.maximum(np.abs(v) - b["extent_y"], 0)
    return float(np.min(np.hypot(out_u, out_v)))


if __name__ == "__main__":
    with open(os.path.join(HERE, "town10_buildings.json")) as f:
        raw = json.load(f)

    # 1. απόλυτες τιμές στις διαστάσεις
    for b in raw["buildings"]:
        for k in ("extent_x", "extent_y", "extent_z"):
            b[k] = abs(b[k])
        b["height"] = 2 * b["extent_z"]

    # 2. ένωση των procedural κτηρίων
    groups, singles = defaultdict(list), []
    for b in raw["buildings"]:
        m = re.match(r"(ProceduralBuilding\d+)_", b["name"])
        if m:
            groups[m.group(1)].append(b)
        else:
            singles.append(dict(b, pieces=1))
    merged = [merge(name, pieces) for name, pieces in groups.items()]

    # 3. φίλτρο
    keep = [b for b in singles + merged
            if b["height"] > MIN_HEIGHT and 4 * b["extent_x"] * b["extent_y"] > MIN_BASE
            and not JUNK.search(b["name"])]

    # 4. απόσταση από δρόμο
    rx, ry = road_points()
    for b in keep:
        b["road_dist"] = road_dist(b, rx, ry)

    with open(os.path.join(HERE, "town10_buildings_clean.json"), "w") as f:
        json.dump(dict(map=raw["map"], buildings=keep), f, indent=1)

    print(f"Από {len(raw['buildings'])} κουτιά → {len(keep)} κτήρια "
          f"({len(merged)} procedural ενώθηκαν)")
    print(f"Έως 40 m από δρόμο: {sum(b['road_dist'] < 40 for b in keep)}")
    print("\nProcedural κτήρια μετά την ένωση:")
    for b in sorted(merged, key=lambda b: b["name"]):
        base = 4 * b["extent_x"] * b["extent_y"]
        flag = ("" if b in keep else
                "  ← πετάχτηκε: χαμηλό" if b["height"] <= MIN_HEIGHT else
                "  ← πετάχτηκε: μικρή βάση")
        print(f"  {b['name']:<22} κομμάτια={b['pieces']:3d}  ύψος={b['height']:5.1f} m  "
              f"βάση={base:6.0f} m²  κέντρο=({b['x']:6.1f}, {b['y']:6.1f}){flag}")
    print("\nΑποθηκεύτηκε: town10_buildings_clean.json")
