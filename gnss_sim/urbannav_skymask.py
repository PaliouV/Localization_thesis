"""
Πόσο ουρανό κρύβουν τα κτίρια στις διαδρομές του UrbanNav, με το ίδιο μέτρο που
χρησιμοποιεί ο χάρτης canyon του Town10 (town10/canyon_map.py): ποσοστό των διευθύνσεων
όπου τα κτίρια κρύβουν τον ουρανό πάνω από 15°.

Αρχεία Skymask του UrbanNav (link στο README γρ. 67–69, στο gnss_sim/urbannav/<διαδρομή>/):
  κεφαλίδα «Latitude(deg),Longitude(deg),Altitude(m),Skymask(1*361,deg)», μετά μία γραμμή ανά σημείο
  (σημεία ανά ~2 m). ΕΡΜΗΝΕΙΑ ΜΑΣ, το README δεν το ορίζει: οι 361 τιμές είναι η γωνία ανύψωσης
  (μοίρες) των κτιρίων ανά αζιμούθιο 0°, 1°, …, 360°. Στηρίζεται στο ότι οι τιμές είναι 0–89 και
  η 1η ίδια με την 361η στο ~68% των σημείων. Το Altitude είναι 5 m σε όλα τα σημεία· τι
  ύψος κεραίας αντιστοιχεί, δεν λέγεται.

Για κάθε δευτερόλεπτο της πραγματικής θέσης βρίσκουμε το πλησιέστερο σημείο Skymask και
μετράμε το ποσοστό των 360 διευθύνσεων πάνω από 15°. Μετά:
  1. ανά διαδρομή: κατανομή του ποσοστού,
  2. το ίδιο μέτρο για τις ζώνες του Town10,
  3. σφάλμα GPS ανά ομάδα δεκτών, (α) ανά διαδρομή και (β) ανά «κουτί» κάλυψης ουρανού, με όλα
     τα δευτερόλεπτα των τριών διαδρομών μαζί. Το σφάλμα διαβάζεται από τα errors_<δέκτης>.csv
     (πρώτα: python gnss_sim/urbannav_errors.py medium | deep | harsh).
Το ίδιο μέτρο για τις ζώνες (από το town10/town10_canyon.npz). Οι δύο υπολογισμοί ΔΕΝ είναι ίδιοι: το Town10
έχει 36 διευθύνσεις, κεραία στα 2 m, ορίζοντα 120 m και LiDAR που βλέπει έως ~107 m ύψος.

    python gnss_sim/urbannav_skymask.py
"""

import math
import os
import sys

import numpy as np
from scipy.spatial import cKDTree

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
import urbannav_errors as un  # noqa: E402
from gps_noise import ZONES, zone_at  # noqa: E402

MASK_DEG = 15.0
BINS = [0.0, 0.5, 0.75, 0.9, 1.0001]          # κουτιά κάλυψης ουρανού
GROUPS = {"F9P": ["ublox.f9p", "ublox.f9p.splitter"],
          "κινητά": ["google.pixel4", "huawei.p40pro", "samsung.note8", "xiaomi.mi8"],
          "M8T": ["ublox.m8t.GC", "ublox.m8t.GEJ", "ublox.m8t.GRJ", "ublox.m8t.GR"]}
FILES = {"medium": "UrbanNav-HK-Medium-Urban-TSTE.csv", "deep": "UrbanNav-HK-Deep-Urban-WP.csv",
         "harsh": "UrbanNav-HK-Harsh-Urban-MK.csv"}


def read_skymask(path):
    """Πίνακας Ν × 364: lat, lon, alt, 361 γωνίες. Γρήγορη ανάγνωση (τα αρχεία είναι 250–340 MB)."""
    with open(path) as f:
        f.readline()
        values = np.fromstring(f.read().replace("\n", ","), sep=",")
    a = values[: len(values) // 364 * 364].reshape(-1, 364)
    return a[~np.isnan(a).any(axis=1)]


def to_xy(lat, lon, lat0, lon0):
    return np.column_stack([np.radians(lon - lon0) * un.R_EARTH * math.cos(math.radians(lat0)),
                            np.radians(lat - lat0) * un.R_EARTH])


def route_coverage(route):
    """Δευτερόλεπτο UTC → ποσοστό κρυμμένου ορίζοντα, πάνω στη διαδρομή."""
    a = read_skymask(os.path.join(HERE, "urbannav", route, FILES[route]))
    lat0, lon0 = a[:, 0].mean(), a[:, 1].mean()
    tree = cKDTree(to_xy(a[:, 0], a[:, 1], lat0, lon0))
    truth = un.read_truth(os.path.join(HERE, "urbannav", route))
    secs = np.array(list(truth))
    t = np.array([v[:2] for v in truth.values()])
    dist, idx = tree.query(to_xy(t[:, 0], t[:, 1], lat0, lon0))
    cover = (a[idx, 3:363] > MASK_DEG).mean(axis=1)    # 360 διευθύνσεις (η 361η = η 1η)
    cover[dist >= 5] = np.nan
    return dict(zip(secs, cover)), a[idx, 3:363][dist < 5]


def errors(route, cover):
    """Για κάθε ομάδα δεκτών: (2D σφάλμα, κάλυψη) για όλα τα δευτερόλεπτα με θέση."""
    out = {}
    for group, receivers in GROUPS.items():
        e, c = [], []
        for rec in receivers:
            path = os.path.join(HERE, "urbannav", route, f"errors_{rec}.csv")
            if not os.path.exists(path):
                continue
            r = np.genfromtxt(path, delimiter=",", skip_header=1)
            ok = ~np.isnan(r[:, 1])
            e.append(np.hypot(r[ok, 1], r[ok, 2]))
            c.append(np.array([cover.get(round(t), np.nan) for t in r[ok, 0]]))
        out[group] = (np.concatenate(e), np.concatenate(c))
    return out


def fmt(e):
    return f"{e.mean():6.1f} / {np.median(e):5.1f} m" if len(e) else "      —        "


if __name__ == "__main__":
    print(f"1. % ορίζοντα κρυμμένο πάνω από {MASK_DEG:.0f}°, πάνω στη διαδρομή του οχήματος:\n")
    data, cov = {}, {}
    for route in FILES:
        cover, masks = route_coverage(route)
        b = np.array([v for v in cover.values() if not np.isnan(v)])
        cov[route] = b
        h = np.histogram(b, bins=BINS)[0] / len(b) * 100
        print(f"  {un.ROUTES[route]:<32} μέσο {100 * b.mean():3.0f}%  διάμεσο {100 * np.median(b):3.0f}%  "
              f"ελάχιστο {100 * b.min():3.0f}%  μέση γωνία {masks.mean():2.0f}°   "
              f"κουτιά <50/50–75/75–90/90–100%: " + " / ".join(f"{x:.0f}%" for x in h) + f" των {len(b)} s")
        data[route] = errors(route, cover)

    c = np.load(os.path.join(os.path.dirname(HERE), "town10", "town10_canyon.npz"))
    names = np.array([zone_at(x, y, ZONES).get("name", "open sky") for x, y in zip(c["x"], c["y"])])
    print("\n2. Town10, ζώνες του gps_noise.py (town10/canyon_map.py):\n")
    for z in dict.fromkeys(z["name"] for z in ZONES):
        sel = names == z
        if sel.any():
            print(f"  {z:<32} μέσο {100 * c['blocked'][sel].mean():3.0f}%   ({sel.sum()} σημεία δρόμου)")
    print(f"  {'όλο το Town10':<32} μέσο {100 * c['blocked'].mean():3.0f}%")

    print("\n3α. Σφάλμα GPS ανά διαδρομή (μέσο / διάμεσο, όλοι οι δέκτες της ομάδας μαζί):\n")
    print(f"  {'':<32}" + "".join(f"{g:>22}" for g in GROUPS))
    for route in FILES:
        print(f"  {un.ROUTES[route]:<32}" + "".join(f"{fmt(data[route][g][0]):>22}" for g in GROUPS))

    print("\n3β. Σφάλμα GPS ανά κουτί κάλυψης ουρανού (οι τρεις διαδρομές μαζί):\n")
    print(f"  {'κάλυψη':<12}{'δευτ. (ανά διαδρομή M/D/H)':>30}" + "".join(f"{g:>22}" for g in GROUPS))
    for lo, hi in zip(BINS[:-1], BINS[1:]):
        per_route = [int(np.sum((cov[r] >= lo) & (cov[r] < hi))) for r in FILES]
        cells = []
        for g in GROUPS:
            e = np.concatenate([data[r][g][0][(data[r][g][1] >= lo) & (data[r][g][1] < hi)] for r in FILES])
            cells.append(f"{fmt(e):>22}")
        label = f"{100 * lo:.0f}–{min(100, 100 * hi):.0f}%"
        print(f"  {label:<12}{' / '.join(str(n) for n in per_route):>30}" + "".join(cells))
