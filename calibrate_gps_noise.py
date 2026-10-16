"""
Ελέγχει αν οι ζώνες του gps_noise.py δίνουν λάθος θέσης που μοιάζει με τις
πραγματικές μετρήσεις (GPS_NOISE_MODEL.md §6). Χωρίς CARLA.

Για κάθε ζώνη (STRONG, MODERATE, MILD) και για κάθε λόγο:
  - ένα ψεύτικο όχημα οδηγεί μέσα στη ζώνη, με τις συνθήκες της §5:
    1 μέτρηση/s, 8 m/s, 3600 s, 5 seeds, στροφή 90° κάθε 120 s
  - μετράμε το 2D λάθος (απόσταση GPS ↔ αληθινή θέση) σε κάθε μέτρηση
  - τυπώνουμε μέσο / STD / RMS / P95 / μέγιστο ανά ώρα δίπλα στους στόχους,
    και τον μετρημένο λόγο μέσο|εγκάρσιο| / μέσο|κατά μήκος|

Στόχοι (GPS_NOISE_MODEL.md §6):
  STRONG   ↔ [S4] Πίν. 2, 4: μέσο ~31 m, STD 27–38 m, RMS 41–49 m, μέγιστο 90–180 m
  MODERATE ↔ Suhr 2017 Πίν. II (GANGNAM): μέσο ~14.3 m, STD ~16.1 m
  MILD     ↔ Suhr 2017 Πίν. I (GANGBOOK):  μέσο ~5.3 m,  STD ~9.0 m
  Ανοχή για τα «~»: ±10% (απόφαση 4 Οκτ 2026). Τα εύρη του [S4] ισχύουν όπως είναι.
  Για MODERATE / MILD ο Suhr δεν δίνει μέγιστο, οπότε δεν ελέγχεται.

    python calibrate_gps_noise.py                    # λόγοι 1.2 και 2, TAU 30 s
    python calibrate_gps_noise.py --ratios 2 --tau 120
"""

import argparse
import math

import numpy as np

from gps_noise import MILD, MODERATE, STRONG, GpsNoise

SPEED = 8.0          # m/s
DURATION = 3600      # s
TURN_EVERY = 120     # s
SEEDS = 5

# (όνομα, ζώνη, στόχοι: μέγεθος → (κάτω, πάνω) όριο)
TARGETS = [
    ("STRONG", STRONG, {"μέσο": (31 * 0.9, 31 * 1.1), "STD": (27, 38), "RMS": (41, 49), "max/ώρα": (90, 180)}),
    ("MODERATE", MODERATE, {"μέσο": (14.3 * 0.9, 14.3 * 1.1), "STD": (16.1 * 0.9, 16.1 * 1.1)}),
    ("MILD", MILD, {"μέσο": (5.3 * 0.9, 5.3 * 1.1), "STD": (9.0 * 0.9, 9.0 * 1.1)}),
]


def drive(zone, ratio, tau, seed):
    """Μία ώρα οδήγησης μέσα στη ζώνη. Επιστρέφει λάθος κατά μήκος και εγκάρσια (m) ανά δευτερόλεπτο."""
    everywhere = [dict(zone, x_min=-math.inf, x_max=math.inf, y_min=-math.inf, y_max=math.inf)]
    noise = GpsNoise(zones=everywhere, dt=1.0, seed=seed, ratio=ratio, tau=tau)
    x = y = yaw = 0.0
    along_err, cross_err = [], []
    for t in range(DURATION):
        if t > 0 and t % TURN_EVERY == 0:
            yaw += 90.0
        r = math.radians(yaw)
        x += SPEED * math.cos(r)
        y += SPEED * math.sin(r)
        fix = noise.step(x, y, yaw)
        dx, dy = fix[0] - x, fix[1] - y
        along_err.append(dx * math.cos(r) + dy * math.sin(r))
        cross_err.append(-dx * math.sin(r) + dy * math.cos(r))
    return np.array(along_err), np.array(cross_err)


def stats(zone, ratio, tau):
    along, cross, maxima = [], [], []
    for seed in range(SEEDS):
        a, c = drive(zone, ratio, tau, seed)
        along.append(a)
        cross.append(c)
        maxima.append(np.hypot(a, c).max())
    a, c = np.concatenate(along), np.concatenate(cross)
    err = np.hypot(a, c)
    return {
        "μέσο": err.mean(),
        "STD": err.std(),
        "RMS": math.sqrt(np.mean(err**2)),
        "P95": np.percentile(err, 95),
        "max/ώρα": (min(maxima), max(maxima)),
        "λόγος": np.abs(c).mean() / np.abs(a).mean(),
    }


def check(value, bounds):
    low, high = bounds
    if isinstance(value, tuple):           # μέγιστο: εύρος ανάμεσα στα seeds
        ok = low <= value[0] and value[1] <= high
    else:
        ok = low <= value <= high
    return "ΜΕΣΑ" if ok else "ΕΞΩ"


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--ratios", default="1.2,2", help="λόγοι προς έλεγχο, χωρισμένοι με κόμμα")
    parser.add_argument("--tau", type=float, default=30.0, help="TAU σε s")
    args = parser.parse_args()
    ratios = [float(r) for r in args.ratios.split(",")]

    print(f"TAU = {args.tau:g} s, {SEEDS} seeds × {DURATION} s, {SPEED:g} m/s, στροφή κάθε {TURN_EVERY} s\n")
    for name, zone, targets in TARGETS:
        for ratio in ratios:
            s = stats(zone, ratio, args.tau)
            print(f"{name}  λόγος (σ) = {ratio:g}   μετρημένος λόγος |εγκάρσιο|/|κατά μήκος| = {s['λόγος']:.2f}")
            for key in ["μέσο", "STD", "RMS", "P95", "max/ώρα"]:
                value = s[key]
                shown = f"{value[0]:.0f}–{value[1]:.0f} m" if isinstance(value, tuple) else f"{value:.1f} m"
                if key in targets:
                    low, high = targets[key]
                    print(f"    {key:<8} {shown:>12}   στόχος {low:.1f}–{high:.1f} m   {check(value, targets[key])}")
                else:
                    print(f"    {key:<8} {shown:>12}   (χωρίς στόχο)")
            print()
