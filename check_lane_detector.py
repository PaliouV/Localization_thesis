"""
Έλεγχος του ανιχνευτή λωρίδων (lane_detector.py) στο Mac: συγκρίνει τις αποστάσεις που βρήκε από τις εικόνες
με τις αληθινές, από τον χάρτη λωρίδων (lane_map.py) στην αληθινή θέση του αυτοκινήτου.

    python3 check_lane_detector.py drive_2026-10-30_20-45-55

Διαβάζει drives/<καταγραφή>.csv (αλήθεια) και drives/<καταγραφή>_lanes.csv (ανιχνευτής), ενωμένα με τη στήλη frame.
Τυπώνει, για κάθε πλευρά:
  - πόσο συχνά ο χάρτης λέει «βαμμένη γραμμή» και πόσο συχνά την είδε ο ανιχνευτής
  - το λάθος (ανιχνευτής − χάρτης): μέσο, τυπική απόκλιση, πόσες φορές πάνω από 30 cm
  - πόσες φορές «είδε» γραμμή εκεί που ο χάρτης λέει ότι δεν υπάρχει βαφή
Συνθετική δοκιμή (9 Νοε, εικόνες ζωγραφισμένες από τον χάρτη): τυπ. απόκλιση 4,5 cm (αριστερά), 14 cm (δεξιά).
"""

import os
import sys

import numpy as np

from lane_map import expected_line_distances

HERE = os.path.dirname(os.path.abspath(__file__))
drive = sys.argv[1][:-4] if sys.argv[1].endswith(".csv") else sys.argv[1]
truth = np.genfromtxt(os.path.join(HERE, "drives", drive + ".csv"), delimiter=",", names=True)
det = np.genfromtxt(os.path.join(HERE, "drives", drive + "_lanes.csv"), delimiter=",", names=True)

row_of_frame = {int(f): k for k, f in enumerate(truth["frame"])}
k = np.array([row_of_frame.get(int(f), -1) for f in det["frame"]])
print(f"{drive}: {len(det)} εικόνες, {np.sum(k < 0)} χωρίς γραμμή στο CSV της καταγραφής")
det, k = det[k >= 0], k[k >= 0]

left, right, left_painted, right_painted = expected_line_distances(
    truth["gt_x"][k], truth["gt_y"][k], np.radians(truth["gt_yaw"][k]))

for side, expected, painted in [("left", left, left_painted), ("right", right, right_painted)]:
    ok = det[f"{side}_ok"] == 1
    error = det[f"{side}_dist"][ok] - expected[ok]
    name = "αριστερά" if side == "left" else "δεξιά"
    print(f"\n{name}: βαμμένη στον χάρτη {100 * painted.mean():.0f}%, ο ανιχνευτής είδε {100 * ok.mean():.0f}% των εικόνων")
    if ok.any():
        print(f"  λάθος (ανιχνευτής − χάρτης): μέσο {100 * error.mean():+.1f} cm, τυπ. απόκλιση {100 * error.std():.1f} cm, "
              f"διάμεσο |λάθος| {100 * np.median(np.abs(error)):.1f} cm")
        print(f"  πάνω από 30 cm: {100 * np.mean(np.abs(error) > 0.3):.1f}%, πάνω από 1 m: {100 * np.mean(np.abs(error) > 1.0):.1f}%")
        print(f"  είδε γραμμή εκεί που ο χάρτης λέει άβαφη: {np.sum(ok & ~painted)} εικόνες")

both = (det["left_ok"] == 1) & (det["right_ok"] == 1)
if both.any():
    width = det["left_dist"][both] + det["right_dist"][both]
    print(f"\nπλάτος λωρίδας (αριστερά + δεξιά, όταν είδε και τις δύο): διάμεσος {np.median(width):.3f} m (πρέπει ~3,5)")
    print("  Αν είναι π.χ. 3,8 αντί για 3,5, το ύψος της κάμερας στο camera_front_geometry.json είναι λάθος κατά 3,8/3,5.")
