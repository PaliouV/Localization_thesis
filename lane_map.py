"""
Ο χάρτης λωρίδων του Town10, για τη μέτρηση της κάμερας (λωρίδες) στο particle_filter.py.

Βήμα 1 (9 Νοε 2026): διαβάζει τον χάρτη και βρίσκει, για οποιοδήποτε σημείο, το πιο κοντινό σημείο του χάρτη.

Ο χάρτης (town10/town10_lanes.csv, από το town10/export_lanes.py): ένα σημείο κάθε 1 m, στο ΚΕΝΤΡΟ κάθε λωρίδας.
Για κάθε σημείο: θέση (x, y), κατεύθυνση (yaw), πλάτος, πού είναι η αριστερή και η δεξιά γραμμή, και αν είναι
βαμμένες (left_marking / right_marking: Solid, Broken, SolidSolid ή NONE).

Δοκιμή:
    python3 lane_map.py
"""

import os

import numpy as np
from scipy.spatial import cKDTree

# Ο χάρτης διαβάζεται μία φορά, όταν γίνει import το αρχείο.
# dtype=None, encoding="utf-8": ο χάρτης έχει και κείμενο (π.χ. "Solid", "True"), όχι μόνο αριθμούς.
MAP_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "town10", "town10_lanes.csv")
LANES = np.genfromtxt(MAP_FILE, delimiter=",", names=True, dtype=None, encoding="utf-8")

# Τα κέντρα των λωρίδων: πίνακας με δύο στήλες (x, y), μία γραμμή για κάθε σημείο του χάρτη.
CENTERS = np.column_stack((LANES["x"], LANES["y"]))

# KD-tree: «ευρετήριο» των σημείων, για να βρίσκουμε γρήγορα το πιο κοντινό (χωρίς να ψάχνουμε και τα 6042).
TREE = cKDTree(CENTERS)


def nearest_lane_point(x, y):
    """Το πιο κοντινό σημείο του χάρτη στο (x, y). Επιστρέφει (απόσταση σε m, θέση του σημείου στον LANES).

    Δουλεύει και για πολλά σημεία μαζί (π.χ. όλα τα particles): x, y πίνακες → απόσταση, θέση πίνακες.
    """
    points = np.column_stack((np.atleast_1d(x), np.atleast_1d(y)))
    distance, index = TREE.query(points)
    if np.ndim(x) == 0:          # ένα σημείο → επιστρέφουμε απλούς αριθμούς, όχι πίνακες
        return distance[0], index[0]
    return distance, index


if __name__ == "__main__":
    print("σημεία στον χάρτη:", len(LANES))

    drive = os.path.join(os.path.dirname(os.path.abspath(__file__)), "drives", "drive_2026-10-30_20-45-55.csv")
    data = np.genfromtxt(drive, delimiter=",", names=True)

    # 1. Η πρώτη γραμμή της καταγραφής
    distance, index = nearest_lane_point(data["gt_x"][0], data["gt_y"][0])
    p = LANES[index]
    print(f"\nπρώτη θέση του αυτοκινήτου: ({data['gt_x'][0]:.2f}, {data['gt_y'][0]:.2f})")
    print(f"  πιο κοντινό σημείο του χάρτη: #{index} στο ({p['x']:.2f}, {p['y']:.2f}), απόσταση {distance:.2f} m")
    print(f"  δρόμος {p['road_id']}, λωρίδα {p['lane_id']}, διασταύρωση: {p['is_junction']}, "
          f"γραμμές: αριστερά {p['left_marking']}, δεξιά {p['right_marking']}")

    # 2. Όλη η διαδρομή: πόσο απέχει κάθε αληθινή θέση από το πιο κοντινό κέντρο λωρίδας
    distances, _ = nearest_lane_point(data["gt_x"], data["gt_y"])
    print(f"\nόλη η διαδρομή ({len(data)} θέσεις):")
    print(f"  απόσταση από το πιο κοντινό κέντρο λωρίδας: μέση {distances.mean():.2f} m, "
          f"μέγιστη {distances.max():.2f} m")
    print(f"  πάνω από 1,75 m (μισή λωρίδα): {100 * np.mean(distances > 1.75):.2f}% του χρόνου")
