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


def expected_line_distances(x, y, yaw):
    """Βήμα 2 (9 Νοε 2026): πόσο απέχει ένα σημείο από την αριστερή και τη δεξιά γραμμή της λωρίδας του.

    x, y: θέση (m)· yaw: κατεύθυνση (rad), όπως στο particle_filter.py. Ένα σημείο ή πίνακες (π.χ. όλα τα particles).
    Επιστρέφει (αριστερά m, δεξιά m, αριστερή βαμμένη, δεξιά βαμμένη): «αριστερά/δεξιά» όπως τα βλέπει το
    αυτοκίνητο (η κάμερα), όχι όπως τα ορίζει ο χάρτης.

    Πώς: στο πιο κοντινό σημείο του χάρτη ξέρουμε το κέντρο της λωρίδας και πού είναι οι δύο γραμμές της.
    Μετράμε πόσο έχει φύγει το σημείο από το κέντρο προς κάθε γραμμή (κάθετα στον δρόμο) και το αφαιρούμε
    από την απόσταση κέντρου–γραμμής (1,75 m με πλάτος 3,5 m).
    Απλοποίηση: η απόσταση μετριέται κάθετα στον ΔΡΟΜΟ· η κάμερα τη μετράει κάθετα στο ΑΥΤΟΚΙΝΗΤΟ.
    Όταν το αυτοκίνητο είναι λίγο στραβά (π.χ. 10°), διαφέρουν ελάχιστα (~3 cm).
    """
    scalar = np.ndim(x) == 0
    x, y, yaw = np.atleast_1d(x), np.atleast_1d(y), np.atleast_1d(yaw)
    _, index = nearest_lane_point(x, y)
    p = LANES[index]

    # Από το κέντρο προς κάθε γραμμή: διάνυσμα από τον χάρτη (έτσι δεν χρειάζεται να ξέρουμε τους άξονες του CARLA).
    to_left_x, to_left_y = p["left_x"] - p["x"], p["left_y"] - p["y"]
    to_right_x, to_right_y = p["right_x"] - p["x"], p["right_y"] - p["y"]
    half_left = np.hypot(to_left_x, to_left_y)          # απόσταση κέντρου–αριστερής γραμμής (~1,75 m)
    half_right = np.hypot(to_right_x, to_right_y)       # απόσταση κέντρου–δεξιάς γραμμής (~1,75 m)

    # Πόσο έχει φύγει το σημείο από το κέντρο, προς κάθε γραμμή (προβολή πάνω στο διάνυσμα κέντρο → γραμμή).
    off_x, off_y = x - p["x"], y - p["y"]
    toward_left = (off_x * to_left_x + off_y * to_left_y) / half_left
    toward_right = (off_x * to_right_x + off_y * to_right_y) / half_right
    left = half_left - toward_left
    right = half_right - toward_right
    left_painted = p["left_marking"] != "NONE"
    right_painted = p["right_marking"] != "NONE"

    # Αν το σημείο κοιτάει ανάποδα από τη φορά της λωρίδας (π.χ. particle στο αντίθετο ρεύμα),
    # η «αριστερή» του είναι η «δεξιά» της λωρίδας: αλλάζουμε τα δύο μεταξύ τους.
    lane_yaw = np.radians(p["yaw"])
    angle_diff = np.arctan2(np.sin(yaw - lane_yaw), np.cos(yaw - lane_yaw))   # διαφορά γωνίας στο [-π, π]
    flip = np.abs(angle_diff) > np.pi / 2
    left, right = np.where(flip, right, left), np.where(flip, left, right)
    left_painted, right_painted = np.where(flip, right_painted, left_painted), np.where(flip, left_painted, right_painted)

    if scalar:
        return float(left[0]), float(right[0]), bool(left_painted[0]), bool(right_painted[0])
    return left, right, left_painted, right_painted


# ─── Είδος γραμμής και γωνία λωρίδας (10–11 Νοε 2026) ───────────────────────────────────────────────────
# Η απόσταση από τις γραμμές είναι ίδια σε κάθε λωρίδα (1,75 m αριστερά και δεξιά), άρα δεν λέει ΠΟΙΑ λωρίδα είναι.
# Το είδος της γραμμής το λέει: στους δρόμους με 4 λωρίδες του Town10 η εσωτερική λωρίδα έχει (διπλή, διακεκομμένη)
# και η εξωτερική (διακεκομμένη, συνεχής). Κωδικοί: 0 = άβαφη, 1 = διακεκομμένη, 2 = συνεχής, 3 = διπλή.
LINE_BROKEN, LINE_SOLID, LINE_DOUBLE = 1, 2, 3
_TYPE_CODE = {"NONE": 0, "Broken": LINE_BROKEN, "Solid": LINE_SOLID, "SolidSolid": LINE_DOUBLE}
LEFT_TYPE = np.array([_TYPE_CODE.get(t, 0) for t in LANES["left_marking"]])
RIGHT_TYPE = np.array([_TYPE_CODE.get(t, 0) for t in LANES["right_marking"]])


def expected_line_types_and_angle(x, y, yaw):
    """Για ένα σημείο ή πίνακες (π.χ. όλα τα particles), από τον χάρτη:
    (είδος αριστερής γραμμής, είδος δεξιάς, γωνία της λωρίδας ως προς την κατεύθυνση του σημείου).
    Είδος: 0 άβαφη, 1 διακεκομμένη, 2 συνεχής, 3 διπλή. Γωνία σε rad, θετική = η λωρίδα πάει προς τα δεξιά του σημείου
    (όπως τη μετράει ο ανιχνευτής). «Αριστερά/δεξιά» όπως τα βλέπει το αυτοκίνητο: με ανάποδη φορά αλλάζουν θέση,
    όπως στο expected_line_distances. Επιστρέφει πάντα πίνακες."""
    x, y, yaw = np.atleast_1d(x), np.atleast_1d(y), np.atleast_1d(yaw)
    _, index = nearest_lane_point(x, y)
    lane_yaw = np.radians(LANES["yaw"][index])
    diff = np.arctan2(np.sin(lane_yaw - yaw), np.cos(lane_yaw - yaw))       # κατεύθυνση λωρίδας − κατεύθυνση σημείου
    flip = np.abs(diff) > np.pi / 2                                        # το σημείο κοιτάει ανάποδα από τη λωρίδα
    left_type = np.where(flip, RIGHT_TYPE[index], LEFT_TYPE[index])
    right_type = np.where(flip, LEFT_TYPE[index], RIGHT_TYPE[index])
    angle = np.where(flip, np.arctan2(np.sin(diff + np.pi), np.cos(diff + np.pi)), diff)
    return left_type, right_type, angle


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

    # 3. Βήμα 2: αποστάσεις από τις γραμμές, στις αληθινές θέσεις και κατευθύνσεις
    yaw = np.radians(data["gt_yaw"])
    left, right, left_painted, right_painted = expected_line_distances(data["gt_x"], data["gt_y"], yaw)
    print("\nβήμα 2, στις αληθινές θέσεις:")
    print(f"  αριστερά + δεξιά: μέσο {np.mean(left + right):.3f} m, από {np.min(left + right):.3f} "
          f"έως {np.max(left + right):.3f} m (πρέπει 3,5)")
    print(f"  απόσταση από την αριστερή γραμμή: μέση {left.mean():.2f} m, από {left.min():.2f} έως {left.max():.2f} m")
    print(f"  απόσταση από τη δεξιά γραμμή:     μέση {right.mean():.2f} m, από {right.min():.2f} έως {right.max():.2f} m")
    print(f"  βαμμένη γραμμή: αριστερά {100 * left_painted.mean():.0f}%, δεξιά {100 * right_painted.mean():.0f}%, "
          f"και οι δύο {100 * np.mean(left_painted & right_painted):.0f}%, καμία {100 * np.mean(~left_painted & ~right_painted):.0f}% του χρόνου")

    # 4. Η παγίδα του αντίθετου ρεύματος: ίδιες θέσεις, ανάποδη κατεύθυνση → αριστερά και δεξιά αλλάζουν θέση
    left_r, right_r, _, _ = expected_line_distances(data["gt_x"], data["gt_y"], yaw + np.pi)
    print(f"\nανάποδη κατεύθυνση: αριστερά == δεξιά της κανονικής σε {100 * np.mean(np.isclose(left_r, right)):.0f}% των θέσεων")

    # 5. Ένα σημείο μόνο (όπως θα το καλεί η «ψεύτικη» κάμερα στην αληθινή θέση)
    l0, r0, lp0, rp0 = expected_line_distances(data["gt_x"][0], data["gt_y"][0], yaw[0])
    print(f"\nπρώτη θέση: αριστερά {l0:.2f} m (βαμμένη: {lp0}), δεξιά {r0:.2f} m (βαμμένη: {rp0})")
