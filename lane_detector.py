"""
Ανιχνευτής λωρίδων από τη semantic κάμερα («μέση οδός», 7–9 Νοε 2026). Τρέχει στο PC του εργαστηρίου,
όπου είναι οι εικόνες. Χρειάζεται μόνο numpy και cv2 (και carla μόνο για το --from-carla).

Τι κάνει, για κάθε εικόνα της ΜΠΡΟΣΤΙΝΗΣ semantic κάμερας (10 τη στιγμή):
  1. Κρατάει τα pixels με ετικέτα 24 (RoadLine: «οι βαφές του δρόμου»). Σε κάθε σειρά της εικόνας, κάθε βαφή
     γίνεται ΕΝΑ σημείο στο κέντρο της· δύο λεπτές κολλητές βαφές (διπλή γραμμή) γίνονται ένα σημείο στη ΜΕΣΗ τους·
     οι φαρδιές βαφές (διάβαση, στοπ, βέλη) πετιούνται (10 Νοε 2026, βλ. line_points).
  2. Τα μεταφέρει στο έδαφος: κάμερα με γνωστό ύψος, κλίση 0°, οπτικό πεδίο 90°, δρόμος επίπεδος.
     Κάθε σημείο → «Χ m μπροστά, Υ m δεξιά» από το αυτοκίνητο.
  3. Κρατάει τα σημεία 3–15 m μπροστά από την κάμερα, αριστερά και δεξιά του αυτοκινήτου (0,5–3,5 m στο πλάι).
  4. Σε κάθε πλευρά ταιριάζει μια καμπύλη Υ = c0 + c1·Χ + c2·Χ² (RANSAC: αγνοεί τα σημεία που δεν ταιριάζουν,
     π.χ. βέλη· καμπύλη και όχι ευθεία, για τις στροφές).
     c0 = πλάγια απόσταση της γραμμής στο ΣΗΜΕΙΟ ΑΝΑΦΟΡΑΣ του αυτοκινήτου (εκεί που είναι τα gt_x, gt_y)·
     c1 = κλίση στο σημείο αναφοράς → γωνία της λωρίδας ως προς το αυτοκίνητο.
  5. Βρίσκει ΟΛΕΣ τις παράλληλες γραμμές της πλευράς: αν είναι πάνω από μία, δεν ξέρουμε ποια είναι η δική μας → «δεν είδα».
  6. «Δεν είδα» αν: λίγα σημεία, πολύ μικρό μήκος, παράλογη γωνία, ή (με δύο γραμμές) πλάτος λωρίδας εκτός 3–4 m.
     Η γωνία σημειώνεται «ok» μόνο αν η γραμμή απλώνεται τουλάχιστον 8 m.
Βγάζει ΤΕΛΕΙΕΣ μετρήσεις (χωρίς επιπλέον λάθος): το λάθος 15 cm και οι απώλειες μπαίνουν στο Mac (απόφαση 9 Νοε).

Αποτέλεσμα: drives/<καταγραφή>_lanes.csv, μία γραμμή ανά εικόνα:
  frame, left_dist, left_angle_deg, left_points, left_ok, right_dist, right_angle_deg, right_points, right_ok,
  left_curv, right_curv (c2 της καμπύλης, 1/m: μόνο για τις εικόνες ελέγχου· προστέθηκε 9 Νοε 2026),
  left_kind, right_kind (0 αμφίσημο/δύο γραμμές, 1 απλή γραμμή, 2 διπλή → η μέση της, 3 πολλές ρίγες), left_angle_ok, right_angle_ok
  left_type, right_type (11 Νοε 2026: 0 δεν ξέρω, 1 διακεκομμένη, 2 συνεχής, 3 διπλή), left_run, right_run (το μεγαλύτερο
  συνεχόμενο βαμμένο κομμάτι της γραμμής, m)
  (left_points, right_points: από 10 Νοε 2026 είναι σημεία ανά σειρά εικόνας, όχι pixels)
  (dist σε m, πάντα θετική· angle σε μοίρες, θετική = η λωρίδα στρίβει προς τα δεξιά του αυτοκινήτου)

Χρήση (από τον φάκελο του repo, στο εργαστήριο):
    ~/carla-env/bin/python lane_detector.py --from-carla               (μία φορά: γεωμετρία κάμερας από το CARLA)
    ~/carla-env/bin/python lane_detector.py drive_2026-10-30_20-45-55 --debug 30
    ~/carla-env/bin/python lane_detector.py drive_2026-10-30_21-24-29 --debug 30
--debug N: σώζει και N εικόνες ελέγχου (οι γραμμές που βρήκε, ζωγραφισμένες πάνω στην εικόνα).
"""

import argparse
import csv
import glob
import json
import os
from concurrent.futures import ProcessPoolExecutor

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
DRIVES_DIR = os.path.join(HERE, "drives")
GEOMETRY_FILE = os.path.join(DRIVES_DIR, "camera_front_geometry.json")

ROAD_LINE = 24                                   # ετικέτα RoadLine (sources/CARLA_ref_sensors_ue4-dev.md:854)
WIDTH, HEIGHT, FOV_DEG = 1280, 960, 90           # όπως στο BMW_spawn.py
FOCAL = WIDTH / (2 * np.tan(np.radians(FOV_DEG / 2)))   # = 640 pixels
CX, CY = WIDTH / 2, HEIGHT / 2                   # κέντρο της εικόνας (εκεί είναι ο ορίζοντας, με κλίση 0°)

NEAR, FAR = 3.0, 15.0           # m μπροστά από την κάμερα: εδώ ψάχνουμε γραμμές (ο δρόμος φαίνεται από ~2,7 m)
SIDE_MIN, SIDE_MAX = 0.5, 3.5   # m στο πλάι: εδώ περιμένουμε τις γραμμές της λωρίδας μας (κέντρο → γραμμή: 1,75 m)
MIN_POINTS = 30                 # ελάχιστα σημεία γραμμής (ένα σημείο ανά σειρά της εικόνας) για να πιστέψουμε ότι είδαμε γραμμή
MIN_LENGTH = 4.0                # m: η γραμμή πρέπει να απλώνεται τουλάχιστον τόσο μπροστά-πίσω
MAX_ANGLE_DEG = 30              # γραμμή πιο στραβή από αυτό δεν είναι γραμμή της λωρίδας μας (π.χ. γραμμή στοπ)
MIN_ANGLE_LENGTH = 8.0          # m: για να εμπιστευτούμε τη ΓΩΝΙΑ, η γραμμή πρέπει να απλώνεται τόσο (9 Νοε 2026)
RANSAC_TOL = 0.08               # m: ένα σημείο «ταιριάζει» στην καμπύλη αν απέχει λιγότερο από αυτό
MAX_LINE_WIDTH = 0.35           # m: βαφή πιο φαρδιά από αυτό δεν είναι γραμμή λωρίδας (ρίγα διάβασης, γραμμή στοπ, βέλος)
PAINT_GAP = 1.0                 # m: κενό στη βαφή μεγαλύτερο από αυτό, κατά μήκος της γραμμής, χωρίζει δύο «κομμάτια»
SOLID_MIN_RUN = 4.0             # m: συνεχόμενη βαφή τουλάχιστον τόσο = ΣΥΝΕΧΗΣ γραμμή (οι παύλες του Town10 είναι ~2 m)
DOUBLE_MAX_GAP = 0.25           # m: δύο λεπτές βαφές με κενό έως τόσο = ΔΙΠΛΗ γραμμή (στο Town10 το κενό είναι ~0,1 m)
LANE_WIDTH_RANGE = (3.0, 4.0)   # m: με δύο γραμμές, το πλάτος πρέπει να είναι λογικό (στο Town10 είναι 3,5)


# ─── Γεωμετρία κάμερας ──────────────────────────────────────────────────────────────────────────────────
def geometry_from_carla():
    """Φτιάχνει το αυτοκίνητο του BMW_spawn.py για λίγο, διαβάζει το bounding box, και υπολογίζει πού είναι η
    μπροστινή κάμερα με τον ΙΔΙΟ τύπο με το BMW_spawn.py. Χρειάζεται ανοιχτό CARLA."""
    import carla
    client = carla.Client("localhost", 2000)
    client.set_timeout(20.0)
    world = client.get_world()
    bp = world.get_blueprint_library().find("vehicle.bmw.grandtourer")
    vehicle = None
    for sp in world.get_map().get_spawn_points():
        vehicle = world.try_spawn_actor(bp, sp)
        if vehicle is not None:
            break
    if vehicle is None:
        raise RuntimeError("Δεν μπόρεσα να φτιάξω το αυτοκίνητο σε κανένα spawn point.")
    box = vehicle.bounding_box
    vehicle.destroy()
    # Ύψος της κάμερας ΠΑΝΩ ΑΠΟ ΤΟΝ ΔΡΟΜΟ: roof_z του BMW_spawn.py (box.location.z + box.extent.z + 0.05, μετρημένο από
    # το σημείο αναφοράς του αυτοκινήτου) συν την απόσταση σημείου αναφοράς–δρόμου (box.extent.z − box.location.z: το
    # κάτω μέρος του αυτοκινήτου είναι ~7 cm κάτω από το σημείο αναφοράς). Άρα 2·extent.z + 0.05 = 1,717 m.
    # Διορθώθηκε 9 Νοε 2026: με το ύψος από το σημείο αναφοράς (1,649 m) το πλάτος λωρίδας έβγαινε 3,375 αντί 3,5 m.
    geometry = {
        "height": 2 * box.extent.z + 0.05,
        "forward": box.location.x + 0.25 * box.extent.x,     # POS["front"] του BMW_spawn.py
        "box": [box.location.x, box.location.y, box.location.z, box.extent.x, box.extent.y, box.extent.z],
    }
    os.makedirs(DRIVES_DIR, exist_ok=True)
    with open(GEOMETRY_FILE, "w") as f:
        json.dump(geometry, f, indent=1)
    print("Γεωμετρία κάμερας:", geometry, "→", GEOMETRY_FILE)
    return geometry


# ─── Από pixels σε σημεία στο έδαφος ────────────────────────────────────────────────────────────────────
def pixels_to_ground(u, v, height, forward):
    """pixel (u, v) → σημείο στο έδαφος σε συντεταγμένες αυτοκινήτου: Χ μπροστά από το σημείο αναφοράς,
    Υ προς τα ΔΕΞΙΑ (όπως ο άξονας y του CARLA). Κάμερα με κλίση 0°, επίπεδος δρόμος."""
    below = v - CY                                   # pixels κάτω από τον ορίζοντα (> 0 για το έδαφος)
    x_cam = FOCAL * height / below                   # m μπροστά από την κάμερα
    y = (u - CX) * x_cam / FOCAL                     # m δεξιά
    return x_cam + forward, y, x_cam


def ground_to_pixels(x, y, height, forward):
    """Το αντίστροφο (για τις εικόνες ελέγχου): σημείο στο έδαφος → pixel."""
    x_cam = x - forward
    return CX + FOCAL * y / x_cam, CY + FOCAL * height / x_cam


# ─── Από την εικόνα σε σημεία γραμμών ───────────────────────────────────────────────────────────────────
def line_points(tags, height, forward):
    """Ένα σημείο ανά σειρά της εικόνας για κάθε βαφή (διόρθωση 10 Νοε 2026).
    Σε κάθε σειρά (= μία απόσταση μπροστά) οι βαφές είναι οριζόντια «κομμάτια» από pixels με ετικέτα 24:
      - κομμάτι πιο φαρδύ από MAX_LINE_WIDTH → δεν είναι γραμμή λωρίδας (διάβαση, στοπ, βέλος) → πετιέται
      - δύο λεπτά κομμάτια με μικρό κενό (≤ DOUBLE_MAX_GAP) → ΔΙΠΛΗ γραμμή → ΕΝΑ σημείο στη μέση τους,
        γιατί εκεί βάζει ο χάρτης το όριο της λωρίδας
      - τρία ή περισσότερα κολλητά → κάτι άλλο (π.χ. διαγράμμιση) → πετιούνται
      - μόνο του → ένα σημείο στο κέντρο του
    Γιατί έτσι: η διπλή κίτρινη του Town10 είναι δύο γραμμές σε απόσταση ~0,28 m. Με όλα τα pixels μαζί, η καμπύλη
    «έπιανε» και τις δύο λοξά (από τη μία στην άλλη) και, προεκτεινόμενη ως το αυτοκίνητο, έκανε λάθος ±0,5 m και
    ψεύτικη γωνία ±10° (μετρημένο 10 Νοε 2026, εικόνες 17744–17900 της καταγραφής 30 Οκτ 20:45).
    Επιστρέφει (Χ μπροστά, Υ δεξιά, είναι_διπλή) για τα σημεία NEAR–FAR m μπροστά από την κάμερα."""
    v0 = int(np.ceil(CY + FOCAL * height / FAR))
    v1 = min(int(np.floor(CY + FOCAL * height / NEAR)), HEIGHT - 1)
    mask = np.zeros((v1 - v0 + 1, WIDTH + 2), np.int8)
    mask[:, 1:-1] = tags[v0:v1 + 1] == ROAD_LINE
    change = np.diff(mask, axis=1)
    row, start = np.nonzero(change == 1)             # αρχή κάθε κομματιού (στήλη του πρώτου pixel)
    _, end = np.nonzero(change == -1)                # τέλος (στήλη μετά το τελευταίο pixel) — ίδια σειρά με τις αρχές
    if len(row) == 0:
        return np.empty(0), np.empty(0), np.empty(0, bool)
    x_cam = FOCAL * height / (row + v0 - CY)         # m μπροστά από την κάμερα, για κάθε κομμάτι
    metres = x_cam / FOCAL                           # πόσα m είναι ένα pixel σε αυτή τη σειρά
    narrow = (end - start) * metres <= MAX_LINE_WIDTH
    # κολλητό με το επόμενο κομμάτι της ίδιας σειράς; (και τα δύο λεπτά, μικρό κενό)
    close = np.zeros(len(row), bool)
    close[:-1] = ((row[1:] == row[:-1]) & narrow[1:] & narrow[:-1]
                  & ((start[1:] - end[:-1]) * metres[:-1] <= DOUBLE_MAX_GAP))
    close_prev = np.zeros(len(row), bool)
    close_prev[1:] = close[:-1]                      # κολλητό με το προηγούμενο
    single = narrow & ~close & ~close_prev
    first_of_pair = close & ~close_prev              # ζευγάρι ακριβώς δύο κομματιών: το πρώτο του...
    first_of_pair[:-1] &= ~close[1:]                 # ...και το δεύτερο δεν είναι κολλητό με τρίτο
    i = np.nonzero(first_of_pair)[0]
    centre = np.concatenate([(start[single] + end[single] - 1) / 2, (start[i] + end[i + 1] - 1) / 2])
    x_cam = np.concatenate([x_cam[single], x_cam[i]])
    is_double = np.concatenate([np.zeros(single.sum(), bool), np.ones(len(i), bool)])
    return x_cam + forward, (centre - CX) * x_cam / FOCAL, is_double


# ─── Ευθεία με RANSAC ───────────────────────────────────────────────────────────────────────────────────
def fit_line(x, y, rng):
    """Καμπύλη y = c0 + c1·x + c2·x² που ταιριάζει στα περισσότερα σημεία (RANSAC με 3 σημεία).
    Δεύτερου βαθμού και όχι ευθεία: στις στροφές η γραμμή καμπυλώνει, και μια ευθεία «προεκτεινόμενη» πίσω ως το
    σημείο αναφοράς του αυτοκινήτου (3–4 m πίσω από τα πρώτα ορατά σημεία) έκανε λάθος ~45 cm (δοκιμή 9 Νοε).
    Επιστρέφει (c0, c1, πόσα ταιριάζουν, μήκος) ή None."""
    if len(x) < MIN_POINTS:
        return None
    if len(x) > 3000:                                # αρκετά για την καμπύλη, και πιο γρήγορα
        pick = rng.choice(len(x), 3000, replace=False)
        x, y = x[pick], y[pick]
    best = None
    for _ in range(150):
        i = rng.choice(len(x), 3, replace=False)
        if np.ptp(x[i]) < 2.0:                       # τρία σημεία πολύ κοντά μπροστά-πίσω: κακή καμπύλη
            continue
        try:                                         # καμπύλη ακριβώς από τα 3 σημεία: c0 + c1·x + c2·x²
            c0, c1, c2 = np.linalg.solve(np.stack([np.ones(3), x[i], x[i] ** 2], axis=1), y[i])
        except np.linalg.LinAlgError:
            continue
        inliers = np.abs(y - (c0 + c1 * x + c2 * x ** 2)) < RANSAC_TOL
        if best is None or inliers.sum() > best.sum():
            best = inliers
    if best is None or best.sum() < MIN_POINTS:
        return None
    c2, c1, c0 = np.polyfit(x[best], y[best], 2)    # τελική καμπύλη: μόνο με τα σημεία που ταιριάζουν
    length = x[best].max() - x[best].min()
    return c0, c1, int(best.sum()), length, c2


def paint_runs(x):
    """Τα βαμμένα «κομμάτια» κατά μήκος μιας γραμμής (11 Νοε 2026). x: πόσο μπροστά είναι κάθε σημείο της (m).
    Όπου δύο διαδοχικά σημεία απέχουν πάνω από PAINT_GAP, εκεί η βαφή έχει κενό. (Οι σειρές της εικόνας απέχουν το
    πολύ ~0,2 m στα 15 m, άρα ένα κενό 1 m είναι πραγματικό κενό.)
    Επιστρέφει (μήκος του μεγαλύτερου κομματιού σε m, πόσα κομμάτια)."""
    if len(x) == 0:
        return 0.0, 0
    x = np.sort(x)
    cuts = np.nonzero(np.diff(x) > PAINT_GAP)[0]
    starts = np.concatenate(([0], cuts + 1))
    ends = np.concatenate((cuts, [len(x) - 1]))
    return float(np.max(x[ends] - x[starts])), len(starts)


def find_lines(x, y, is_double, rng, max_lines=4):
    """Όλες οι ξεχωριστές γραμμές στα σημεία μιας πλευράς (μέχρι max_lines): βρίσκει την πιο «γεμάτη» με RANSAC,
    βγάζει τα σημεία της, ξαναψάχνει.
    Επιστρέφει λίστα από (c0, c1, πόσα σημεία, μήκος, c2, ποσοστό των σημείων της που είναι από διπλή γραμμή,
    μεγαλύτερο συνεχόμενο βαμμένο κομμάτι σε m, πόσα κομμάτια)."""
    lines = []
    remaining = np.ones(len(x), bool)
    for _ in range(max_lines):
        if remaining.sum() < MIN_POINTS:
            break
        fit = fit_line(x[remaining], y[remaining], rng)
        if fit is None:
            break
        c0, c1, _, _, c2 = fit
        distance = np.abs(y - (c0 + c1 * x + c2 * x ** 2))
        on_line = remaining & (distance < RANSAC_TOL)
        longest_run, runs = paint_runs(x[on_line])
        lines.append(fit + (float(is_double[on_line].mean()) if on_line.any() else 0.0, longest_run, runs))
        remaining &= distance >= 2 * RANSAC_TOL      # βγάζουμε τα σημεία της
    return lines


def choose_line(lines):
    """Από τις γραμμές μιας πλευράς, ποια είναι η γραμμή της λωρίδας μας.
    Κρατάμε μόνο «σωστές» γραμμές (αρκετά μακριές, όχι πολύ στραβές) που είναι ΠΑΡΑΛΛΗΛΕΣ με την πιο γεμάτη. Μετά:
      1 γραμμή           → αυτή. Είδος 1 = απλή, είδος 2 = διπλή (τα περισσότερα σημεία της είναι «μέση διπλής»,
                           βλ. line_points: η διπλή γραμμή έχει ήδη γίνει ΜΙΑ γραμμή στη μέση της)
      2 γραμμές          → δεν ξέρουμε ποια είναι η δική μας → «δεν είδα»                          (είδος 0)
      3 ή περισσότερες   → πολλές παράλληλες ρίγες → «δεν είδα»                                    (είδος 3)
    Τύπος της γραμμής (11 Νοε 2026), από το πώς είναι βαμμένη κατά μήκος της:
      3 = διπλή·  2 = συνεχής (έχει συνεχόμενη βαφή ≥ SOLID_MIN_RUN: μια παύλα δεν είναι ποτέ τόσο μακριά, και μια
      συνεχής που την κρύβει λίγο ένα αυτοκίνητο έχει ακόμα ένα μακρύ κομμάτι)·  1 = διακεκομμένη (δύο ή περισσότερα
      κοντά κομμάτια)·  0 = δεν ξέρω.
    Επιστρέφει (c0, c1, πόσα σημεία, μήκος, c2, είδος, τύπος, μεγαλύτερο συνεχόμενο κομμάτι σε m) ή None."""
    good = [f for f in lines if f[3] >= MIN_LENGTH and abs(np.degrees(np.arctan(f[1]))) <= MAX_ANGLE_DEG]
    if not good:
        return None
    main = max(good, key=lambda f: f[2])
    parallel = [f for f in good if abs(f[1] - main[1]) < 0.05 and abs(f[4] - main[4]) < 0.02]
    if len(parallel) == 1:
        if main[5] >= 0.5:
            return main[:5] + (2, 3, main[6])
        line_type = 2 if main[6] >= SOLID_MIN_RUN else (1 if main[7] >= 2 else 0)
        return main[:5] + (1, line_type, main[6])
    return main[:5] + (3 if len(parallel) >= 3 else 0, 0, main[6])


def detect(tags, height, forward, rng):
    """Μία εικόνα (πίνακας με ετικέτες) → dict με τις μετρήσεις αριστερά και δεξιά:
    (απόσταση, γωνία, σημεία, ok, c2, είδος γραμμής, γωνία ok, τύπος γραμμής, μεγαλύτερο συνεχόμενο κομμάτι)."""
    x, y, is_double = line_points(tags, height, forward)   # ένα σημείο ανά σειρά και βαφή· διπλή γραμμή → η μέση της

    result = {}
    for side, sign in [("left", -1), ("right", +1)]:     # αριστερά: y < 0, δεξιά: y > 0
        band = (sign * y >= SIDE_MIN) & (sign * y <= SIDE_MAX)
        line = choose_line(find_lines(x[band], y[band], is_double[band], rng))
        ok = angle_ok = False
        dist = angle = np.nan
        points = kind = line_type = 0
        curv = run = 0.0
        if line is not None:
            c0, c1, points, length, curv, kind, line_type, run = line
            dist = sign * c0                             # απόσταση στο σημείο αναφοράς (θετική)
            angle = np.degrees(np.arctan(c1))
            ok = kind in (1, 2) and SIDE_MIN <= dist <= SIDE_MAX
            # Η γωνία θέλει μακρύτερη γραμμή από την απόσταση: με λίγα μέτρα, μικρά λάθη στα σημεία τη «γέρνουν»
            angle_ok = ok and length >= MIN_ANGLE_LENGTH
        result[side] = (dist, angle, points, ok, curv, kind, angle_ok, line_type, run)

    # Με δύο γραμμές: το πλάτος της λωρίδας πρέπει να είναι λογικό, αλλιώς μάλλον μία από τις δύο είναι λάθος
    # (π.χ. ρίγα διάβασης πεζών). Δεν ξέρουμε ποια, οπότε δεν κρατάμε καμία.
    if result["left"][3] and result["right"][3]:
        width = result["left"][0] + result["right"][0]
        if not (LANE_WIDTH_RANGE[0] <= width <= LANE_WIDTH_RANGE[1]):
            for s in ("left", "right"):
                r = result[s]
                result[s] = r[:3] + (False,) + r[4:6] + (False,) + r[7:]
    return result


def process(job):
    path, height, forward = job
    tags = cv2.imread(path, cv2.IMREAD_UNCHANGED)
    rng = np.random.default_rng(int(os.path.basename(path).split(".")[0]))   # ίδιο αποτέλεσμα σε κάθε τρέξιμο
    r = detect(tags, height, forward, rng)
    frame = int(os.path.basename(path).split(".")[0])
    return [frame] + [r["left"][0], r["left"][1], r["left"][2], int(r["left"][3]),
                      r["right"][0], r["right"][1], r["right"][2], int(r["right"][3]),
                      r["left"][4], r["right"][4],
                      r["left"][5], r["right"][5], int(r["left"][6]), int(r["right"][6]),
                      r["left"][7], r["right"][7], r["left"][8], r["right"][8]]


def debug_image(path, row, height, forward, out_path):
    """Εικόνα ελέγχου: γκρι = δρόμος κ.λπ., άσπρο = βαφές (ετικέτα 24), κόκκινη/μπλε = η αριστερή/δεξιά γραμμή που
    βρήκε (ζωγραφισμένη μόνο αν ok). Η κίτρινη ζώνη δείχνει πού ψάχνει (3–15 m μπροστά)."""
    tags = cv2.imread(path, cv2.IMREAD_UNCHANGED)
    img = np.full((HEIGHT, WIDTH, 3), 60, np.uint8)
    img[tags == ROAD_LINE] = (255, 255, 255)
    for x_cam in (NEAR, FAR):
        v = int(CY + FOCAL * height / x_cam)
        cv2.line(img, (0, v), (WIDTH - 1, v), (0, 200, 255), 1)
    _, l_dist, l_ang, _, l_ok, r_dist, r_ang, _, r_ok, l_curv, r_curv = row[:11]
    for dist, ang, ok, curv, sign, color in [(l_dist, l_ang, l_ok, l_curv, -1, (0, 0, 255)),
                                             (r_dist, r_ang, r_ok, r_curv, +1, (255, 0, 0))]:
        if not ok:
            continue
        xs = np.linspace(forward + NEAR, forward + FAR, 30)
        ys = sign * dist + np.tan(np.radians(ang)) * xs + curv * xs ** 2   # η καμπύλη όπως την ταίριαξε (και στις στροφές)
        u, v = ground_to_pixels(xs, ys, height, forward)
        pts = np.stack([u, v], axis=1).astype(np.int32)
        cv2.polylines(img, [pts], False, color, 3)
    text = f"L {l_dist:.2f} m ok={l_ok}   R {r_dist:.2f} m ok={r_ok}"
    cv2.putText(img, text, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)
    cv2.imwrite(out_path, img)


def find_images(drive):
    """Ο φάκελος front_semantic της καταγραφής: δίπλα στο CSV (drives/) ή, για παλιές καταγραφές, στον κεντρικό φάκελο."""
    for base in (DRIVES_DIR, HERE):
        folder = os.path.join(base, drive, "front_semantic")
        if os.path.isdir(folder):
            return folder
    raise FileNotFoundError(f"Δεν βρήκα το {drive}/front_semantic ούτε στο drives/ ούτε στον κεντρικό φάκελο.")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("drive", nargs="?", help="π.χ. drive_2026-10-30_20-45-55 (χωρίς .csv)")
    parser.add_argument("--from-carla", action="store_true", help="γεωμετρία κάμερας από το CARLA (μία φορά)")
    parser.add_argument("--debug", type=int, default=0, help="πόσες εικόνες ελέγχου να σώσει")
    args = parser.parse_args()

    if args.from_carla:
        geometry = geometry_from_carla()
    elif os.path.exists(GEOMETRY_FILE):
        geometry = json.load(open(GEOMETRY_FILE))
    else:
        raise SystemExit("Πρώτα: lane_detector.py --from-carla (με ανοιχτό το CARLA), για τη γεωμετρία της κάμερας.")
    if args.drive is None:
        return
    drive = args.drive[:-4] if args.drive.endswith(".csv") else args.drive
    height, forward = geometry["height"], geometry["forward"]

    folder = find_images(drive)
    paths = sorted(glob.glob(os.path.join(folder, "*.png")), key=lambda p: int(os.path.basename(p).split(".")[0]))
    print(f"{drive}: {len(paths)} εικόνες στο {folder}, κάμερα σε ύψος {height:.2f} m, {forward:.2f} m μπροστά")

    with ProcessPoolExecutor() as pool:
        rows = list(pool.map(process, [(p, height, forward) for p in paths], chunksize=50))

    out = os.path.join(DRIVES_DIR, f"{drive}_lanes.csv")
    with open(out, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["frame", "left_dist", "left_angle_deg", "left_points", "left_ok",
                    "right_dist", "right_angle_deg", "right_points", "right_ok", "left_curv", "right_curv",
                    "left_kind", "right_kind", "left_angle_ok", "right_angle_ok",
                    "left_type", "right_type", "left_run", "right_run"])
        for r in rows:
            w.writerow([r[0]] + [f"{v:.4f}" if isinstance(v, float) else v for v in r[1:]])
    rows_arr = np.array([[r[4], r[8], r[1] + r[5] if (r[4] and r[8]) else np.nan] for r in rows], dtype=float)
    print(f"→ {out}")
    print(f"είδε γραμμή: αριστερά {100 * rows_arr[:, 0].mean():.0f}%, δεξιά {100 * rows_arr[:, 1].mean():.0f}% των εικόνων")
    widths = rows_arr[:, 2][~np.isnan(rows_arr[:, 2])]
    if len(widths):
        # Έλεγχος της γεωμετρίας χωρίς αλήθεια: αριστερά + δεξιά πρέπει να βγαίνει ~3,5 m (πλάτος λωρίδας στο Town10).
        # Αν βγαίνει π.χ. 3,8, το ύψος της κάμερας είναι λάθος κατά 3,8/3,5 (οι αποστάσεις είναι ανάλογες του ύψους).
        print(f"πλάτος λωρίδας (αριστερά + δεξιά): διάμεσος {np.median(widths):.3f} m (πρέπει ~3,5)")

    if args.debug:
        debug_dir = os.path.join(DRIVES_DIR, f"{drive}_lanes_debug")
        os.makedirs(debug_dir, exist_ok=True)
        for k in np.linspace(0, len(paths) - 1, args.debug).astype(int):
            debug_image(paths[k], rows[k], height, forward, os.path.join(debug_dir, f"{rows[k][0]}.png"))
        print(f"εικόνες ελέγχου → {debug_dir}")


if __name__ == "__main__":
    main()
