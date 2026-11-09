"""
Δοκιμή της επιλογής Α για τα κτήρια (9 Νοε 2026): βρίσκουμε κάθετες άκρες κτηρίων ΜΟΝΟ από την κανονική (RGB)
κάμερα, με κλασική επεξεργασία εικόνας (OpenCV), και μετράμε πόσες είναι σωστές.

Τι κάνει, σε λίγες εικόνες (π.χ. 20) μιας καταγραφής:
  1. Ανιχνευτής (μόνο RGB): ακμές (Canny) → ευθείες (Hough) → κρατάει μόνο τις ΚΑΘΕΤΕΣ (έως 5°), ΜΑΚΡΙΕΣ
     (≥ 80 px) που φτάνουν ΠΑΝΩ από τον ορίζοντα (δηλαδή πάνω από τα αυτοκίνητα). Γραμμές στην ίδια περίπου στήλη
     (< 6 px) ενώνονται σε μία άκρη.
     Γιατί κάθετες στην εικόνα: η κάμερα κοιτάει οριζόντια (pitch 0), άρα κάθε κατακόρυφη γραμμή του κόσμου
     (γωνία κτηρίου, κολόνα) φαίνεται κατακόρυφη και στην εικόνα.
  2. Βαθμολόγηση (ΔΕΝ είναι μέρος του ανιχνευτή, μόνο για να δούμε πόσο καλός είναι):
     α) Χάρτης: οι γωνίες των κτηρίων (town10/town10_buildings_clean.json), στην ΑΛΗΘΙΝΗ θέση του αυτοκινήτου,
        σε ποια στήλη της εικόνας πρέπει να φαίνονται (μόνο όσες δεν κρύβονται πίσω από κτήριο, έως 100 m).
        Μια άκρη είναι «σωστή» αν απέχει έως 15 px (~1,3°) από μια τέτοια γωνία.
     β) Semantic κάμερα (ίδια θέση, ίδιο frame): τι είναι η κάθε άκρη:
          B = όριο κτηρίου (κτήριο από τη μία μεριά, όχι από την άλλη)
          F = μέσα στο κτήριο (γραμμή της πρόσοψης, ή γωνία που φαίνονται και οι δύο πλευρές της)
          P = κολόνα / φανάρι / πινακίδα,  T = δέντρο,  O = κάτι άλλο (αυτοκίνητο, φράχτης κ.λπ.)
  3. Τυπώνει τα νούμερα και γράφει εικόνες ελέγχου.

Προσοχή: ο χάρτης έχει χοντρά κουτιά, όχι το ακριβές σχήμα των κτηρίων. Άρα κάποιες σωστές άκρες μπορεί να
μετρηθούν ως λάθος (και το ανάποδο). Γι' αυτό υπάρχει και το (β). Με τον χάρτη του LiDAR θα γίνει πιο ακριβές.

Εκτέλεση (στο εργαστήριο, όπου είναι οι εικόνες):
    ~/carla-env/bin/python building_edge_test.py --drive drive_2026-10-30_20-45-55 --camera front --n 20
    ~/carla-env/bin/python building_edge_test.py --drive drive_2026-10-30_20-45-55 --camera all --n 20
Εικόνες ελέγχου: drives/building_edge_test/<καταγραφή>_<κάμερα>/<frame>.jpg
  πράσινη = άκρη που ταιριάζει με γωνία του χάρτη, κόκκινη = δεν ταιριάζει, κίτρινη λεπτή = γωνία του χάρτη,
  μπλε οριζόντια = ορίζοντας. Πάνω από κάθε άκρη το γράμμα της semantic (B/F/P/T/O).
"""

import argparse
import glob
import json
import os
from collections import Counter

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
DRIVES_DIR = os.path.join(HERE, "drives")
GEOMETRY_FILE = os.path.join(DRIVES_DIR, "camera_front_geometry.json")
BUILDINGS_FILE = os.path.join(HERE, "town10", "town10_buildings_clean.json")

# Κάμερα (ίδια σε όλες, BMW_spawn.py): 1280×960, fov 90° → εστιακή απόσταση 640 px
WIDTH, HEIGHT = 1280, 960
FOCAL = WIDTH / 2 / np.tan(np.radians(90 / 2))
CX, CY = WIDTH / 2, HEIGHT / 2
CAMERA_YAW = {"front": 0, "right": 90, "rear": 180, "left": -90}   # όπως στο BMW_spawn.py

# Ετικέτες της semantic (sources/CARLA_ref_sensors_ue4-dev.md:833–841)
BUILDING = 3
POLE_LIKE = (6, 7, 8)            # κολόνα, φανάρι, πινακίδα
VEGETATION = 9

# Ρυθμίσεις του ανιχνευτή
MAX_TILT_DEG = 5                 # πόσο μπορεί να γέρνει μια «κάθετη» γραμμή
MIN_LENGTH_PX = 80               # μόνο μακριές (κόβει παράθυρα κ.λπ.)
ABOVE_HORIZON_PX = 10            # η γραμμή πρέπει να φτάνει τουλάχιστον τόσο πάνω από τον ορίζοντα
MERGE_PX = 6                     # γραμμές τόσο κοντά ενώνονται σε μία άκρη

# Ρυθμίσεις της βαθμολόγησης
MATCH_PX = 15                    # άκρη «σωστή» αν απέχει τόσο από γωνία του χάρτη (~1,3°)
MAX_RANGE_M = 100                # γωνίες πιο μακριά δεν μετράνε (πολύ μικρές στην εικόνα)
SIDE_PX = 4                      # η semantic κοιτάζεται τόσα px αριστερά και δεξιά της άκρης


# ---------------------------------------------------------------- γεωμετρία κάμερας και χάρτης

def camera_mounts():
    """Θέση κάθε κάμερας πάνω στο αυτοκίνητο (μπροστά m, δεξιά m), με τους ίδιους τύπους με το BMW_spawn.py,
    από το κουτί του αυτοκινήτου που γράφτηκε στο camera_front_geometry.json. Και το ύψος πάνω από τον δρόμο."""
    geo = json.load(open(GEOMETRY_FILE))
    loc_x, ext_x, ext_y = geo["box"][0], geo["box"][3], geo["box"][4]
    mounts = {"front": (loc_x + 0.25 * ext_x, 0.0),
              "rear": (loc_x - 0.70 * ext_x, 0.0),
              "left": (loc_x - 0.30 * ext_x, -0.85 * ext_y),
              "right": (loc_x - 0.30 * ext_x, 0.85 * ext_y)}
    return mounts, geo["height"]


def building_boxes():
    """Τα 112 κουτιά των κτηρίων: λίστα, με τις 4 γωνίες (x, y) του καθενός."""
    boxes = []
    for b in json.load(open(BUILDINGS_FILE))["buildings"]:
        yaw = np.radians(b["yaw"])
        c, s = np.cos(yaw), np.sin(yaw)
        corners = []
        for a, d in ((1, 1), (1, -1), (-1, -1), (-1, 1)):          # με τη σειρά γύρω από το κουτί
            dx, dy = a * b["extent_x"], d * b["extent_y"]
            corners.append((b["x"] + c * dx - s * dy, b["y"] + s * dx + c * dy))
        boxes.append(np.array(corners))
    return boxes


def segments_cross(p1, p2, q1, q2):
    """Τέμνονται τα ευθύγραμμα τμήματα p1–p2 και q1–q2; (στο επίπεδο)"""
    def side(a, b, c):
        return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
    d1, d2 = side(q1, q2, p1), side(q1, q2, p2)
    d3, d4 = side(p1, p2, q1), side(p1, p2, q2)
    return (d1 * d2 < 0) and (d3 * d4 < 0)


def visible_corner_columns(boxes, cam_x, cam_y, cam_heading):
    """Σε ποιες στήλες της εικόνας πρέπει να φαίνονται γωνίες κτηρίων. Μια γωνία «φαίνεται» αν είναι μπροστά από την
    κάμερα, μέσα στην εικόνα, έως MAX_RANGE_M, και η ευθεία από την κάμερα ως αυτήν δεν περνάει μέσα από κάποιο κτήριο
    (ούτε από το δικό της: έτσι κρύβονται οι πίσω γωνίες)."""
    c, s = np.cos(cam_heading), np.sin(cam_heading)
    edges = [(box[k], box[(k + 1) % 4]) for box in boxes for k in range(4)]
    columns = []
    for box in boxes:
        for corner in box:
            dx, dy = corner[0] - cam_x, corner[1] - cam_y
            forward = c * dx + s * dy
            right = -s * dx + c * dy                    # CARLA: το δεξί της κατεύθυνσης ψ είναι (−sin ψ, cos ψ)
            dist = np.hypot(dx, dy)
            if forward < 1.0 or dist > MAX_RANGE_M:
                continue
            u = CX + FOCAL * right / forward
            if not 0 <= u < WIDTH:
                continue
            # Λίγο πιο κοντά στην κάμερα (0,3 m), για να μη «χτυπάει» στις δικές της πλευρές
            near = (corner[0] - 0.3 * dx / dist, corner[1] - 0.3 * dy / dist)
            if any(segments_cross((cam_x, cam_y), near, q1, q2) for q1, q2 in edges):
                continue
            columns.append(u)
    return np.array(sorted(columns))


# ---------------------------------------------------------------- ο ανιχνευτής (ΜΟΝΟ RGB)

def detect_vertical_edges(bgr):
    """Επιστρέφει λίστα με άκρες: (στήλη u, πάνω y, κάτω y, συνολικό μήκος px)."""
    gray = cv2.GaussianBlur(cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY), (5, 5), 0)
    edges = cv2.Canny(gray, 50, 150)
    lines = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold=60, minLineLength=MIN_LENGTH_PX, maxLineGap=5)
    if lines is None:
        return []
    kept = []
    for x1, y1, x2, y2 in lines[:, 0]:
        tilt = np.degrees(np.arctan2(abs(x2 - x1), abs(y2 - y1)))
        top, bottom = min(y1, y2), max(y1, y2)
        if tilt <= MAX_TILT_DEG and top < CY - ABOVE_HORIZON_PX:
            kept.append(((x1 + x2) / 2, top, bottom, np.hypot(x2 - x1, y2 - y1)))
    kept.sort()
    # Ένωση γραμμών στην ίδια περίπου στήλη (π.χ. μια γωνία που κόπηκε σε κομμάτια)
    merged = []
    for u, top, bottom, length in kept:
        if merged and u - merged[-1][0] < MERGE_PX:
            mu, mtop, mbottom, mlength = merged[-1]
            total = mlength + length
            merged[-1] = ((mu * mlength + u * length) / total, min(mtop, top), max(mbottom, bottom), total)
        else:
            merged.append((u, top, bottom, length))
    return merged


# ---------------------------------------------------------------- βαθμολόγηση με τη semantic

def semantic_label(tags, u, top, bottom):
    """Τι είναι η άκρη σύμφωνα με τη semantic: B, F, P, T ή O (βλ. πάνω). Ψηφοφορία σε σημεία κατά μήκος της."""
    u = int(round(u))
    left = max(u - SIDE_PX, 0)
    right = min(u + SIDE_PX, WIDTH - 1)
    votes = Counter()
    for y in range(int(top), int(bottom) + 1, 4):
        l_bld, r_bld = tags[y, left] == BUILDING, tags[y, right] == BUILDING
        on = tags[y, u]
        if l_bld != r_bld:
            votes["B"] += 1
        elif l_bld and r_bld:
            votes["F"] += 1
        elif on in POLE_LIKE or tags[y, left] in POLE_LIKE or tags[y, right] in POLE_LIKE:
            votes["P"] += 1
        elif on == VEGETATION:
            votes["T"] += 1
        else:
            votes["O"] += 1
    return votes.most_common(1)[0][0] if votes else "O"


# ---------------------------------------------------------------- αρχεία

def image_folder(drive, camera, kind):
    """drives/<καταγραφή>/<κάμερα>_<είδος>/ ή, για παλιές καταγραφές, στον κεντρικό φάκελο."""
    for base in (DRIVES_DIR, HERE):
        folder = os.path.join(base, drive, f"{camera}_{kind}")
        if os.path.isdir(folder):
            return folder
    return None


def load_poses(drive):
    """frame → (x, y, yaw σε rad) από το CSV της καταγραφής."""
    data = np.genfromtxt(os.path.join(DRIVES_DIR, drive + ".csv"), delimiter=",", names=True)
    return {int(f): (x, y, np.radians(yaw)) for f, x, y, yaw in
            zip(data["frame"], data["gt_x"], data["gt_y"], data["gt_yaw"])}


def debug_image(bgr, found, labels, matched, map_columns, out_path):
    img = bgr.copy()
    cv2.line(img, (0, int(CY)), (WIDTH - 1, int(CY)), (255, 0, 0), 1)
    for u in map_columns:
        cv2.line(img, (int(u), 0), (int(u), HEIGHT - 1), (0, 255, 255), 1)
    for (u, top, bottom, _), label, ok in zip(found, labels, matched):
        color = (0, 200, 0) if ok else (0, 0, 255)
        cv2.line(img, (int(u), int(top)), (int(u), int(bottom)), color, 3)
        cv2.putText(img, label, (int(u) - 6, max(int(top) - 6, 14)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
    cv2.imwrite(out_path, img, [cv2.IMWRITE_JPEG_QUALITY, 85])


# ---------------------------------------------------------------- κύριο

def run_camera(drive, camera, n, poses, boxes, mounts):
    rgb_dir = image_folder(drive, camera, "rgb")
    sem_dir = image_folder(drive, camera, "semantic")
    if rgb_dir is None:
        print(f"\n[{camera}] Δεν βρήκα εικόνες RGB ({drive}/{camera}_rgb). Παραλείπεται.")
        return
    frames = sorted(int(os.path.basename(p)[:-4]) for p in glob.glob(os.path.join(rgb_dir, "*.jpg")))
    frames = [f for f in frames if any(g in poses for g in (f, f - 1, f + 1))]
    if not frames:
        print(f"\n[{camera}] Καμία εικόνα δεν ταιριάζει με frame του CSV. Παραλείπεται.")
        return
    chosen = [frames[k] for k in np.linspace(0, len(frames) - 1, min(n, len(frames))).astype(int)]
    out_dir = os.path.join(DRIVES_DIR, "building_edge_test", f"{drive}_{camera}")
    os.makedirs(out_dir, exist_ok=True)

    mount_forward, mount_right = mounts[camera]
    n_edges = n_matched = n_map = n_map_found = 0
    label_all, label_matched = Counter(), Counter()
    bearing_errors = []
    for frame in chosen:
        x, y, yaw = next(poses[g] for g in (frame, frame - 1, frame + 1) if g in poses)
        cam_x = x + mount_forward * np.cos(yaw) - mount_right * np.sin(yaw)
        cam_y = y + mount_forward * np.sin(yaw) + mount_right * np.cos(yaw)
        map_columns = visible_corner_columns(boxes, cam_x, cam_y, yaw + np.radians(CAMERA_YAW[camera]))

        bgr = cv2.imread(os.path.join(rgb_dir, f"{frame}.jpg"))
        found = detect_vertical_edges(bgr)

        sem_path = None if sem_dir is None else os.path.join(sem_dir, f"{frame}.png")
        tags = cv2.imread(sem_path, cv2.IMREAD_UNCHANGED) if sem_path and os.path.exists(sem_path) else None
        labels = [semantic_label(tags, u, t, b) if tags is not None else "?" for u, t, b, _ in found]

        matched = []
        for (u, _, _, _), label in zip(found, labels):
            gap = np.min(np.abs(map_columns - u)) if len(map_columns) else np.inf
            ok = gap <= MATCH_PX
            matched.append(ok)
            label_all[label] += 1
            if ok:
                label_matched[label] += 1
                nearest = map_columns[np.argmin(np.abs(map_columns - u))]
                bearing_errors.append(np.degrees(np.arctan((u - CX) / FOCAL) - np.arctan((nearest - CX) / FOCAL)))
        n_edges += len(found)
        n_matched += sum(matched)
        n_map += len(map_columns)
        n_map_found += sum(any(abs(u - mc) <= MATCH_PX for u, _, _, _ in found) for mc in map_columns)

        debug_image(bgr, found, labels, matched, map_columns, os.path.join(out_dir, f"{frame}.jpg"))

    k = len(chosen)
    print(f"\n[{camera}] {k} εικόνες ({drive})")
    print(f"  άκρες που βρήκε: {n_edges} (≈ {n_edges / k:.1f} ανά εικόνα)")
    if n_edges:
        print(f"  ταιριάζουν με γωνία του χάρτη (≤ {MATCH_PX} px): {n_matched} ({100 * n_matched / n_edges:.0f}%)")
    if n_map:
        print(f"  γωνίες του χάρτη που φαίνονται: {n_map} (≈ {n_map / k:.1f} ανά εικόνα), "
              f"βρέθηκαν: {n_map_found} ({100 * n_map_found / n_map:.0f}%)")
    if bearing_errors:
        e = np.array(bearing_errors)
        print(f"  λάθος γωνίας στις σωστές: μέσο {e.mean():+.2f}°, διάμεσο |λάθος| {np.median(np.abs(e)):.2f}°")
    names = {"B": "όριο κτηρίου", "F": "μέσα σε κτήριο", "P": "κολόνα/πινακίδα", "T": "δέντρο", "O": "άλλο",
             "?": "χωρίς semantic"}
    print("  τι είναι οι άκρες (semantic): όλες / όσες ταιριάζουν με τον χάρτη")
    for key in ("B", "F", "P", "T", "O", "?"):
        if label_all[key]:
            print(f"    {names[key]:16s} {label_all[key]:4d} / {label_matched[key]:4d}")
    print(f"  εικόνες ελέγχου: {out_dir}")


def main():
    parser = argparse.ArgumentParser(description="Δοκιμή: κάθετες άκρες κτηρίων μόνο από RGB (επιλογή Α).")
    parser.add_argument("--drive", default="drive_2026-10-30_20-45-55", help="όνομα καταγραφής (χωρίς .csv)")
    parser.add_argument("--camera", default="front", choices=["front", "left", "right", "rear", "all"])
    parser.add_argument("--n", type=int, default=20, help="πόσες εικόνες (ομοιόμορφα σε όλη τη διαδρομή)")
    args = parser.parse_args()

    poses = load_poses(args.drive)
    boxes = building_boxes()
    mounts, _ = camera_mounts()
    cameras = ["front", "left", "right", "rear"] if args.camera == "all" else [args.camera]
    for camera in cameras:
        run_camera(args.drive, camera, args.n, poses, boxes, mounts)


if __name__ == "__main__":
    main()
