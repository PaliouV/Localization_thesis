"""
Έλεγχος για τα κτήρια (11 Νοε 2026): τι βλέπει η SEMANTIC κάμερα από τα κτήρια, και πόσο ταιριάζει με τον χάρτη.
Δεν είναι ακόμα μέρος του φίλτρου. Απαντάει σε δύο ερωτήσεις, πριν διαλέξουμε τι θα μετράει η «κάμερα κτηρίων»:
  (Α) «κτήριο ή κενό» ανά στήλη της εικόνας: σε πόσες στήλες συμφωνεί η κάμερα με τον χάρτη;
  (Β) άκρες κτηρίων (εκεί που το κτήριο αρχίζει ή τελειώνει, π.χ. σε κάθετο δρόμο): πόσες φαίνονται σε κάθε εικόνα,
      πόσο συχνά, και πόσες ταιριάζουν με άκρη του χάρτη;

Πώς δουλεύει, για κάθε εικόνα:
  1. Κάμερα: κοιτάμε μια οριζόντια ΛΩΡΙΔΑ της εικόνας λίγο πάνω από τον ορίζοντα (BAND_TOP..BAND_BOTTOM px πάνω από
     τη μέση). Εκεί δεν φτάνουν τα αυτοκίνητα, αλλά φαίνονται τα κτήρια, κοντινά και μακρινά. Κάθε στήλη γίνεται:
        κτήριο  (ετικέτα Building ή Wall)
        κρυμμένο (δέντρο, κολόνα, φανάρι, πινακίδα, όχημα, πεζός: κάτι μπροστά που δεν ξέρουμε τι κρύβει)
        ανοιχτό (ουρανός, δρόμος και ό,τι άλλο)
     Πολύ στενά κομμάτια (< MIN_RUN px) σβήνονται. «Άκρη» = στήλη όπου το κτήριο συναντάει ΑΝΟΙΧΤΟ.
     Αν ανάμεσα υπάρχει κρυμμένο, μετριέται χωριστά ως «κρυμμένη άκρη» (δεν ξέρουμε πού ακριβώς τελειώνει το κτήριο).
  2. Χάρτης: από την ΑΛΗΘΙΝΗ θέση της κάμερας ρίχνουμε μια ακτίνα για κάθε στήλη πάνω στον χάρτη κτηρίων (κάτοψη με
     ύψος σε κάθε κελί). Η ακτίνα «χτυπάει κτήριο» στο πρώτο κελί που είναι αρκετά ΨΗΛΟ ώστε να φαίνεται μέσα στη
     λωρίδα της εικόνας από εκείνη την απόσταση (ένα μακρινό κτήριο πρέπει να είναι ψηλό για να φτάνει ως εκεί).
     Δύο χάρτες, για σύγκριση:
        lidar:  town10/town10_lidar_map.npz (πραγματικοί τοίχοι· ύψος κελιού = το ψηλότερο σημείο του)
        boxes:  town10/town10_buildings_clean.json (112 χοντρά κουτιά· ύψος = η κορυφή του κουτιού)
     ⚠️ Υποθέτει ότι ο δρόμος είναι στο ύψος z = ROAD_Z παντού (οι καταγραφές δεν έχουν το z του αυτοκινήτου).
  3. Σύγκριση και εκτύπωση. Γράφει και λίγες εικόνες ελέγχου (μισό μέγεθος).

Εκτέλεση (στον σταθερό, όπου είναι οι εικόνες):
    ~/carla-env/bin/python building_corner_check.py --drive drive_2026-10-30_20-45-55
    ~/carla-env/bin/python building_corner_check.py --drive drive_2026-10-30_20-45-55 --camera front --n 500
Έξοδος: building_corner_check/summary_<καταγραφή>.txt και building_corner_check/<κάμερα>_<καταγραφή>/<frame>.jpg
  Εικόνα ελέγχου: η semantic χρωματισμένη (κτήριο γκρι, ουρανός γαλάζιο, δέντρα πράσινο, δρόμος μοβ).
  Πάνω: δύο λωρίδες, 1η = τι λέει η ΚΑΜΕΡΑ, 2η = τι λέει ο ΧΑΡΤΗΣ lidar (άσπρο κτήριο, μαύρο ανοιχτό, πορτοκαλί κρυμμένο).
  Κάθετες γραμμές: πράσινη = άκρη της κάμερας που ταιριάζει με τον χάρτη, κόκκινη = δεν ταιριάζει,
  κίτρινη λεπτή = άκρη του χάρτη. Οι δύο άσπρες οριζόντιες = η λωρίδα που κοιτάμε.
"""

import argparse
import glob
import json
import os

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
DRIVES_DIR = os.path.join(HERE, "drives")
OUT_DIR = os.path.join(HERE, "building_corner_check")
GEOMETRY_FILE = os.path.join(DRIVES_DIR, "camera_front_geometry.json")
BUILDINGS_FILE = os.path.join(HERE, "town10", "town10_buildings_clean.json")
LIDAR_FILE = os.path.join(HERE, "town10", "town10_lidar_map.npz")

# Κάμερα (ίδια σε όλες, BMW_spawn.py): 1280×960, fov 90° → εστιακή απόσταση 640 px
WIDTH, HEIGHT = 1280, 960
FOCAL = WIDTH / 2 / np.tan(np.radians(90 / 2))
CX, CY = WIDTH / 2, HEIGHT / 2
CAMERA_YAW = {"front": 0, "right": 90, "rear": 180, "left": -90}   # όπως στο BMW_spawn.py

# Ετικέτες της semantic (sources/CARLA_ref_sensors_ue4-dev.md:829–850)
BUILDING_TAGS = (3, 4)                               # Building, Wall (ο χάρτης lidar έχει και τα δύο)
HIDING_TAGS = (6, 7, 8, 9, 12, 13, 14, 15, 16, 17, 18, 19)   # κολόνα, φανάρι, πινακίδα, δέντρα, πεζοί, οχήματα

# Η λωρίδα της εικόνας που κοιτάμε: από 60 έως 12 px πάνω από τον ορίζοντα (1°–5° προς τα πάνω).
# Στα 10 m αυτό είναι ~2–2,7 m από το έδαφος (πάνω από τα αυτοκίνητα), στα 100 m ~3,5–11 m.
BAND_TOP, BAND_BOTTOM = 60, 12
MIN_RUN = 8             # px: κομμάτια πιο στενά από αυτό σβήνονται (~0,7°)
MATCH_PX = 15           # px: άκρη «ταιριάζει» αν απέχει έως τόσο από άκρη του χάρτη (~1,3°)
BORDER_PX = 10          # px: άκρες τόσο κοντά στο πλάι της εικόνας δεν μετράνε

# Χάρτης
CELL = 0.5              # m: κελί του πλέγματος κάτοψης
MAX_RANGE_M = 400       # m: πόσο μακριά ψάχνει η ακτίνα για κτήριο
RAY_STEP = 0.25         # m
ROAD_Z = 0.0            # m: ύψος του δρόμου στον χάρτη (υπόθεση: επίπεδη πόλη)

BUILDING, OPEN, HIDDEN = 1, 0, 2

# (Γ) Λεπτά αντικείμενα (12 Νοε 2026): πόσα βλέπει η κάμερα, για να δούμε αν αξίζουν ως σημάδια αντί για (ή μαζί με) κτήρια.
# Δεν υπάρχει ακόμα χάρτης τους, άρα εδώ ΜΟΝΟ μετράμε πόσα φαίνονται. Όνομα: (ετικέτα, ελάχιστα px ύψος σε μια στήλη).
# Με εστιακή 640 px: κολόνα 5 m φαίνεται 30 px στα ~105 m· φανάρι 1 m φαίνεται 10 px στα ~65 m· πινακίδα 0,6 m 8 px στα ~50 m.
THIN_OBJECTS = {"κολόνες": (6, 30), "φανάρια": (7, 10), "πινακίδες": (8, 8)}
THIN_MIN_WIDTH = 2      # px: πιο στενές ομάδες στηλών δεν μετράνε


# ---------------------------------------------------------------- χάρτης: πλέγμα κάτοψης και ακτίνες

def grid_from_lidar():
    """Πλέγμα κάτοψης από τα σημεία του lidar: σε κάθε κελί, το ύψος (z) του ψηλότερου σημείου."""
    points = np.load(LIDAR_FILE)["points"]
    return make_grid(points[:, 0], points[:, 1], points[:, 2])


def grid_from_boxes():
    """Το ίδιο πλέγμα από τα 112 κουτιά: γεμίζουμε κάθε κουτί με σημεία ανά CELL/2."""
    xs, ys, zs = [], [], []
    for b in json.load(open(BUILDINGS_FILE))["buildings"]:
        yaw = np.radians(b["yaw"])
        a = np.arange(-b["extent_x"], b["extent_x"] + CELL / 2, CELL / 2)
        d = np.arange(-b["extent_y"], b["extent_y"] + CELL / 2, CELL / 2)
        a, d = np.meshgrid(a, d)
        xs.append((b["x"] + np.cos(yaw) * a - np.sin(yaw) * d).ravel())
        ys.append((b["y"] + np.sin(yaw) * a + np.cos(yaw) * d).ravel())
        zs.append(np.full(a.size, b["top_z"]))
    return make_grid(np.concatenate(xs), np.concatenate(ys), np.concatenate(zs))


def make_grid(x, y, z):
    """cells[i, j] = το μεγαλύτερο z μέσα στο κελί (−inf αν είναι άδειο)."""
    x0, y0 = np.floor(x.min()) - 2, np.floor(y.min()) - 2
    nx, ny = int((x.max() - x0) / CELL) + 5, int((y.max() - y0) / CELL) + 5
    cells = np.full((nx, ny), -np.inf, dtype=np.float32)
    np.maximum.at(cells, (((x - x0) / CELL).astype(int), ((y - y0) / CELL).astype(int)), z)
    return {"x0": x0, "y0": y0, "cells": cells}


def map_profile(grid, cam_x, cam_y, cam_heading, cam_height):
    """Για κάθε στήλη της εικόνας: χτυπάει η ακτίνα κτήριο; (BUILDING / OPEN) και σε πόσα μέτρα (nan αν όχι).
    «Χτυπάει» = κελί με ύψος τουλάχιστον όσο η ΜΕΣΗ της λωρίδας της εικόνας σε εκείνη την απόσταση
    (τότε το κτήριο γεμίζει πάνω από τη μισή λωρίδα, όπως ζητάει και το camera_profile).
    Η στήλη u κοιτάει υπό γωνία atan((u − CX) / FOCAL) δεξιά της κάμερας. Στο CARLA το «δεξιά» της κατεύθυνσης ψ είναι
    (−sin ψ, cos ψ), άρα η ακτίνα έχει κατεύθυνση ψ + γωνία."""
    offset = np.arctan((np.arange(WIDTH) - CX) / FOCAL)
    angle = cam_heading + offset
    steps = np.arange(1.0, MAX_RANGE_M, RAY_STEP)
    px = cam_x + np.outer(np.cos(angle), steps)
    py = cam_y + np.outer(np.sin(angle), steps)
    ix = ((px - grid["x0"]) / CELL).astype(int)
    iy = ((py - grid["y0"]) / CELL).astype(int)
    cells = grid["cells"]
    inside = (ix >= 0) & (ix < cells.shape[0]) & (iy >= 0) & (iy < cells.shape[1])
    depth = np.outer(np.cos(offset), steps)               # απόσταση «προς τα μπρος» της κάμερας
    needed = ROAD_Z + cam_height + depth * ((BAND_TOP + BAND_BOTTOM) / 2) / FOCAL
    hit = np.zeros(ix.shape, dtype=bool)
    hit[inside] = cells[ix[inside], iy[inside]] >= needed[inside]
    any_hit = hit.any(axis=1)
    distance = np.where(any_hit, steps[np.argmax(hit, axis=1)], np.nan)
    return np.where(any_hit, BUILDING, OPEN), distance


# ---------------------------------------------------------------- κάμερα: προφίλ από τη semantic εικόνα

def camera_profile(tags):
    """Για κάθε στήλη: BUILDING / OPEN / HIDDEN, από τη λωρίδα λίγο πάνω από τον ορίζοντα (ό,τι υπερισχύει)."""
    band = tags[int(CY) - BAND_TOP:int(CY) - BAND_BOTTOM, :]
    building = np.isin(band, BUILDING_TAGS).mean(axis=0)
    hidden = np.isin(band, HIDING_TAGS).mean(axis=0)
    open_ = 1 - building - hidden
    return np.argmax(np.stack([open_, building, hidden]), axis=0)      # 0 OPEN, 1 BUILDING, 2 HIDDEN


def remove_short_runs(profile):
    """Κομμάτια πιο στενά από MIN_RUN px παίρνουν την τιμή του αριστερού τους γείτονα (ή του δεξιού, στην άκρη)."""
    profile = profile.copy()
    starts = np.flatnonzero(np.diff(profile)) + 1
    starts = np.concatenate([[0], starts, [len(profile)]])
    for a, b in zip(starts[:-1], starts[1:]):
        if b - a < MIN_RUN:
            profile[a:b] = profile[a - 1] if a > 0 else profile[min(b, len(profile) - 1)]
    return profile


def edges_of(profile):
    """Οι άκρες ενός προφίλ. Επιστρέφει (καθαρές, κρυμμένες): λίστες με στήλες u.
    Καθαρή: κτήριο δίπλα σε ανοιχτό. Κρυμμένη: κτήριο – κρυμμένο – ανοιχτό (στη μέση του κρυμμένου κομματιού)."""
    change = np.flatnonzero(np.diff(profile)) + 1
    runs = np.concatenate([[0], change, [len(profile)]])
    labels = [profile[a] for a in runs[:-1]]
    clean, hidden = [], []
    for k in range(1, len(labels)):
        u = runs[k]
        if {labels[k - 1], labels[k]} == {BUILDING, OPEN}:
            clean.append(float(u))
        elif labels[k] == HIDDEN and 0 < k < len(labels) - 1 and {labels[k - 1], labels[k + 1]} == {BUILDING, OPEN}:
            hidden.append((runs[k] + runs[k + 1]) / 2)
    inside = lambda us: [u for u in us if BORDER_PX <= u < WIDTH - BORDER_PX]
    return inside(clean), inside(hidden)


def thin_objects(tags, tag, min_pixels):
    """Τα αντικείμενα μιας ετικέτας σε όλη την εικόνα, ως ομάδες από διπλανές στήλες.
    Μια στήλη μετράει αν έχει τουλάχιστον min_pixels px με αυτή την ετικέτα. Επιστρέφει τα πλάτη των ομάδων (px).
    ⚠️ Δύο αντικείμενα που ακουμπάνε στην εικόνα μετράνε ως ένα· ένα φανάρι με οριζόντιο βραχίονα βγαίνει φαρδύ."""
    has = np.concatenate([[False], (tags == tag).sum(axis=0) >= min_pixels, [False]])
    change = np.flatnonzero(np.diff(has.astype(int)))
    widths = change[1::2] - change[0::2]
    return widths[widths >= THIN_MIN_WIDTH]


# ---------------------------------------------------------------- αρχεία και γεωμετρία

def camera_mounts():
    """Θέση κάθε κάμερας πάνω στο αυτοκίνητο (μπροστά m, δεξιά m), με τους ίδιους τύπους με το BMW_spawn.py,
    και το ύψος της πάνω από τον δρόμο (ίδιο για όλες)."""
    geo = json.load(open(GEOMETRY_FILE))
    loc_x, ext_x, ext_y = geo["box"][0], geo["box"][3], geo["box"][4]
    return {"front": (loc_x + 0.25 * ext_x, 0.0), "rear": (loc_x - 0.70 * ext_x, 0.0),
            "left": (loc_x - 0.30 * ext_x, -0.85 * ext_y), "right": (loc_x - 0.30 * ext_x, 0.85 * ext_y)}, geo["height"]


def image_folder(drive, camera):
    """drives/<καταγραφή>/<κάμερα>_semantic/ ή, για παλιές καταγραφές, στον κεντρικό φάκελο."""
    for base in (DRIVES_DIR, HERE):
        folder = os.path.join(base, drive, f"{camera}_semantic")
        if os.path.isdir(folder):
            return folder
    return None


def load_poses(drive):
    """frame → (x, y, yaw σε rad) από το CSV της καταγραφής."""
    data = np.genfromtxt(os.path.join(DRIVES_DIR, drive + ".csv"), delimiter=",", names=True)
    return {int(f): (x, y, np.radians(yaw)) for f, x, y, yaw in
            zip(data["frame"], data["gt_x"], data["gt_y"], data["gt_yaw"])}


def read_tags(path):
    import cv2
    tags = cv2.imread(path, cv2.IMREAD_UNCHANGED)
    return tags if tags.ndim == 2 else tags[:, :, 2]     # αν σώθηκε έγχρωμη: η ετικέτα είναι στο κόκκινο κανάλι (BGR)


PALETTE = {0: (0, 0, 0), 1: (128, 64, 128), 2: (232, 35, 244), 3: (70, 70, 70), 4: (156, 102, 102), 9: (35, 142, 107),
           10: (152, 251, 152), 11: (180, 130, 70), 14: (142, 0, 0), 24: (50, 234, 157)}     # BGR


def debug_image(tags, cam, cam_edges, matched, map_prof, map_edges, path):
    import cv2
    img = np.full((HEIGHT, WIDTH, 3), 90, dtype=np.uint8)
    for tag, color in PALETTE.items():
        img[tags == tag] = color
    img[np.isin(tags, HIDING_TAGS) & (tags != 9) & (tags != 14)] = (0, 140, 255)
    strip = {OPEN: (0, 0, 0), BUILDING: (255, 255, 255), HIDDEN: (0, 140, 255)}
    for row, prof in ((0, cam), (26, map_prof)):
        for value, color in strip.items():
            img[row:row + 22, prof == value] = color
    for y in (int(CY) - BAND_TOP, int(CY) - BAND_BOTTOM):
        cv2.line(img, (0, y), (WIDTH - 1, y), (255, 255, 255), 1)
    for u in map_edges:
        cv2.line(img, (int(u), 50), (int(u), HEIGHT - 1), (0, 255, 255), 1)
    for u, ok in zip(cam_edges, matched):
        cv2.line(img, (int(u), 50), (int(u), int(CY) + 60), (0, 200, 0) if ok else (0, 0, 255), 3)
    cv2.imwrite(path, cv2.resize(img, (WIDTH // 2, HEIGHT // 2), interpolation=cv2.INTER_AREA),
                [cv2.IMWRITE_JPEG_QUALITY, 80])


# ---------------------------------------------------------------- σύγκριση για μία κάμερα

def run_camera(drive, camera, n, n_debug, poses, grids, mounts, cam_height, say):
    folder = image_folder(drive, camera)
    if folder is None:
        say(f"\n[{camera}] Δεν βρήκα εικόνες ({drive}/{camera}_semantic). Παραλείπεται.")
        return
    frames = sorted(int(os.path.basename(p)[:-4]) for p in glob.glob(os.path.join(folder, "*.png")))
    frames = [f for f in frames if any(g in poses for g in (f, f - 1, f + 1))]
    if not frames:
        say(f"\n[{camera}] Καμία εικόνα δεν ταιριάζει με frame του CSV. Παραλείπεται.")
        return
    chosen = [frames[k] for k in np.linspace(0, len(frames) - 1, min(n, len(frames))).astype(int)]
    debug_frames = set(chosen[k] for k in np.linspace(0, len(chosen) - 1, min(n_debug, len(chosen))).astype(int))
    out_dir = os.path.join(OUT_DIR, f"{camera}_{drive}")
    if debug_frames:
        os.makedirs(out_dir, exist_ok=True)

    forward, right = mounts[camera]
    share = np.zeros(3)                                   # ανοιχτό / κτήριο / κρυμμένο, στήλες της κάμερας
    stats = {name: dict(same=0, mirrored=0, compared=0, cam_only=0, map_only=0, cam_edges=0, cam_matched=0,
                        map_edges=0, map_found=0, errors=[], distances=[]) for name in grids}
    clean_per_image, hidden_per_image = [], []
    thin_counts = {name: [] for name in THIN_OBJECTS}     # πόσα ανά εικόνα
    thin_widths = {name: [] for name in THIN_OBJECTS}     # πλάτος κάθε αντικειμένου (px)
    for frame in chosen:
        x, y, yaw = next(poses[g] for g in (frame, frame - 1, frame + 1) if g in poses)
        cam_x = x + forward * np.cos(yaw) - right * np.sin(yaw)
        cam_y = y + forward * np.sin(yaw) + right * np.cos(yaw)
        tags = read_tags(os.path.join(folder, f"{frame}.png"))
        cam = remove_short_runs(camera_profile(tags))
        cam_clean, cam_hidden = edges_of(cam)
        share += np.bincount(cam, minlength=3)
        clean_per_image.append(len(cam_clean))
        hidden_per_image.append(len(cam_hidden))
        for name, (tag, min_pixels) in THIN_OBJECTS.items():
            widths = thin_objects(tags, tag, min_pixels)
            thin_counts[name].append(len(widths))
            thin_widths[name].extend(widths)
        seen = cam != HIDDEN                             # συγκρίνουμε μόνο όπου η κάμερα βλέπει κάτι

        for name, grid in grids.items():
            s = stats[name]
            raw, distance = map_profile(grid, cam_x, cam_y, yaw + np.radians(CAMERA_YAW[camera]), cam_height)
            prof = remove_short_runs(raw)
            s["compared"] += seen.sum()
            s["same"] += np.sum(cam[seen] == prof[seen])
            s["mirrored"] += np.sum(cam[seen] == prof[::-1][seen])
            s["cam_only"] += np.sum((cam == BUILDING) & (prof == OPEN))
            s["map_only"] += np.sum((cam == OPEN) & (prof == BUILDING))
            map_edges = np.array(edges_of(prof)[0])
            # άκρες του χάρτη που η κάμερα θα μπορούσε να δει (όχι πίσω από δέντρο κ.λπ.)
            visible = [u for u in map_edges if not np.all(cam[max(int(u) - MATCH_PX, 0):int(u) + MATCH_PX] == HIDDEN)]
            matched = []
            for u in cam_clean:
                gap = np.min(np.abs(map_edges - u)) if len(map_edges) else np.inf
                matched.append(gap <= MATCH_PX)
                if gap <= MATCH_PX:
                    nearest = map_edges[np.argmin(np.abs(map_edges - u))]
                    s["errors"].append(np.degrees(np.arctan((u - CX) / FOCAL) - np.arctan((nearest - CX) / FOCAL)))
                    near = distance[max(int(nearest) - 3, 0):int(nearest) + 4]
                    if np.isfinite(near).any():
                        s["distances"].append(np.nanmin(near))
            s["cam_edges"] += len(cam_clean)
            s["cam_matched"] += sum(matched)
            s["map_edges"] += len(visible)
            s["map_found"] += sum(any(abs(u - c) <= MATCH_PX for c in cam_clean) for u in visible)
            if name == "lidar" and frame in debug_frames:
                try:
                    debug_image(tags, cam, cam_clean, matched, prof, map_edges, os.path.join(out_dir, f"{frame}.jpg"))
                except Exception as error:      # μια αποτυχημένη εικόνα ελέγχου δεν πρέπει να σταματάει τις μετρήσεις
                    print(f"  (η εικόνα ελέγχου {frame} δεν γράφτηκε: {error})")

    k = len(chosen)
    share = 100 * share / share.sum()
    clean_per_image = np.array(clean_per_image)
    say(f"\n[{camera}] {k} εικόνες ({drive})")
    say(f"  στήλες της κάμερας: κτήριο {share[BUILDING]:.0f}%, ανοιχτό {share[OPEN]:.0f}%, κρυμμένο {share[HIDDEN]:.0f}%")
    say(f"  καθαρές άκρες κτηρίων: {clean_per_image.mean():.1f} ανά εικόνα· εικόνες με καμία {100 * np.mean(clean_per_image == 0):.0f}%, "
        f"με 1+ {100 * np.mean(clean_per_image >= 1):.0f}%, με 2+ {100 * np.mean(clean_per_image >= 2):.0f}%")
    say(f"  κρυμμένες άκρες (πίσω από δέντρο, κολόνα, όχημα): {np.mean(hidden_per_image):.1f} ανά εικόνα")
    for name, s in stats.items():
        say(f"  χάρτης {name}:")
        if s["compared"]:
            same, mirrored = 100 * s["same"] / s["compared"], 100 * s["mirrored"] / s["compared"]
            say(f"    (Α) ίδιο «κτήριο ή ανοιχτό» με την κάμερα: {same:.1f}% των στηλών "
                f"(κάμερα κτήριο / χάρτης όχι: {100 * s['cam_only'] / s['compared']:.1f}%, "
                f"χάρτης κτήριο / κάμερα όχι: {100 * s['map_only'] / s['compared']:.1f}%)")
            if mirrored > same:
                say(f"    ⚠️ με τον χάρτη ΑΝΑΠΟΔΑ (δεξιά-αριστερά) ταιριάζει περισσότερο ({mirrored:.1f}%): λάθος στη γεωμετρία;")
        if s["cam_edges"]:
            say(f"    (Β) άκρες της κάμερας που ταιριάζουν με άκρη του χάρτη (≤ {MATCH_PX} px): "
                f"{s['cam_matched']} από {s['cam_edges']} ({100 * s['cam_matched'] / s['cam_edges']:.0f}%)")
        if s["map_edges"]:
            say(f"        άκρες του χάρτη που βρήκε η κάμερα: {s['map_found']} από {s['map_edges']} "
                f"({100 * s['map_found'] / s['map_edges']:.0f}%)")
        if s["errors"]:
            e, d = np.array(s["errors"]), np.array(s["distances"])
            say(f"        λάθος γωνίας στις σωστές: μέσο {e.mean():+.2f}°, διάμεσο |λάθος| {np.median(np.abs(e)):.2f}°"
                + (f"· απόσταση της άκρης: διάμεσος {np.median(d):.0f} m (25%–75%: {np.percentile(d, 25):.0f}–{np.percentile(d, 75):.0f} m)"
                   if len(d) else ""))
    say("  (Γ) λεπτά αντικείμενα που βλέπει η κάμερα (μόνο μέτρημα, χωρίς χάρτη):")
    for name in THIN_OBJECTS:
        c = np.array(thin_counts[name])
        width = f"· πλάτος διάμεσος {np.median(thin_widths[name]):.0f} px" if thin_widths[name] else ""
        say(f"    {name}: {c.mean():.1f} ανά εικόνα· εικόνες με καμία {100 * np.mean(c == 0):.0f}%, "
            f"με 1+ {100 * np.mean(c >= 1):.0f}%, με 2+ {100 * np.mean(c >= 2):.0f}%{width}")
    any_thin = np.sum([thin_counts[name] for name in THIN_OBJECTS], axis=0)
    say(f"    όλα μαζί: εικόνες με κανένα {100 * np.mean(any_thin == 0):.0f}%, με 2+ {100 * np.mean(any_thin >= 2):.0f}%")
    if debug_frames:
        say(f"  εικόνες ελέγχου: {out_dir}")


def main():
    parser = argparse.ArgumentParser(description="Τι βλέπει η semantic κάμερα από τα κτήρια, απέναντι στον χάρτη.")
    parser.add_argument("--drive", default="drive_2026-10-30_20-45-55", help="όνομα καταγραφής (χωρίς .csv)")
    parser.add_argument("--camera", default="all", choices=["front", "left", "right", "rear", "all"])
    parser.add_argument("--n", type=int, default=300, help="πόσες εικόνες ανά κάμερα (ομοιόμορφα σε όλη τη διαδρομή)")
    parser.add_argument("--debug", type=int, default=6, help="πόσες εικόνες ελέγχου ανά κάμερα")
    args = parser.parse_args()

    os.makedirs(OUT_DIR, exist_ok=True)
    lines = []

    def say(text):
        print(text, flush=True)
        lines.append(text)

    poses = load_poses(args.drive)
    grids = {"lidar": grid_from_lidar(), "boxes": grid_from_boxes()}
    mounts, cam_height = camera_mounts()
    say(f"Ρυθμίσεις: λωρίδα {BAND_TOP}–{BAND_BOTTOM} px πάνω από τον ορίζοντα, MIN_RUN {MIN_RUN} px, MATCH {MATCH_PX} px, "
        f"ακτίνα έως {MAX_RANGE_M} m, ύψος κάμερας {cam_height:.2f} m, δρόμος στο z = {ROAD_Z:.1f} m")
    cameras = ["front", "left", "right", "rear"] if args.camera == "all" else [args.camera]
    for camera in cameras:
        run_camera(args.drive, camera, args.n, args.debug, poses, grids, mounts, cam_height, say)
    with open(os.path.join(OUT_DIR, f"summary_{args.drive}.txt"), "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")
    print(f"\nΓράφτηκε: {os.path.join(OUT_DIR, f'summary_{args.drive}.txt')}")


if __name__ == "__main__":
    main()
