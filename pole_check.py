"""
Έλεγχος για κολόνες, φανάρια, πινακίδες (12 Νοε 2026): αυτά που βλέπει η SEMANTIC κάμερα ταιριάζουν με τον χάρτη τους;
Δεν είναι ακόμα μέρος του φίλτρου. Ίδια λογική με το building_corner_check.py, για λεπτά αντικείμενα.

Πώς δουλεύει, για κάθε εικόνα:
  1. Κάμερα: για κάθε είδος (ετικέτα 6 κολόνα, 7 φανάρι, 8 πινακίδα) βρίσκουμε ομάδες από διπλανές στήλες που έχουν αρκετά
     px αυτής της ετικέτας. Κάθε ομάδα = ένα αντικείμενο· κρατάμε τη μεσαία στήλη του (= γωνία προς το αντικείμενο).
  2. Χάρτης (town10/town10_poles.json, από το town10/export_poles.py): από την ΑΛΗΘΙΝΗ θέση της κάμερας, κάθε αντικείμενο
     που είναι μπροστά, μέσα στην εικόνα και έως MAX_RANGE μακριά γίνεται μια στήλη u. Όσα κρύβονται πίσω από κτήριο
     (χάρτης lidar) δεν μετράνε.
  3. Ταίριασμα: αντικείμενο της κάμερας «ταιριάζει» αν υπάρχει αντικείμενο του χάρτη έως MATCH_PX στήλες δίπλα.
     Το μετράμε δύο φορές: με το ΙΔΙΟ είδος, και με ΟΠΟΙΟΔΗΠΟΤΕ είδος (γιατί π.χ. ο στύλος ενός φαναριού μπορεί να έχει
     ετικέτα «κολόνα» στην εικόνα αλλά να είναι «φανάρι» στον χάρτη).

Εκτέλεση (στον σταθερό, όπου είναι οι εικόνες):
    ~/carla-env/bin/python pole_check.py --drive drive_2026-10-30_20-45-55
Έξοδος: pole_check/summary_<καταγραφή>.txt και pole_check/<κάμερα>_<καταγραφή>/<frame>.jpg
  Εικόνα ελέγχου: κάθετη γραμμή στο ΠΑΝΩ μισό = αντικείμενο της κάμερας (πράσινη ταιριάζει με τον χάρτη, κόκκινη όχι).
  Κάθετη γραμμή στο ΚΑΤΩ μισό = αντικείμενο του χάρτη (κίτρινη κολόνα, γαλάζια φανάρι, άσπρη πινακίδα).
"""

import argparse
import glob
import json
import os

import numpy as np

import building_corner_check as bc
from building_corner_check import WIDTH, HEIGHT, FOCAL, CX, CY, CAMERA_YAW, CELL, ROAD_Z

HERE = os.path.dirname(os.path.abspath(__file__))
OUT_DIR = os.path.join(HERE, "pole_check")
POLES_FILE = os.path.join(HERE, "town10", "town10_poles.json")

# είδος: (ετικέτα semantic, ελάχιστα px ύψος σε μια στήλη, έως πόσα m ψάχνουμε στον χάρτη)
# Τα px και τα m πάνε μαζί (εστιακή 640 px): κολόνα 5 m = 30 px στα ~105 m· φανάρι 1 m = 10 px στα ~65 m· πινακίδα 0,6 m = 8 px στα ~50 m.
KINDS = {"pole": (6, 30, 100.0), "light": (7, 10, 65.0), "sign": (8, 8, 50.0)}
GREEK = {"pole": "κολόνες", "light": "φανάρια", "sign": "πινακίδες", "any": "όλα μαζί (όποιο είδος)"}
MIN_WIDTH = 2           # px: πιο στενές ομάδες στηλών δεν μετράνε
MATCH_PX = 15           # px (~1,3°)
BORDER_PX = 10          # px: αντικείμενα τόσο κοντά στο πλάι της εικόνας δεν μετράνε
BINS = [0, 20, 40, 70, 100]     # m: για το «πόσα του χάρτη βρήκε η κάμερα, ανά απόσταση»


def camera_objects(tags, tag, min_pixels):
    """Μεσαίες στήλες (u) των αντικειμένων μιας ετικέτας. Στήλη μετράει αν έχει ≥ min_pixels px της ετικέτας."""
    has = np.concatenate([[False], (tags == tag).sum(axis=0) >= min_pixels, [False]])
    change = np.flatnonzero(np.diff(has.astype(int)))
    start, end = change[0::2], change[1::2]
    keep = (end - start) >= MIN_WIDTH
    centre = (start[keep] + end[keep] - 1) / 2
    return centre[(centre >= BORDER_PX) & (centre < WIDTH - BORDER_PX)]


def map_objects(xy, max_range, cam_x, cam_y, cam_heading, cam_height, grid):
    """Στήλη u και απόσταση (m) για κάθε αντικείμενο του χάρτη που η κάμερα θα έπρεπε να βλέπει.
    Η γωνία δεξιά της κάμερας είναι atan2(dy, dx) − κατεύθυνση (ίδια σύμβαση με το building_corner_check.map_profile)."""
    dx, dy = xy[:, 0] - cam_x, xy[:, 1] - cam_y
    forward = dx * np.cos(cam_heading) + dy * np.sin(cam_heading)
    right = -dx * np.sin(cam_heading) + dy * np.cos(cam_heading)
    distance = np.hypot(dx, dy)
    with np.errstate(divide="ignore", invalid="ignore"):
        u = CX + FOCAL * right / forward
    ok = (forward > 1.0) & (distance <= max_range) & (u >= BORDER_PX) & (u < WIDTH - BORDER_PX)
    cells = grid["cells"]
    for k in np.flatnonzero(ok):            # κρυμμένο πίσω από κτήριο; (ακτίνα ως 1,5 m πριν το αντικείμενο)
        steps = np.arange(1.0, distance[k] - 1.5, 0.25)
        ix = ((cam_x + dx[k] / distance[k] * steps - grid["x0"]) / CELL).astype(int)
        iy = ((cam_y + dy[k] / distance[k] * steps - grid["y0"]) / CELL).astype(int)
        inside = (ix >= 0) & (ix < cells.shape[0]) & (iy >= 0) & (iy < cells.shape[1])
        if np.any(cells[ix[inside], iy[inside]] >= ROAD_Z + cam_height + 1.0):
            ok[k] = False
    return u[ok], distance[ok]


def match(cam_u, map_u):
    """Για κάθε αντικείμενο της κάμερας: (ταιριάζει;, δείκτης του κοντινότερου του χάρτη)."""
    if len(cam_u) == 0 or len(map_u) == 0:
        return np.zeros(len(cam_u), dtype=bool), np.zeros(len(cam_u), dtype=int)
    gap = np.abs(cam_u[:, None] - map_u[None, :])
    return gap.min(axis=1) <= MATCH_PX, gap.argmin(axis=1)


def debug_image(tags, cam_u, cam_ok, map_by_kind, path):
    import cv2
    img = np.full((HEIGHT, WIDTH, 3), 90, dtype=np.uint8)
    for tag, color in bc.PALETTE.items():
        img[tags == tag] = color
    for tag, color in ((6, (0, 140, 255)), (7, (0, 0, 160)), (8, (200, 0, 200))):      # κολόνα, φανάρι, πινακίδα
        img[tags == tag] = color
    for kind, color in (("pole", (0, 255, 255)), ("light", (255, 255, 0)), ("sign", (255, 255, 255))):
        for u in map_by_kind[kind]:
            cv2.line(img, (int(u), int(CY) + 40), (int(u), HEIGHT - 1), color, 2)
    for u, ok in zip(cam_u, cam_ok):
        cv2.line(img, (int(u), 0), (int(u), int(CY) - 100), (0, 200, 0) if ok else (0, 0, 255), 2)
    cv2.imwrite(path, cv2.resize(img, (WIDTH // 2, HEIGHT // 2), interpolation=cv2.INTER_AREA),
                [cv2.IMWRITE_JPEG_QUALITY, 80])


def compare(tags, objects, cam_x, cam_y, cam_heading, cam_height, grid):
    """Μία εικόνα. Επιστρέφει ανά είδος (και 'any'): cam_u, cam_ok, λάθη γωνίας (°), map_u, map_distance, map_found."""
    cam = {kind: camera_objects(tags, tag, px) for kind, (tag, px, _) in KINDS.items()}
    on_map = {kind: map_objects(objects[kind], rng, cam_x, cam_y, cam_heading, cam_height, grid)
              for kind, (_, _, rng) in KINDS.items()}
    cam["any"] = np.concatenate(list(cam.values()))
    on_map["any"] = (np.concatenate([on_map[k][0] for k in KINDS]), np.concatenate([on_map[k][1] for k in KINDS]))
    result = {}
    for kind in list(KINDS) + ["any"]:
        map_u, map_distance = on_map[kind]
        ok, nearest = match(cam[kind], map_u)
        errors = [np.degrees(np.arctan((u - CX) / FOCAL) - np.arctan((map_u[j] - CX) / FOCAL))
                  for u, good, j in zip(cam[kind], ok, nearest) if good]
        found = np.array([np.any(np.abs(cam[kind] - u) <= MATCH_PX) for u in map_u], dtype=bool)
        result[kind] = dict(cam_u=cam[kind], cam_ok=ok, errors=errors, map_u=map_u, map_distance=map_distance, map_found=found)
    return result


def run_camera(drive, camera, n, n_debug, poses, objects, grid, mounts, cam_height, say):
    folder = bc.image_folder(drive, camera)
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
    kinds = list(KINDS) + ["any"]
    total = {k: dict(cam=0, cam_ok=0, mirrored_ok=0, errors=[], distance=[], found=[], matched_per_image=[]) for k in kinds}
    for frame in chosen:
        x, y, yaw = next(poses[g] for g in (frame, frame - 1, frame + 1) if g in poses)
        cam_x = x + forward * np.cos(yaw) - right * np.sin(yaw)
        cam_y = y + forward * np.sin(yaw) + right * np.cos(yaw)
        tags = bc.read_tags(os.path.join(folder, f"{frame}.png"))
        result = compare(tags, objects, cam_x, cam_y, yaw + np.radians(CAMERA_YAW[camera]), cam_height, grid)
        for k in kinds:
            r, t = result[k], total[k]
            t["cam"] += len(r["cam_u"])
            t["cam_ok"] += int(r["cam_ok"].sum())
            t["mirrored_ok"] += int(match(r["cam_u"], WIDTH - 1 - r["map_u"])[0].sum())
            t["errors"].extend(r["errors"])
            t["distance"].extend(r["map_distance"])
            t["found"].extend(r["map_found"])
            t["matched_per_image"].append(int(r["cam_ok"].sum()))
        if frame in debug_frames:
            try:
                debug_image(tags, result["any"]["cam_u"], result["any"]["cam_ok"],
                            {k: result[k]["map_u"] for k in KINDS}, os.path.join(out_dir, f"{frame}.jpg"))
            except Exception as error:
                print(f"  (η εικόνα ελέγχου {frame} δεν γράφτηκε: {error})")

    say(f"\n[{camera}] {len(chosen)} εικόνες ({drive})")
    for k in kinds:
        t = total[k]
        distance, found = np.array(t["distance"]), np.array(t["found"], dtype=bool)
        say(f"  {GREEK[k]}:")
        if t["cam"]:
            say(f"    αντικείμενα της κάμερας που ταιριάζουν με τον χάρτη (≤ {MATCH_PX} px): "
                f"{t['cam_ok']} από {t['cam']} ({100 * t['cam_ok'] / t['cam']:.0f}%)")
            if t["mirrored_ok"] > t["cam_ok"]:
                say(f"    ⚠️ με τον χάρτη ΑΝΑΠΟΔΑ (δεξιά-αριστερά) ταιριάζουν περισσότερα ({t['mirrored_ok']}): λάθος στη γεωμετρία;")
        else:
            say("    η κάμερα δεν είδε κανένα")
        if len(found):
            per_bin = ", ".join(f"{a}–{b} m: {100 * found[(distance >= a) & (distance < b)].mean():.0f}% από {np.sum((distance >= a) & (distance < b))}"
                                for a, b in zip(BINS[:-1], BINS[1:]) if np.any((distance >= a) & (distance < b)))
            say(f"    αντικείμενα του χάρτη που βρήκε η κάμερα: {found.sum()} από {len(found)} ({100 * found.mean():.0f}%) · {per_bin}")
        else:
            say("    ο χάρτης δεν έχει κανένα σε εμβέλεια")
        if t["errors"]:
            e = np.array(t["errors"])
            say(f"    λάθος γωνίας στα ταιριασμένα: μέσο {e.mean():+.2f}°, διάμεσο |λάθος| {np.median(np.abs(e)):.2f}°")
        m = np.array(t["matched_per_image"])
        say(f"    ταιριασμένα ανά εικόνα: {m.mean():.1f} · εικόνες με κανένα {100 * np.mean(m == 0):.0f}%, με 2+ {100 * np.mean(m >= 2):.0f}%")
    if debug_frames:
        say(f"  εικόνες ελέγχου: {out_dir}")


def load_objects():
    """είδος → πίνακας N×2 με x, y (το κέντρο του κουτιού κάθε αντικειμένου)."""
    data = json.load(open(POLES_FILE))["objects"]
    return {kind: np.array([[o["x"], o["y"]] for o in data if o["kind"] == kind], dtype=float).reshape(-1, 2) for kind in KINDS}


def main():
    parser = argparse.ArgumentParser(description="Κολόνες, φανάρια, πινακίδες: semantic κάμερα απέναντι στον χάρτη.")
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

    poses = bc.load_poses(args.drive)
    objects = load_objects()
    grid = bc.grid_from_lidar()
    mounts, cam_height = bc.camera_mounts()
    say(f"Ρυθμίσεις: MATCH {MATCH_PX} px · " + " · ".join(
        f"{GREEK[k]}: {len(objects[k])} στον χάρτη, ≥ {px} px, έως {rng:.0f} m" for k, (_, px, rng) in KINDS.items()))
    cameras = ["front", "left", "right", "rear"] if args.camera == "all" else [args.camera]
    for camera in cameras:
        run_camera(args.drive, camera, args.n, args.debug, poses, objects, grid, mounts, cam_height, say)
    with open(os.path.join(OUT_DIR, f"summary_{args.drive}.txt"), "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")
    print(f"\nΓράφτηκε: {os.path.join(OUT_DIR, f'summary_{args.drive}.txt')}")


if __name__ == "__main__":
    main()
