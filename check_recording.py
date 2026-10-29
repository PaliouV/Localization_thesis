"""
Γρήγορος έλεγχος μιας καταγραφής του BMW_spawn.py (με ή χωρίς --cameras), στον ίδιο υπολογιστή.

Ελέγχει:
  - το CSV: διάρκεια προσομοίωσης, βήμα χρόνου, στάσεις
  - πόσο αργή ήταν η καταγραφή: πραγματικός χρόνος (από το όνομα του CSV ως την τελευταία εγγραφή του)
  - τις εικόνες, αν υπάρχουν: πλήθος ανά κάμερα, κάθε πόσα frames, αν ταιριάζουν με τα frames του CSV,
    χώρος, και ότι μία κανονική και μία «αλήθειας» ανοίγουν σωστά

    ~/carla-env/bin/python check_recording.py                       (η πιο πρόσφατη καταγραφή)
    ~/carla-env/bin/python check_recording.py drive_2026-10-30_10-00-00.csv
"""

import glob
import os
import sys
import time

import numpy as np

csv_path = sys.argv[1] if len(sys.argv) > 1 else sorted(glob.glob("drive_*.csv"))[-1]
print("Καταγραφή:", csv_path)

# ─── CSV ────────────────────────────────────────────────────────────────────
data = np.genfromtxt(csv_path, delimiter=",", names=True, invalid_raise=False)
sim_s = data["sim_time"][-1] - data["sim_time"][0]
steps = np.round(np.diff(data["sim_time"]), 3)
print(f"\nCSV: {len(data)} γραμμές, {sim_s / 60:.1f} λεπτά προσομοίωσης, "
      f"βήμα {np.median(steps)} s (διαφορετικά βήματα: {np.sum(steps != np.median(steps))})")
print(f"     σταματημένο {np.mean(data['vel'] < 0.5) * 100:.0f}% του χρόνου")
has_frame = "frame" in data.dtype.names
if not has_frame:
    print("     ΠΡΟΣΟΧΗ: δεν έχει στήλη frame (παλιά μορφή) — οι εικόνες δεν μπορούν να αντιστοιχιστούν")

# Πραγματικός χρόνος: η ώρα έναρξης είναι στο όνομα του CSV, το τέλος είναι η τελευταία εγγραφή του.
start = time.mktime(time.strptime(os.path.basename(csv_path)[len("drive_"):-len(".csv")], "%Y-%m-%d_%H-%M-%S"))
real_s = os.path.getmtime(csv_path) - start
ratio = sim_s / real_s if real_s > 0 else float("nan")
print(f"\nΤαχύτητα: {real_s / 60:.1f} λεπτά πραγματικού χρόνου → {ratio:.2f} λεπτά προσομοίωσης ανά λεπτό")
print(f"          άρα 30 λεπτά προσομοίωσης θέλουν ~{30 / ratio:.0f} λεπτά πραγματικού χρόνου")

# ─── Εικόνες ────────────────────────────────────────────────────────────────
img_dir = csv_path[:-len(".csv")]
folders = sorted(glob.glob(os.path.join(img_dir, "*")))
if not folders:
    print("\nΕικόνες: καμία (καταγραφή χωρίς --cameras)")
    sys.exit()

csv_frames = set(data["frame"].astype(int)) if has_frame else set()
total_mb = 0.0
print(f"\nΕικόνες στο {img_dir}/:")
for folder in folders:
    files = sorted(glob.glob(os.path.join(folder, "*.*")))
    frames = np.array(sorted(int(os.path.splitext(os.path.basename(f))[0]) for f in files))
    mb = sum(os.path.getsize(f) for f in files) / 1e6
    total_mb += mb
    gaps = np.diff(frames)
    gap_text = ", ".join(f"{g}:{n}" for g, n in zip(*np.unique(gaps, return_counts=True))) if len(gaps) else "—"
    missing = sum(f not in csv_frames for f in frames) if has_frame else "?"
    print(f"  {os.path.basename(folder):<15} {len(files):6d} εικόνες ({len(files) / sim_s:.1f}/s), "
          f"{mb:8.1f} MB ({mb / max(len(files), 1):.2f} MB/εικόνα) | κενά σε frames {gap_text} | "
          f"χωρίς γραμμή στο CSV: {missing}")
print(f"  ΣΥΝΟΛΟ {total_mb / 1000:.1f} GB → για 30 λεπτά προσομοίωσης ~{total_mb / 1000 * 1800 / sim_s:.0f} GB")

# Ανοίγουν οι εικόνες; (μία κανονική, μία «αλήθειας»)
try:
    import cv2
    for kind, ext in (("rgb", "jpg"), ("semantic", "png")):
        sample = sorted(glob.glob(os.path.join(img_dir, f"front_{kind}", f"*.{ext}")))
        if sample:
            im = cv2.imread(sample[len(sample) // 2], cv2.IMREAD_UNCHANGED)
            extra = f", κατηγορίες: {sorted(np.unique(im).tolist())}" if kind == "semantic" else ""
            print(f"  δείγμα front_{kind}: σχήμα {im.shape}{extra}")
except ImportError:
    print("  (χωρίς cv2: δεν ελέγχθηκε αν ανοίγουν οι εικόνες)")
