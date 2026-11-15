"""
Ρύθμιση και έλεγχος της πυξίδας (15 Νοε 2026): πόσο πρέπει να την εμπιστεύεται το φίλτρο (compass_sigma).

Τρέχει το particle_filter.py πολλές φορές (καταγραφές × δέκτες × τιμές compass_sigma × seeds), χωρίς πυξίδα και
με πυξίδα, και τυπώνει πίνακα. Κάθε τρέξιμο γράφεται και σε αρχείο .jsonl, για να μη χαθεί.

    python3 tune_compass.py tune              # ρύθμιση: οι 4 καταγραφές της 21 Οκτ (όπως το tune_pf.py)
    python3 tune_compass.py test              # έλεγχος: 30 Οκτ και 14 Νοε, που δεν τις είδε η ρύθμιση
    python3 tune_compass.py test --lanes      # το ίδιο με κάμερα λωρίδων
    python3 tune_compass.py show ΑΡΧΕΙΟ.jsonl # ξανατυπώνει τον πίνακα από αρχείο αποτελεσμάτων

Επιλογές: --sigmas 20,30 (τιμές compass_sigma σε μοίρες) · --seeds 6 · --workers 8 (πόσα τρεξίματα ταυτόχρονα).
"""

import argparse
import json
import re
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor

import numpy as np

CSVS = {"tune": ["drive_2026-10-21_20-33-16.csv", "drive_2026-10-21_20-46-36.csv",
                 "drive_2026-10-21_21-01-27.csv", "drive_2026-10-21_21-13-32.csv"],
        "test": ["drive_2026-10-30_20-45-55.csv", "drive_2026-10-30_21-24-29.csv", "drive_2026-11-14_21-44-59.csv",
                 "drive_2026-11-14_22-20-56.csv", "drive_2026-11-14_22-56-47.csv"]}


def run(job):
    """Τρέχει το φίλτρο μία φορά και επιστρέφει τα νούμερα που τύπωσε."""
    receiver, csv, sigma, start, seed, lanes = job
    cmd = [sys.executable, "particle_filter.py", "--receiver", receiver, "--csv", csv, "--seed", str(seed)]
    if lanes:
        cmd += ["--lanes"]
    if sigma is not None:
        cmd += ["--compass", "--compass-sigma", str(sigma)] + (["--compass-start"] if start else [])
    result = subprocess.run(cmd, capture_output=True, text=True)
    row = dict(receiver=receiver, csv=csv, sigma=sigma, start=start, seed=seed, lanes=lanes)
    if result.returncode != 0:
        return dict(row, error=result.stderr[-300:])
    out = result.stdout
    return dict(row,
                pos=float(re.search(r"φίλτρο: μέσο ([\d.]+)", out).group(1)),
                pos_max=float(re.search(r"φίλτρο: μέσο [\d.]+ m, μέγιστο ([\d.]+)", out).group(1)),
                head_med=float(re.search(r"κατεύθυνση: διάμεσο λάθος ([\d.]+)", out).group(1)),
                head_95=float(re.search(r"κάτω από ([\d.]+)", out).group(1)),
                lane=float(re.search(r"σωστή λωρίδα: ([\d.]+)", out).group(1)))


def show(rows):
    """Πίνακας: μία γραμμή ανά ρύθμιση, μέσος όρος σε καταγραφές και seeds."""
    bad = [r for r in rows if "error" in r]
    print(f"{len(rows)} τρεξίματα, {len(bad)} με σφάλμα")
    if bad:
        print("  πρώτο σφάλμα:", bad[0]["error"])
    rows = [r for r in rows if "error" not in r]
    for receiver in ("f9p", "m8t"):
        mine = [r for r in rows if r["receiver"] == receiver]
        if not mine:
            continue
        base = {(r["csv"], r["seed"]): r["pos"] for r in mine if r["sigma"] is None}
        print(f"\n{receiver}:  σ  αρχή |  θέση m  (±)   μέγιστο m | κατεύθυνση: διάμεσο  95% | σωστή λωρίδα % | καλύτερο από «χωρίς»")
        settings = sorted({(r["sigma"], r["start"]) for r in mine}, key=lambda k: (k[0] is not None, k[0] or 0, k[1]))
        for sigma, start in settings:
            rr = [r for r in mine if r["sigma"] == sigma and r["start"] == start]
            pos = np.array([r["pos"] for r in rr])
            wins = "" if sigma is None else f"{100 * np.mean([r['pos'] < base[(r['csv'], r['seed'])] for r in rr]):3.0f}%"
            name = "χωρίς" if sigma is None else f"{sigma:5.0f}"
            print(f"   {name}  {'ναι' if start else 'όχι'} | {pos.mean():6.2f} ±{pos.std():5.2f}   {np.mean([r['pos_max'] for r in rr]):7.1f} |"
                  f"   {np.mean([r['head_med'] for r in rr]):8.2f}  {np.mean([r['head_95'] for r in rr]):6.2f} |"
                  f"   {np.mean([r['lane'] for r in rr]):6.1f}       | {wins}")


parser = argparse.ArgumentParser(description="Ρύθμιση και έλεγχος της πυξίδας.")
parser.add_argument("which", choices=["tune", "test", "show"])
parser.add_argument("file", nargs="?", help="μόνο για το show: αρχείο αποτελεσμάτων")
parser.add_argument("--lanes", action="store_true", help="με κάμερα λωρίδων")
parser.add_argument("--sigmas", default=None, help="τιμές compass_sigma (μοίρες), με κόμμα")
parser.add_argument("--seeds", type=int, default=6)
parser.add_argument("--workers", type=int, default=8)
args = parser.parse_args()

if args.which == "show":
    show([json.loads(line) for line in open(args.file, encoding="utf-8")])
    sys.exit()

default_sigmas = {"tune": "5,10,15,20,30,45,60,90", "test": "20,30"}[args.which]
sigmas = [float(s) for s in (args.sigmas or default_sigmas).split(",")]
settings = [(None, False)] + [(s, start) for s in sigmas for start in (False, True)]     # πρώτα «χωρίς πυξίδα»
jobs = [(receiver, csv, sigma, start, seed, args.lanes)
        for receiver in ("f9p", "m8t") for sigma, start in settings for csv in CSVS[args.which] for seed in range(args.seeds)]
out_file = f"compass_results_{args.which}_{'lanes' if args.lanes else 'nolanes'}.jsonl"
print(f"{len(jobs)} τρεξίματα, {args.workers} ταυτόχρονα → {out_file}", flush=True)

rows = []
with ThreadPoolExecutor(max_workers=args.workers) as pool, open(out_file, "w", encoding="utf-8") as f:
    for k, row in enumerate(pool.map(run, jobs)):
        rows.append(row)
        f.write(json.dumps(row, ensure_ascii=False) + "\n")
        f.flush()
        if (k + 1) % 50 == 0:
            print(f"  {k + 1} / {len(jobs)}", flush=True)
show(rows)
