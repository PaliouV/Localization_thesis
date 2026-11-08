"""
Ρύθμιση του gps_sigma του φίλτρου (particle_filter.py), όπως έγινε στις 7 Νοε 2026.

Τι ρυθμίζεται: ΜΟΝΟ το gps_sigma (πόσο εμπιστεύεται το φίλτρο το GPS), για κάθε δέκτη.
  Τα σ ταχύτητας και γυροσκοπίου ΔΕΝ ρυθμίζονται: βγαίνουν από τους αισθητήρες (particle_filter.py).
Πού: στις 4 καταγραφές με φανάρια της 21 Οκτ (χωρίς κάμερα). Ο έλεγχος γίνεται μετά στις 30 Οκτ (με κάμερα),
  με το evaluate_pf.py, για να μη «μάθουμε απέξω» τις διαδρομές της ρύθμισης.
  Οι καταγραφές χωρίς στάσεις δεν χρησιμοποιούνται (δεν είναι ρεαλιστικές για πόλη).
Πώς: για κάθε τιμή τρέχει το φίλτρο με 20 «ζάρια» (seeds) σε κάθε καταγραφή. Με το ίδιο seed ο «κόσμος»
  (GPS, γυροσκόπιο, ταχύτητα) είναι ίδιος για όλες τις τιμές, οπότε οι τιμές συγκρίνονται δίκαια.
  Τυπώνει το μέσο λάθος για κάθε τιμή, ανά καταγραφή και συνολικά, και πόσο συχνά κερδίζει την τωρινή τιμή.

    python3 tune_pf.py                    (και οι δύο δέκτες, ~15–20 λεπτά)
    python3 tune_pf.py --receiver m8t     (μόνο ο ένας)

Ιστορικό: στις 2 Νοε η ρύθμιση έγινε στην 21 Οκτ 13:54 (χωρίς στάσεις) και ρύθμιζε και τα σ ταχύτητας/γυροσκοπίου
  (βλ. git log). Στις 7 Νοε (φίλτρο με systematic resample και bias από τις στάσεις) βγήκε: F9P 12 (τα 8–12
  σχεδόν ίσα), M8T 200 (το 100 σχεδόν ίσο). Ο έλεγχος στις 30 Οκτ έκρινε τις ισοπαλίες.
"""

import argparse
import re
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor

import numpy as np

parser = argparse.ArgumentParser()
parser.add_argument("--receiver", choices=["f9p", "m8t"], help="μόνο αυτός ο δέκτης (αλλιώς και οι δύο)")
args = parser.parse_args()

CSVS = ["drive_2026-10-21_20-33-16.csv", "drive_2026-10-21_20-46-36.csv",
        "drive_2026-10-21_21-01-27.csv", "drive_2026-10-21_21-13-32.csv"]
SEEDS = range(20)
GPS_SIGMAS = {"f9p": [4, 6, 8, 10, 12, 15],                 # m
              "m8t": [15, 30, 50, 100, 200, 1000]}          # m (1000 ≈ σχεδόν καθόλου GPS)
CURRENT = {"f9p": 12, "m8t": 200}                           # οι τιμές που έχει τώρα το particle_filter.py


def run(receiver, csv, gps_sigma, seed):
    """Τρέχει το φίλτρο μία φορά και επιστρέφει (μέσο λάθος φίλτρου, μέσο λάθος GPS)."""
    out = subprocess.run([sys.executable, "particle_filter.py", "--receiver", receiver, "--csv", csv,
                          "--seed", str(seed), "--gps-sigma", str(gps_sigma)],
                         capture_output=True, text=True, check=True).stdout
    filter_mean = float(re.search(r"φίλτρο: μέσο ([\d.]+)", out).group(1))
    gps_mean = float(re.search(r"gps: μέσο ([\d.]+)", out).group(1))
    return filter_mean, gps_mean


receivers = [args.receiver] if args.receiver else ["f9p", "m8t"]
for receiver in receivers:
    sigmas = GPS_SIGMAS[receiver]
    jobs = [(receiver, csv, g, seed) for g in sigmas for csv in CSVS for seed in SEEDS]
    print(f"\n{receiver}: {len(sigmas)} τιμές × {len(CSVS)} καταγραφές × {len(SEEDS)} seeds = {len(jobs)} τρεξίματα ...",
          flush=True)

    with ThreadPoolExecutor(max_workers=8) as pool:          # 8 τρεξίματα ταυτόχρονα
        results = list(pool.map(lambda job: run(*job), jobs))

    # err[τιμή, καταγραφή, seed]
    err = np.array([r[0] for r in results]).reshape(len(sigmas), len(CSVS), len(SEEDS))
    gps = np.array([r[1] for r in results]).reshape(len(sigmas), len(CSVS), len(SEEDS))[0]   # ίδιο για κάθε τιμή

    print("  καταγραφές:   " + "   ".join(csv[17:22] for csv in CSVS) + "   | όλες | κερδίζει την τωρινή")
    print("  GPS μόνο:     " + "   ".join(f"{gps[c].mean():5.1f}" for c in range(len(CSVS))) + f"   | {gps.mean():5.2f} |")
    current = sigmas.index(CURRENT[receiver])
    for k, g in enumerate(sigmas):
        wins = np.sum(err[k] < err[current])                 # σε πόσα (καταγραφή, seed) βγήκε καλύτερη από την τωρινή
        line = "   ".join(f"{err[k, c].mean():5.1f}" for c in range(len(CSVS)))
        mark = "  ← τωρινή" if k == current else f"  {wins}/{err[k].size}"
        print(f"  σ = {g:>5}:    {line}   | {err[k].mean():5.2f} |{mark}")
    best = int(np.argmin(err.mean(axis=(1, 2))))
    print(f"  Καλύτερη (μικρότερο μέσο λάθος σε όλες): σ = {sigmas[best]}")
