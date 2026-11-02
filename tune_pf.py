"""
Ρύθμιση των σ του φίλτρου (particle_filter.py): δοκιμάζει πολλούς συνδυασμούς και κρατάει τους καλύτερους.

Για κάθε δέκτη, για κάθε συνδυασμό (gps_sigma, velocity_sigma, turn_rate_sigma), τρέχει το φίλτρο
με 10 διαφορετικά «ζάρια» (seeds) στην ίδια καταγραφή και παίρνει τον μέσο όρο του μέσου λάθους.
Ρυθμίζουμε σε ΜΙΑ καταγραφή· ο έλεγχος γίνεται μετά σε ΑΛΛΕΣ (για να μη «μάθουμε απέξω» αυτή).

    python3 tune_pf.py --round 1     (όλοι οι συνδυασμοί, 10 seeds, ~15 λεπτά)
    python3 tune_pf.py --round 2     (μόνο gps_sigma γύρω από τον νικητή του γύρου 1, 20 seeds, ~2 λεπτά)
"""

import argparse
import itertools
import re
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor

import numpy as np

parser = argparse.ArgumentParser()
parser.add_argument("--round", type=int, choices=[1, 2], default=1)
args = parser.parse_args()

CSV = "drive_2026-10-21_13-54-17.csv"

if args.round == 1:
    # Γύρος 1: όλοι οι συνδυασμοί.
    SEEDS = range(10)
    GPS_SIGMAS = {"f9p": [2, 3, 5, 8], "m8t": [10, 20, 30, 50, 80]}                 # m
    VELOCITY_SIGMAS = {"f9p": [0.05, 0.1, 0.2, 0.5], "m8t": [0.05, 0.1, 0.2, 0.5]}  # m/s (πραγματικός θόρυβος: 0.05)
    TURN_RATE_SIGMAS = {"f9p": [0.0025, 0.005, 0.01, 0.02],
                        "m8t": [0.0025, 0.005, 0.01, 0.02]}                         # rad/s (πραγματικός θόρυβος: 0.0023)
else:
    # Γύρος 2: οι νικητές του γύρου 1 (2 Νοε 2026) βγήκαν στην ΑΚΡΗ των τιμών του gps_sigma
    # (F9P 8, M8T 10), οπότε ψάχνουμε λίγο πιο πέρα. Τα άλλα δύο μένουν σταθερά στους νικητές.
    # Ο νικητής του γύρου 1 είναι μέσα, για σύγκριση στα ίδια ζάρια. Περισσότερα seeds, γιατί είναι λίγοι συνδυασμοί.
    SEEDS = range(20)
    GPS_SIGMAS = {"f9p": [8, 10, 12, 15, 20], "m8t": [3, 5, 7, 10, 15]}
    VELOCITY_SIGMAS = {"f9p": [0.05], "m8t": [0.1]}
    TURN_RATE_SIGMAS = {"f9p": [0.005], "m8t": [0.0025]}


def run(receiver, gps_sigma, velocity_sigma, turn_rate_sigma, seed):
    """Τρέχει το φίλτρο μία φορά και επιστρέφει (μέσο λάθος φίλτρου, μέσο λάθος GPS)."""
    out = subprocess.run([sys.executable, "particle_filter.py", "--receiver", receiver, "--csv", CSV,
                          "--seed", str(seed), "--gps-sigma", str(gps_sigma),
                          "--velocity-sigma", str(velocity_sigma), "--turn-rate-sigma", str(turn_rate_sigma)],
                         capture_output=True, text=True, check=True).stdout
    filter_mean = float(re.search(r"φίλτρο: μέσο ([\d.]+)", out).group(1))
    gps_mean = float(re.search(r"gps: μέσο ([\d.]+)", out).group(1))
    return filter_mean, gps_mean


for receiver in ["f9p", "m8t"]:
    combos = list(itertools.product(GPS_SIGMAS[receiver], VELOCITY_SIGMAS[receiver], TURN_RATE_SIGMAS[receiver]))
    jobs = [(receiver, *combo, seed) for combo in combos for seed in SEEDS]
    print(f"\n{receiver}: {len(combos)} συνδυασμοί × {len(SEEDS)} seeds = {len(jobs)} τρεξίματα ...", flush=True)

    with ThreadPoolExecutor(max_workers=8) as pool:          # 8 τρεξίματα ταυτόχρονα
        results = list(pool.map(lambda job: run(*job), jobs))

    filter_err = np.array([r[0] for r in results]).reshape(len(combos), len(SEEDS))
    gps_err = np.array([r[1] for r in results]).reshape(len(combos), len(SEEDS))
    mean_err = filter_err.mean(axis=1)                        # μέσος όρος στα 10 seeds, για κάθε συνδυασμό

    print(f"GPS μόνο (μέσος όρος στα seeds): {gps_err.mean():.2f} m")
    print("Οι 5 καλύτεροι συνδυασμοί:")
    print("  gps_sigma  velocity_sigma  turn_rate_sigma   λάθος φίλτρου (μέσο ± τυπ. απόκλιση στα seeds)")
    for k in np.argsort(mean_err)[:5]:
        g, v, t = combos[k]
        print(f"  {g:>9}  {v:>14}  {t:>15}   {mean_err[k]:.2f} ± {filter_err[k].std():.2f} m")
    k = int(np.argmax(mean_err))
    g, v, t = combos[k]
    print(f"Ο χειρότερος: {g}, {v}, {t} → {mean_err[k]:.2f} m")
