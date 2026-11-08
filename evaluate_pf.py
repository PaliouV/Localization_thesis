"""
Έλεγχος του φίλτρου σε καταγραφές που ΔΕΝ χρησιμοποιήθηκαν στη ρύθμιση (το tune_pf.py ρυθμίζει στις 4 καταγραφές
με φανάρια της 21 Οκτ· εδώ ο έλεγχος γίνεται στις 30 Οκτ, με κάμερα).

Για κάθε καταγραφή και κάθε δέκτη, τρέχει το particle_filter.py με 20 «ζάρια» (seeds), δύο φορές:
  - «τωρινό»:     οι τωρινές τιμές του particle_filter.py (ό,τι τρέχει χωρίς επιπλέον επιλογές)
  - «παλιό»:      τα αρχικά νούμερα δοκιμής (πριν τη ρύθμιση), για σύγκριση
και τυπώνει το μέσο λάθος (μέσος όρος ± τυπική απόκλιση στα seeds), μαζί με το λάθος του GPS μόνο.

    python3 evaluate_pf.py
"""

import re
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor

import numpy as np

CSVS = ["drive_2026-10-30_20-45-55.csv", "drive_2026-10-30_21-24-29.csv"]
SEEDS = range(20)

# Τα αρχικά νούμερα δοκιμής (πριν το tune_pf.py), για σύγκριση.
OLD = {"f9p": ["--gps-sigma", "3", "--velocity-sigma", "0.5", "--turn-rate-sigma", "0.02"],
       "m8t": ["--gps-sigma", "50", "--velocity-sigma", "0.5", "--turn-rate-sigma", "0.02"]}


def run(csv, receiver, extra, seed):
    """Τρέχει το φίλτρο μία φορά και επιστρέφει (μέσο λάθος φίλτρου, μέσο λάθος GPS)."""
    out = subprocess.run([sys.executable, "particle_filter.py", "--receiver", receiver, "--csv", csv,
                          "--seed", str(seed)] + extra,
                         capture_output=True, text=True, check=True).stdout
    filter_mean = float(re.search(r"φίλτρο: μέσο ([\d.]+)", out).group(1))
    gps_mean = float(re.search(r"gps: μέσο ([\d.]+)", out).group(1))
    return filter_mean, gps_mean


for csv in CSVS:
    print(f"\n{csv}")
    for receiver in ["f9p", "m8t"]:
        for name, extra in [("τωρινό", []), ("παλιό", OLD[receiver])]:
            with ThreadPoolExecutor(max_workers=8) as pool:
                results = np.array(list(pool.map(lambda seed: run(csv, receiver, extra, seed), SEEDS)))
            filter_err, gps_err = results[:, 0], results[:, 1]
            print(f"  {receiver} {name:<7} φίλτρο {filter_err.mean():6.2f} ± {filter_err.std():5.2f} m"
                  f"   | GPS μόνο {gps_err.mean():6.2f} m", flush=True)
