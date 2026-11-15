"""
Έλεγχος του φίλτρου με λωρίδες στις 4 καταγραφές με κάμερα, με τον πραγματικό ανιχνευτή (drives/*_lanes.csv).

    python3 run_lane_check.py

Τρέχει το particle_filter.py 160 φορές: 4 καταγραφές × 2 δέκτες × 10 seeds × 2 παραλλαγές
  - «χωρίς»: μόνο απόσταση από τις γραμμές (--no-lane-type --no-lane-angle)
  - «με»:    απόσταση + τύπος γραμμής + γωνία λωρίδας (οι προεπιλογές του particle_filter.py)
Γράφει κάθε τρέξιμο στο lane_check_results.jsonl και στο τέλος τυπώνει έναν πίνακα.
«Χαμένο» τρέξιμο = μέσο λάθος πάνω από 10 m.
"""
import json
import os
import re
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "lane_check_results.jsonl")
DRIVES = ["drive_2026-10-30_20-45-55.csv", "drive_2026-10-30_21-24-29.csv",
          "drive_2026-11-09_15-03-51.csv", "drive_2026-11-09_15-38-05.csv"]
VARIANTS = {"χωρίς": ["--no-lane-type", "--no-lane-angle"], "με": []}
jobs = [(v, r, c, s) for c in DRIVES for r in ["f9p", "m8t"] for v in VARIANTS for s in range(10)]


def run(job):
    variant, receiver, csv, seed = job
    out = subprocess.run([sys.executable, os.path.join(HERE, "particle_filter.py"), "--receiver", receiver, "--csv", csv,
                          "--seed", str(seed), "--lanes", "--lane-source", "detector"] + VARIANTS[variant],
                         capture_output=True, text=True, cwd=HERE)
    error = re.search(r"φίλτρο: μέσο ([\d.]+) m", out.stdout)
    lane = re.search(r"σωστή λωρίδα: ([\d.]+) %", out.stdout)
    result = dict(variant=variant, receiver=receiver, csv=csv, seed=seed)
    if error and lane:
        result.update(mean=float(error.group(1)), lane_ok=float(lane.group(1)))
    else:
        result.update(error=out.stderr[-300:])
    return result


results = []
with ThreadPoolExecutor(max(1, (os.cpu_count() or 2) - 1)) as pool, open(OUT, "w") as f:
    for n, result in enumerate(pool.map(run, jobs), 1):
        f.write(json.dumps(result, ensure_ascii=False) + "\n")
        f.flush()
        results.append(result)
        print(f"\r{n}/{len(jobs)}", end="", flush=True)

failed = [r for r in results if "error" in r]
print(f"\nαποτυχημένα τρεξίματα: {len(failed)}" + (f"  (πρώτο: {failed[0]['error']})" if failed else ""))
print("δέκτης  παραλλαγή  διάμεσο λάθος  μέσο λάθος  χειρότερο  χαμένα  σωστή λωρίδα (μέση)")
for receiver in ["f9p", "m8t"]:
    for variant in VARIANTS:
        rows = [r for r in results if r["receiver"] == receiver and r["variant"] == variant and "mean" in r]
        if not rows:
            continue
        mean = np.array([r["mean"] for r in rows])
        print(f"{receiver:6s}  {variant:9s}  {np.median(mean):10.2f} m  {mean.mean():8.2f} m  {mean.max():7.1f} m"
              f"  {np.sum(mean > 10):3d}/{len(rows)}  {np.mean([r['lane_ok'] for r in rows]):6.1f} %")
