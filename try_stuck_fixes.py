"""
Σύγκριση τριών τρόπων για το φίλτρο που μένει «σίγουρο αλλά λάθος» (16–19 Νοε 2026).

Το πρόβλημα (M8T + λωρίδες + πυξίδα): τα σωματίδια μαζεύονται σε ένα σημείο σε λάθος θέση και δεν υπάρχει κανένα
κοντά στη σωστή για να τα τραβήξει πίσω. Οι τρόποι που δοκιμάζονται:
  1. νέα σωματίδια κάθε δευτερόλεπτο γύρω από το GPS
  2. περισσότερο ανακάτεμα μετά από κάθε διόρθωση της κάμερας (σε όλες τις κατευθύνσεις, ή 2β μόνο κατά μήκος)
  3. λιγότερη εμπιστοσύνη στην κάμερα (μεγαλύτερο lane_sigma)

Δεν αλλάζει το particle_filter.py: το διαβάζει, του προσθέτει στη μνήμη τις αλλαγές του κάθε τρόπου και το τρέχει.
Κάθε τρέξιμο γίνεται σε δικό του πρόγραμμα Python. Αν το αρχείο αποτελεσμάτων υπάρχει, συνεχίζει από εκεί που σταμάτησε.

    python3 try_stuck_fixes.py --workers 18 --seeds 10     # τρέχει όλα και τυπώνει πίνακα
    python3 try_stuck_fixes.py --dry-run                   # μόνο πόσα τρεξίματα θα γίνουν
    python3 try_stuck_fixes.py show                        # ξανατυπώνει τον πίνακα από το αρχείο
"""

import argparse
import contextlib
import io
import json
import os
import re
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
FILTER = os.path.join(HERE, "particle_filter.py")
OUT_FILE = os.path.join(HERE, "stuck_fixes_results.jsonl")
CSVS = ["drive_2026-10-30_20-45-55.csv", "drive_2026-10-30_21-24-29.csv", "drive_2026-11-14_21-44-59.csv",
        "drive_2026-11-14_22-20-56.csv", "drive_2026-11-14_22-56-47.csv"]
# όνομα: (επιπλέον flags, ποσοστό νέων σωματιδίων, ακτίνα τους σε m, ανακάτεμα μόνο κατά μήκος σε m)
VARIANTS = {
    "όπως είναι τώρα":                   ([], 0, 0, 0),
    "1. νέα σωματίδια 2%, ακτίνα 20 m":   ([], 0.02, 20, 0),
    "1. νέα σωματίδια 5%, ακτίνα 20 m":   ([], 0.05, 20, 0),
    "1. νέα σωματίδια 5%, ακτίνα 50 m":   ([], 0.05, 50, 0),
    "2. ανακάτεμα 0,3 m":                ([ "--lane-jitter", "0.3"], 0, 0, 0),
    "2. ανακάτεμα 0,5 m":                (["--lane-jitter", "0.5"], 0, 0, 0),
    "2. ανακάτεμα 1,0 m":                (["--lane-jitter", "1.0"], 0, 0, 0),
    "2β. ανακάτεμα μόνο κατά μήκος 1 m":  ([], 0, 0, 1.0),
    "2β. ανακάτεμα μόνο κατά μήκος 2 m":  ([], 0, 0, 2.0),
    "3. κάμερα: lane_sigma 2":           (["--lane-sigma", "2"], 0, 0, 0),
    "3. κάμερα: lane_sigma 3":           (["--lane-sigma", "3"], 0, 0, 0),
}


def patched_filter():
    """Ο κώδικας του particle_filter.py με τρεις προσθήκες: μέτρηση απλώματος, νέα σωματίδια, ανακάτεμα κατά μήκος."""
    src = open(FILTER, encoding="utf-8").read()
    a = "    est_x, est_y, est_yaw = estimate(particles)\n"
    assert src.count(a) == 1, "άλλαξε το particle_filter.py: δεν βρέθηκε η γραμμή του estimate"
    src = src.replace(a, a + "    _c = np.cov(particles[:, :2].T)\n    SPREAD.append(np.sqrt(_c[0, 0] + _c[1, 1]))\n")
    a = "        weights = update(particles, fix[0], fix[1], gps_sigma)\n        particles, weights = resample(particles, weights)\n"
    assert src.count(a) == 1, "άλλαξε το particle_filter.py: δεν βρέθηκε η διόρθωση του GPS"
    src = src.replace(a, a + (
        "        if INJECT_FRAC > 0:\n"
        "            _k = int(INJECT_FRAC * N); _i = np.random.choice(N, _k, replace=False)\n"
        "            particles[_i, 0] = np.random.normal(fix[0], INJECT_R, _k)\n"
        "            particles[_i, 1] = np.random.normal(fix[1], INJECT_R, _k)\n"
        "            particles[_i, 2] = (np.random.normal(measured_yaw, compass_sigma, _k) if measured_yaw is not None\n"
        "                                else np.random.uniform(0, 2 * np.pi, _k))\n"))
    a = "            particles[:, 1] += np.random.normal(0, LANE_JITTER, N)\n"
    assert src.count(a) == 1, "άλλαξε το particle_filter.py: δεν βρέθηκε το ανακάτεμα της κάμερας"
    src = src.replace(a, a + (
        "            if ALONG_JIT > 0:\n"
        "                _d = np.random.normal(0, ALONG_JIT, N)\n"
        "                particles[:, 0] += _d * np.cos(particles[:, 2])\n"
        "                particles[:, 1] += _d * np.sin(particles[:, 2])\n"))
    return compile(src, FILTER, "exec")


def one_run(job):
    """Ένα τρέξιμο (μέσα σε δικό του πρόγραμμα). Επιστρέφει τα νούμερα."""
    receiver, name, csv, seed = job
    extra, frac, radius, along = VARIANTS[name]
    os.chdir(HERE)
    sys.path.insert(0, HERE)
    sys.argv = ["particle_filter.py", "--receiver", receiver, "--lanes", "--compass", "--lane-source", "detector",
                "--csv", csv, "--seed", str(seed)] + extra
    g = {"__name__": "__main__", "__file__": FILTER, "SPREAD": [], "INJECT_FRAC": frac, "INJECT_R": radius,
         "ALONG_JIT": along}
    buf = io.StringIO()
    with contextlib.redirect_stdout(buf):
        exec(patched_filter(), g)
    spread, err = np.array(g["SPREAD"]), g["filter_error"]
    wrong = (err > 3) & (spread < 1.5)                    # σίγουρο αλλά λάθος: λάθος > 3 m, νέφος < 1,5 m
    longest = cur = 0
    for w in wrong:
        cur = cur + 1 if w else 0
        longest = max(longest, cur)
    lane = float(re.search(r"σωστή λωρίδα: ([\d.]+)", buf.getvalue()).group(1))
    return dict(receiver=receiver, name=name, csv=csv, seed=seed, error=float(err.mean()), error_max=float(err.max()),
                lane=lane, wrong_pct=float(wrong.mean() * 100), longest_s=longest * 0.05,
                spread=float(np.median(spread)))


def launch(job):
    """Ξεκινά ένα τρέξιμο σε ξεχωριστό πρόγραμμα Python και διαβάζει το αποτέλεσμα."""
    result = subprocess.run([sys.executable, __file__, "--one", json.dumps(job, ensure_ascii=False)],
                            capture_output=True, text=True, cwd=HERE)
    if result.returncode != 0:
        return dict(receiver=job[0], name=job[1], csv=job[2], seed=job[3], failed=result.stderr[-400:])
    return json.loads(result.stdout.strip().splitlines()[-1])


def show(rows):
    bad = [r for r in rows if "failed" in r]
    print(f"{len(rows)} τρεξίματα, {len(bad)} με σφάλμα")
    if bad:
        print("  πρώτο σφάλμα:", bad[0]["failed"])
    rows = [r for r in rows if "failed" not in r]
    for receiver in ("m8t", "f9p"):
        mine = [r for r in rows if r["receiver"] == receiver]
        if not mine:
            continue
        n = len({(r["csv"], r["seed"]) for r in mine})
        print(f"\n{receiver} + λωρίδες (ανιχνευτής) + πυξίδα, {n} τρεξίματα ανά γραμμή")
        print("  τρόπος                              | μέσο λάθος | σωστή λωρίδα | σίγουρο αλλά λάθος | μεγαλύτερο επεισόδιο (μέσο / χειρότερο) | άπλωμα")
        for name in VARIANTS:
            rr = [r for r in mine if r["name"] == name]
            if not rr:
                continue
            m = lambda k: np.mean([r[k] for r in rr])
            print(f"  {name:<35} | {m('error'):6.2f} m   | {m('lane'):6.1f}%     | {m('wrong_pct'):6.1f}% του χρόνου |"
                  f" {m('longest_s'):5.0f} s / {max(r['longest_s'] for r in rr):5.0f} s | {m('spread'):.2f} m")


parser = argparse.ArgumentParser(description="Σύγκριση τρόπων για το «σίγουρο αλλά λάθος» φίλτρο.")
parser.add_argument("what", nargs="?", choices=["run", "show"], default="run")
parser.add_argument("--seeds", type=int, default=10)
parser.add_argument("--workers", type=int, default=8)
parser.add_argument("--dry-run", action="store_true")
parser.add_argument("--one", help=argparse.SUPPRESS)
args = parser.parse_args()

if args.one:
    print(json.dumps(one_run(json.loads(args.one)), ensure_ascii=False))
    sys.exit()
if args.what == "show":
    show([json.loads(line) for line in open(OUT_FILE, encoding="utf-8") if line.strip()])
    sys.exit()

jobs = [(r, n, c, s) for r in ("m8t", "f9p") for n in VARIANTS for c in CSVS for s in range(args.seeds)]
rows = []
if os.path.exists(OUT_FILE):
    rows = [r for r in (json.loads(line) for line in open(OUT_FILE, encoding="utf-8") if line.strip()) if "failed" not in r]
done = {(r["receiver"], r["name"], r["csv"], r["seed"]) for r in rows}
todo = [j for j in jobs if j not in done]
print(f"{len(rows)} τρεξίματα υπάρχουν ήδη, μένουν {len(todo)}, {args.workers} ταυτόχρονα → {os.path.basename(OUT_FILE)}",
      flush=True)
if args.dry_run:
    sys.exit()
with ThreadPoolExecutor(max_workers=args.workers) as pool, open(OUT_FILE, "a", encoding="utf-8") as f:
    for k, row in enumerate(pool.map(launch, todo)):
        rows.append(row)
        f.write(json.dumps(row, ensure_ascii=False) + "\n")
        f.flush()
        if (k + 1) % 50 == 0:
            print(f"  {k + 1} / {len(todo)}", flush=True)
show(rows)
