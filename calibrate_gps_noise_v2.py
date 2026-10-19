"""
Ρύθμιση του gps_noise_v2.py με τα πραγματικά δεδομένα του UrbanNav.

Ιδέα: το μοντέλο «οδηγεί» πάνω στις ΙΔΙΕΣ διαδρομές του UrbanNav (ίδια ταχύτητα, στάσεις, στροφές)
και μετράμε το σφάλμα του με τους ΙΔΙΟΥΣ ορισμούς που μετράμε το σφάλμα του πραγματικού δέκτη.
Οι παράμετροι αλλάζουν μέχρι τα δύο να μοιάζουν. Κάθε παράμετρος έχει «δικό της» στόχο:
  sigma      ↔ μέσο και διάμεσο 2D σφάλμα
  tau        ↔ πόσο απομακρύνεται το σφάλμα σε 10 s και 60 s (sqrt του variogram ανά άξονα)
  step_rate  ↔ πόσα απότομα άλματα (> 10 m σε 1 s) την ώρα          (μόνο M8T)
  step_size  ↔ μαζί με τα παραπάνω, το μέσο έναντι του διαμέσου      (μόνο M8T)
  lag        ↔ μέσο ΚΑΤΑ ΜΗΚΟΣ σφάλμα σε κίνηση (πόσο μένει πίσω)    (μόνο M8T)
Ο λόγος εγκάρσιο / κατά μήκος ΔΕΝ ρυθμίζεται: είναι ο μετρημένος (απόφαση 5β). Σε στάση δεν
μετριέται (δεν υπάρχει «μπρος»), οπότε μπαίνει 1. ΠΑΡΑΔΟΧΗ.

Ζώνες ← διαδρομές και κοινές τιμές όπως αποφασίστηκαν (16 Οκτ 2026):
  F9P: MILD + MODERATE κοινές (medium + deep), STRONG ← harsh
  M8T: στάση MILD ← medium, MODERATE ← deep χωριστά· κίνηση MILD + MODERATE κοινή· STRONG ← harsh
Καταστάσεις: σταματημένο < 0.5 m/s, κινείται > 2 m/s (ίδια όρια με την ανάλυση).

Βελτιστοποίηση: Nelder–Mead σε λογαριθμική κλίμακα, με σταθερά seeds (ίδιος «τυχαίος» θόρυβος σε
κάθε δοκιμή, ώστε η σύγκριση παραμέτρων να είναι δίκαιη). Πρώτα η κίνηση με τη στάση σταθερή, μετά η
στάση με την κίνηση σταθερή, δύο γύροι.

    python calibrate_gps_noise_v2.py f9p
    python calibrate_gps_noise_v2.py m8t
Τυπώνει τις τιμές για το RECEIVERS του gps_noise_v2.py και πίνακα σύγκρισης μοντέλου / δεδομένων.
"""

import copy
import glob
import math
import os
import sys

import numpy as np
from scipy.optimize import minimize

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "gnss_sim"))
import gps_noise_v2 as gn            # noqa: E402
import urbannav_targets as ut        # noqa: E402

STOPPED, MOVING = gn.STOPPED_BELOW, gn.MOVING_ABOVE
LAGS = {"stopped": (1, 10), "moving": (1, 10, 60)}   # s — σημεία του variogram που συγκρίνονται
JUMP = 10.0                                           # m — «απότομο άλμα» σε 1 s
N_SEEDS = 6
ROUNDS = 2
MAXITER = 160
RATIO = {"f9p": {"MILD+MOD": 1.18, "STRONG": 0.68},  # μετρημένοι λόγοι σε κίνηση (απόφαση 5β)
         "m8t": {"MILD+MOD": 0.77, "STRONG": 0.66}}
STOPPED_RATIO = 1.0                                   # ΠΑΡΑΔΟΧΗ: σε στάση δεν μετριέται


# ─── δεδομένα ────────────────────────────────────────────────────────────────
def route(route_name, receivers):
    """Πραγματική διαδρομή και πραγματικά σφάλματα των δεκτών, ανά δευτερόλεπτο.

    Επιστρέφει dict: t, xy (m, ENU), speed (m/s), yaw (μοίρες), real = [(t, err[N×2]) ανά ροή].
    """
    truth = ut.read_truth(route_name)
    t = np.array(sorted(truth))
    lat0, lon0 = truth[t[0]]
    xy = ut.enu(*np.array([truth[k] for k in t]).T, lat0, lon0)
    vel = np.zeros_like(xy)
    vel[1:-1] = (xy[2:] - xy[:-2]) / 2
    speed = np.hypot(*vel.T)
    yaw = np.degrees(np.arctan2(vel[:, 1], vel[:, 0]))
    for i in range(1, len(yaw)):                     # σε στάση η κατεύθυνση δεν ορίζεται: κρατάμε την προηγούμενη
        if speed[i] < 0.3:
            yaw[i] = yaw[i - 1]
    real = []
    for rec in receivers:
        (path,) = glob.glob(os.path.join(ut.DATA, route_name, f"*{rec}.nmea"))
        fixes = ut.read_gga(path)
        keep = np.array([k in fixes for k in t])
        est = ut.enu(*np.array([fixes[k] for k in t[keep]]).T, lat0, lon0)
        real.append((t[keep], est - xy[keep]))
    return dict(t=t, xy=xy, speed=speed, yaw=yaw, real=real)


def simulate(r, receiver, level, profile, seed):
    """Το μοντέλο «οδηγεί» πάνω στη διαδρομή r με σταθερή ζώνη. Επιστρέφει (t, err[N×2])."""
    gn.RECEIVERS[receiver] = profile
    zones = [dict(name="cal", x_min=-math.inf, x_max=math.inf, y_min=-math.inf, y_max=math.inf, level=level)]
    noise = gn.GpsNoise(receiver, seed=seed, zones=zones)
    err = np.empty_like(r["xy"])
    for i, (x, y) in enumerate(r["xy"]):
        fx, fy = noise.step(x, y, r["yaw"][i], r["speed"][i])
        err[i] = (fx - x, fy - y)
    return r["t"], err


# ─── στατιστικά (ΙΔΙΑ για δεδομένα και μοντέλο) ──────────────────────────────
def stats(r, series, state):
    """Στατιστικά του σφάλματος σε μία κατάσταση, από λίστα σειρών (t, err) πάνω στη διαδρομή r."""
    idx = {k: i for i, k in enumerate(r["t"])}
    sp, yaw = r["speed"], np.radians(r["yaw"])
    in_state = (lambda v: v < STOPPED) if state == "stopped" else (lambda v: v > MOVING)
    e2d, along, var_num, var_den, jumps, seconds = [], [], {L: 0.0 for L in LAGS[state]}, {L: 0 for L in LAGS[state]}, 0, 0
    for t, err in series:
        pos = np.array([idx[k] for k in t])
        m = in_state(sp[pos])
        e2d.append(np.hypot(*err[m].T))
        if state == "moving":
            u = np.stack([np.cos(yaw[pos[m]]), np.sin(yaw[pos[m]])], 1)
            along.append(np.sum(err[m] * u, 1))
        where = {k: j for j, k in enumerate(t)}
        for L in LAGS[state]:
            j = np.array([where.get(k + L, -1) for k in t])
            ok = (j >= 0) & m
            ok[ok] &= m[j[ok]]
            d = err[j[ok]] - err[ok]
            var_num[L] += np.sum(d ** 2) / 2
            var_den[L] += ok.sum()
        j = np.array([where.get(k + 1, -1) for k in t])
        ok = (j >= 0) & m
        ok[ok] &= m[j[ok]]
        jumps += np.sum(np.hypot(*(err[j[ok]] - err[ok]).T) > JUMP)
        seconds += ok.sum()
    e = np.concatenate(e2d)
    out = {"mean": e.mean(), "median": np.median(e)}
    for L in LAGS[state]:
        out[f"V{L}"] = math.sqrt(var_num[L] / var_den[L]) if var_den[L] else float("nan")
    out["jumps/h"] = jumps / seconds * 3600 if seconds else float("nan")
    if state == "moving":
        out["along"] = np.concatenate(along).mean()
    return out


# ─── ομάδες προς ρύθμιση ─────────────────────────────────────────────────────
# Κάθε «μπλοκ» = ένα κελί του RECEIVERS (ή κοινό κελί) σε μία κατάσταση, με τις διαδρομές του.
def blocks(receiver):
    if receiver == "f9p":
        return [("MILD+MOD", ["medium", "deep"], ["MILD", "MODERATE"], st) for st in ("moving", "stopped")] + \
               [("STRONG", ["harsh"], ["STRONG"], st) for st in ("moving", "stopped")]
    return [("MILD+MOD", ["medium", "deep"], ["MILD", "MODERATE"], "moving"),
            ("MILD", ["medium"], ["MILD"], "stopped"),
            ("MODERATE", ["deep"], ["MODERATE"], "stopped"),
            ("STRONG", ["harsh"], ["STRONG"], "moving"),
            ("STRONG", ["harsh"], ["STRONG"], "stopped")]


ROUTE_LEVEL = {"medium": "MILD", "deep": "MODERATE", "harsh": "STRONG"}


def free_params(receiver, state):
    names = ["sigma", "tau"]
    if receiver == "m8t":
        names += ["step_rate", "step_size"] + (["lag"] if state == "moving" else [])
    return names


def targets(receiver, state):
    names = ["mean", "median"] + [f"V{L}" for L in LAGS[state]]
    if receiver == "m8t":
        names += ["jumps/h"] + (["along"] if state == "moving" else [])
    return names


START = {"sigma": 5.0, "tau": 120.0, "step_rate": 1 / 60, "step_size": 30.0, "lag": 5.0}
# Λογικά όρια. Χωρίς αυτά, στις σύντομες στάσεις η ρύθμιση «ξεφεύγει» σε άπειρο tau με τεράστιο sigma:
# σε λίγα δευτερόλεπτα το μοντέλο δεν ξεχωρίζει «μεγάλο sigma / τεράστιο tau» από «μικρό sigma / κανονικό tau».
BOUNDS = {"sigma": (0.1, 1000.0), "tau": (2.0, 3600.0), "step_rate": (1e-5, 0.5), "step_size": (0.5, 1000.0),
          "lag": (0.1, 120.0)}


def initial_profile(receiver):
    """Αντίγραφο του RECEIVERS με αρχικές τιμές (κρατάει τα κοινά λεξικά κοινά)."""
    prof = copy.deepcopy(gn.RECEIVERS[receiver])
    seen = set()
    for z in prof:
        for st, cell in prof[z].items():
            if id(cell) in seen:
                continue
            seen.add(id(cell))
            group = "STRONG" if z == "STRONG" else "MILD+MOD"
            cell["ratio"] = RATIO[receiver][group] if st == "moving" else STOPPED_RATIO
            for k in cell:
                if k != "ratio":
                    cell[k] = START[k]
    return prof


def fit_block(receiver, prof, data, real_stats, block):
    name, routes, levels, state = block
    params = free_params(receiver, state)
    tnames = targets(receiver, state)
    cell = prof[levels[0]][state]                      # κοινό λεξικό → αλλάζει παντού όπου μοιράζεται

    def model_stats(values):
        for k, v in zip(params, values):
            cell[k] = v
        series = {}
        for rt in routes:
            series[rt] = [simulate(data[rt], receiver, ROUTE_LEVEL[rt], prof, seed) for seed in range(N_SEEDS)]
        return stats_multi(data, routes, series, state)

    lo = np.log([BOUNDS[k][0] for k in params])
    hi = np.log([BOUNDS[k][1] for k in params])

    def loss(logv):
        logv = np.clip(logv, lo, hi)
        s = model_stats(np.exp(logv))
        err = 0.0
        for k in tnames:
            a, b = s[k], real_stats[k]
            if k == "along":                           # προσημασμένο: απόλυτη διαφορά σε κλίμακα του μέσου
                err += ((a - b) / max(1.0, real_stats["mean"])) ** 2
            elif k == "jumps/h":
                err += (math.log((a + 1) / (b + 1))) ** 2
            else:
                err += (math.log(max(a, 1e-6) / max(b, 1e-6))) ** 2
        return err

    x0 = np.log([cell[k] for k in params])
    res = minimize(loss, x0, method="Nelder-Mead", options={"maxiter": MAXITER, "xatol": 0.02, "fatol": 1e-4})
    best = np.exp(np.clip(res.x, lo, hi))
    for k, v in zip(params, best):
        cell[k] = float(v)
    return model_stats(best), res.fun


def stats_multi(data, routes, series_by_route, state):
    """Στατιστικά πολλών διαδρομών μαζί (δεδομένα ή μοντέλο), με βάρη το πλήθος των δευτερολέπτων."""
    parts = [(stats(data[rt], series_by_route[rt], state), data[rt], series_by_route[rt]) for rt in routes]
    if len(parts) == 1:
        return parts[0][0]
    # για κοινά κελιά: ενώνουμε τα σφάλματα όλων των διαδρομών πριν από τα στατιστικά
    merged = {"t": [], "speed": [], "yaw": []}
    series, offset = [], 0
    for rt in routes:
        d = data[rt]
        series += [(t + offset - d["t"][0], err) for t, err in series_by_route[rt]]
        merged["t"].append(d["t"] - d["t"][0] + offset)
        merged["speed"].append(d["speed"])
        merged["yaw"].append(d["yaw"])
        offset += d["t"][-1] - d["t"][0] + 10_000       # κενό ώστε να μη «γεφυρώνονται» διαδρομές
    m = {k: np.concatenate(v) for k, v in merged.items()}
    return stats(m, series, state)


def main():
    receiver = sys.argv[1] if len(sys.argv) > 1 else "f9p"
    streams = ut.RECEIVERS[receiver]
    data = {rt: route(rt, streams) for rt in ROUTE_LEVEL}
    prof = initial_profile(receiver)
    plan = blocks(receiver)
    real = {b[:1] + (b[3],): stats_multi(data, b[1], {rt: data[rt]["real"] for rt in b[1]}, b[3]) for b in plan}

    for rnd in range(ROUNDS):
        print(f"\n— γύρος {rnd + 1} —")
        for b in plan:
            s, l = fit_block(receiver, prof, data, real[b[:1] + (b[3],)], b)
            print(f"  {b[0]:<9} {b[3]:<8} loss {l:7.4f}")

    print(f"\n=== {receiver}: μοντέλο έναντι πραγματικών ===")
    for b in plan:
        name, routes, levels, state = b
        cell = prof[levels[0]][state]
        ms = stats_multi(data, routes, {rt: [simulate(data[rt], receiver, ROUTE_LEVEL[rt], prof, 1000 + s)
                                             for s in range(N_SEEDS)] for rt in routes}, state)
        rs = real[b[:1] + (b[3],)]
        print(f"\n{name} / {state}:  " + ", ".join(f"{k}={v:.4g}" for k, v in cell.items()))
        for k in targets(receiver, state):
            print(f"    {k:<8} πραγματικό {rs[k]:8.2f}   μοντέλο {ms[k]:8.2f}")

    print("\nΤιμές για το RECEIVERS (στρογγυλεμένες):")
    for z in prof:
        for st, cell in prof[z].items():
            print(f"  {receiver} {z:<9} {st:<8} " + ", ".join(f'"{k}": {v:.3g}' for k, v in cell.items()))


if __name__ == "__main__":
    main()
