"""
Στόχοι ρύθμισης του θορύβου GPS ανά ζώνη και κατάσταση (σταματημένο / κινείται),
από τα πραγματικά δεδομένα του UrbanNav. Βγάζει τους πίνακες των αποφάσεων 15–16 Οκτ 2026.

Αποφάσεις (βλ. συζήτηση με τον χρήστη):
  - ζώνη ← διαδρομή, με βάση την κάλυψη ουρανού (urbannav_skymask.py):
      MILD ← medium (Tsim Sha Tsui), MODERATE ← deep (Whampoa), STRONG ← harsh (Mong Kok)
  - δύο καταστάσεις: σταματημένο (< 0.5 m/s) και κινείται (> 2 m/s). ΠΑΡΑΔΟΧΗ τα όρια·
    τα δευτερόλεπτα ανάμεσα (ξεκινάει / φρενάρει) δεν μετράνε
  - παλιός δέκτης: μόνο οι ροές M8T GC και GEJ (ίδια ρύθμιση σε όλες τις διαδρομές)
  - σύγχρονος δέκτης: u-blox F9P (και οι δύο ροές). Τα κινητά μόνο για σύγκριση: τα αυτόνομα
    οχήματα δεν έχουν κινητό
  - στόχοι: μέσο ΚΑΙ διάμεσο, ανοχή = 90% διάστημα block bootstrap (μπλοκ 60 s, γιατί τα
    διαδοχικά δευτερόλεπτα δεν είναι ανεξάρτητα)
  - MILD / MODERATE: χωριστές μόνο όπου η διαφορά τους είναι στατιστικά σαφής, αλλιώς ΚΟΙΝΕΣ
    τιμές (medium + deep μαζί). Ο κανόνας ανά δέκτη είναι στο RULES:
      M8T: χωριστά σε στάση (MILD < MODERATE στο 95% του bootstrap, διάμεσο), κοινό σε κίνηση
           (σε ίδια ταχύτητα 2–6 m/s δεν διαφέρουν· η διαφορά οφείλεται στο ότι το όχημα του
           UrbanNav έτρεχε πιο γρήγορα στο medium)
      F9P: κοινό και στις δύο καταστάσεις (διαφορές ≤ ~1 m, χωρίς σταθερή φορά, πιθανώς μέσα
           στο λάθος της ίδιας της πραγματικής θέσης)

Ανεξάρτητο από το urbannav_errors.py: διαβάζει μόνο τα αρχικά αρχεία, κρατά μόνο προτάσεις
GGA με σωστό checksum, και μετατρέπει σε μέτρα με ακριβή WGS84 → ENU.

    python gnss_sim/urbannav_targets.py              # παλιός δέκτης (M8T GC + GEJ)
    python gnss_sim/urbannav_targets.py f9p          # σύγχρονος δέκτης
    python gnss_sim/urbannav_targets.py phones       # μόνο σύγκριση (χωρίς τελικούς στόχους)
"""

import glob
import os
import re
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.join(HERE, "urbannav")

ROUTES = {"medium": "MILD", "deep": "MODERATE", "harsh": "STRONG"}
RECEIVERS = {
    "m8t": ["ublox.m8t.GC", "ublox.m8t.GEJ"],
    "f9p": ["ublox.f9p", "ublox.f9p.splitter"],
    "phones": ["google.pixel4", "huawei.p40pro", "samsung.note8", "xiaomi.mi8"],
}
# MILD / MODERATE ανά κατάσταση: "separate" = χωριστά, "common" = κοινή τιμή (medium + deep)
RULES = {
    "m8t": {"σταματημένο": "separate", "κινείται": "common"},   # απόφαση 16 Οκτ 2026
    "f9p": {"σταματημένο": "common", "κινείται": "common"},     # απόφαση 16 Οκτ 2026
}
STOPPED, MOVING = 0.5, 2.0          # m/s — ΠΑΡΑΔΟΧΗ
SAME_SPEED = (2.0, 6.0)             # m/s — για τον έλεγχο «ίδια ταχύτητα»
BLOCK = 60                          # s — μήκος μπλοκ στο bootstrap
N_BOOT = 2000
SEED = 0

# --- WGS84 → τοπικά μέτρα (ανατολή, βορράς) ----------------------------------
A, F = 6378137.0, 1 / 298.257223563
E2 = F * (2 - F)


def ecef(lat, lon):
    la, lo = np.radians(lat), np.radians(lon)
    n = A / np.sqrt(1 - E2 * np.sin(la) ** 2)
    return np.stack([n * np.cos(la) * np.cos(lo), n * np.cos(la) * np.sin(lo), n * (1 - E2) * np.sin(la)], -1)


def enu(lat, lon, lat0, lon0):
    d = ecef(lat, lon) - ecef(lat0, lon0)
    la, lo = np.radians(lat0), np.radians(lon0)
    e = -np.sin(lo) * d[..., 0] + np.cos(lo) * d[..., 1]
    n = -np.sin(la) * np.cos(lo) * d[..., 0] - np.sin(la) * np.sin(lo) * d[..., 1] + np.cos(la) * d[..., 2]
    return np.stack([e, n], -1)


# --- ανάγνωση ----------------------------------------------------------------
def read_truth(route):
    """Πραγματική θέση: δευτερόλεπτο της ημέρας UTC → (lat, lon)."""
    (path,) = glob.glob(os.path.join(DATA, route, "*_raw.txt"))
    truth = {}
    for line in open(path):
        p = line.split()
        if len(p) >= 20 and p[0][0].isdigit():
            t = int(round(float(p[0]))) % 86400
            truth[t] = (float(p[3]) + float(p[4]) / 60 + float(p[5]) / 3600,
                        float(p[6]) + float(p[7]) / 60 + float(p[8]) / 3600)
    return truth


GGA = re.compile(rb"\$G[NP]GGA,([^*\r\n$]*)\*([0-9A-Fa-f]{2})")


def read_gga(path):
    """Θέσεις του δέκτη (GGA με σωστό checksum και θέση): δευτερόλεπτο UTC → (lat, lon)."""
    fixes = {}
    for m in GGA.finditer(open(path, "rb").read()):
        body = b"GPGGA," + m.group(1) if m.group(0)[2:3] == b"P" else b"GNGGA," + m.group(1)
        checksum = 0
        for byte in body:
            checksum ^= byte
        if checksum != int(m.group(2), 16):
            continue
        p = m.group(1).decode().split(",")
        if p[5] in ("", "0") or not p[1]:
            continue
        t = int(round(int(p[0][:2]) * 3600 + int(p[0][2:4]) * 60 + float(p[0][4:]))) % 86400
        lat = (int(p[1][:2]) + float(p[1][2:]) / 60) * (1 if p[2] == "N" else -1)
        lon = (int(p[3][:3]) + float(p[3][3:]) / 60) * (1 if p[4] == "E" else -1)
        fixes[t] = (lat, lon)
    return fixes


def route_series(route, receivers):
    """Για κάθε ροή: (δευτερόλεπτα από την αρχή, 2D σφάλμα σε m, ταχύτητα οχήματος σε m/s)."""
    truth = read_truth(route)
    times = sorted(truth)
    lat0, lon0 = truth[times[0]]
    xy = dict(zip(times, enu(*np.array([truth[t] for t in times]).T, lat0, lon0)))
    speed = {t: np.hypot(*(xy[t + 1] - xy[t - 1])) / 2 for t in times if t - 1 in xy and t + 1 in xy}
    series = []
    for rec in receivers:
        (path,) = glob.glob(os.path.join(DATA, route, f"*{rec}.nmea"))
        fixes = read_gga(path)
        ts = np.array([t for t in times if t in fixes and t in speed])
        est = enu(*np.array([fixes[t] for t in ts]).T, lat0, lon0)
        err = np.hypot(*(est - np.array([xy[t] for t in ts])).T)
        series.append((ts - times[0], err, np.array([speed[t] for t in ts])))
    return series


# --- στατιστικά --------------------------------------------------------------
def pick(series, keep, blocks=None):
    """Σφάλματα των δευτερολέπτων όπου keep(ταχύτητα), προαιρετικά μόνο από τα μπλοκ blocks."""
    out = []
    for k, e, v in series:
        m = keep(v)
        if blocks is None:
            out.append(e[m])
        else:
            out += [e[m & (k >= b * BLOCK) & (k < (b + 1) * BLOCK)] for b in blocks]
    return np.concatenate(out)


def bootstrap(routes_series, keep, rng):
    """Μέσο και διάμεσο σε N_BOOT επαναδειγματοληψίες μπλοκ (κάθε διαδρομή χωριστά, μετά μαζί)."""
    means, medians = [], []
    for _ in range(N_BOOT):
        parts = []
        for series in routes_series:
            n_blocks = int(max(k.max() for k, e, v in series) // BLOCK) + 1
            parts.append(pick(series, keep, rng.integers(0, n_blocks, n_blocks)))
        x = np.concatenate(parts)
        means.append(x.mean() if len(x) else np.nan)
        medians.append(np.median(x) if len(x) else np.nan)
    return np.array(means), np.array(medians)


def cell(routes_series, keep, rng):
    x = np.concatenate([pick(s, keep) for s in routes_series])
    bm, bd = bootstrap(routes_series, keep, rng)
    (m0, m1), (d0, d1) = np.nanpercentile(bm, [5, 95]), np.nanpercentile(bd, [5, 95])
    return (f"{x.mean():5.1f} [{m0:5.1f}–{m1:5.1f}] / {np.median(x):5.1f} [{d0:5.1f}–{d1:5.1f}] m ({len(x)} s)",
            bm, bd)


if __name__ == "__main__":
    group = sys.argv[1] if len(sys.argv) > 1 else "m8t"
    receivers = RECEIVERS[group]
    rng = np.random.default_rng(SEED)
    stopped = lambda v: v < STOPPED                                    # noqa: E731
    moving = lambda v: v > MOVING                                      # noqa: E731
    same_speed = lambda v: (v > SAME_SPEED[0]) & (v <= SAME_SPEED[1])  # noqa: E731

    data = {route: route_series(route, receivers) for route in ROUTES}
    print(f"Δέκτες: {', '.join(receivers)}")
    print(f"Κάθε κελί: μέσο [90% διάστημα] / διάμεσο [90% διάστημα], block bootstrap {BLOCK} s × {N_BOOT}\n")

    print("1. Ανά ζώνη και κατάσταση, χωριστά:")
    boots = {}
    for state, keep in (("σταματημένο", stopped), ("κινείται", moving)):
        for route, zone in ROUTES.items():
            text, bm, bd = cell([data[route]], keep, rng)
            boots[(state, route)] = (bm, bd)
            print(f"  {state:<12} {zone:<9} {text}")

    print("\n2. Έλεγχοι για MILD / MODERATE (πόσο συχνά MILD < MODERATE στις επαναδειγματοληψίες):")
    for state in ("σταματημένο", "κινείται"):
        (am, ad), (bm, bd) = boots[(state, "medium")], boots[(state, "deep")]
        print(f"  {state:<12} μέσο {100 * np.nanmean(am < bm):3.0f}%   διάμεσο {100 * np.nanmean(ad < bd):3.0f}%")
    for route, zone in (("medium", "MILD"), ("deep", "MODERATE")):
        text, _, _ = cell([data[route]], same_speed, rng)
        print(f"  κινείται {SAME_SPEED[0]:g}–{SAME_SPEED[1]:g} m/s, {zone:<9} {text}")
    for route in ("medium", "deep"):
        v = np.concatenate([s[2] for s in data[route]])
        v = v[v > MOVING]
        print(f"  ταχύτητα όταν κινείται, {route}: διάμεσος {np.median(v):.1f} m/s, πάνω από 6 m/s το {100 * np.mean(v > 6):.0f}%")

    if group not in RULES:
        print("\n3. Τελικοί στόχοι: κανένας (αυτή η ομάδα δεκτών είναι μόνο για σύγκριση).")
        sys.exit()
    print("\n3. Τελικοί στόχοι (MILD / MODERATE: " +
          ", ".join(f"{state} {'χωριστά' if rule == 'separate' else 'κοινό'}" for state, rule in RULES[group].items()) + "):")
    for state, keep in (("σταματημένο", stopped), ("κινείται", moving)):
        if RULES[group][state] == "separate":
            for route in ("medium", "deep"):
                text, _, _ = cell([data[route]], keep, rng)
                print(f"  {state:<12} {ROUTES[route]:<24} {text}")
        else:
            text, _, _ = cell([data["medium"], data["deep"]], keep, rng)
            print(f"  {state:<12} {'MILD + MODERATE (κοινό)':<24} {text}")
        text, _, _ = cell([data["harsh"]], keep, rng)
        print(f"  {state:<12} {'STRONG':<24} {text}")
