"""
Λάθος θέσης ΠΡΑΓΜΑΤΙΚΩΝ δεκτών GNSS στο Χονγκ Κονγκ, από το dataset UrbanNav.

UrbanNav (Hsu et al., IPNL PolyU — https://github.com/IPNL-POLYU/UrbanNavDataset, README στο
gnss_sim/urbannav/UrbanNav_README.md). Τρεις διαδρομές, με την ετικέτα «Urban Canyon» του README:
  medium — Medium-Urban-1, Tsim Sha Tsui (TST), «Medium»
  deep   — Deep-Urban-1, Whampoa, «Deep»
  harsh  — Harsh-Urban-1, Mong Kok, «Harsh» (πραγματική θέση επαληθευμένη μόνο για μέρος, README γρ. 143)
Για κάθε διαδρομή:
  - θέση που έβγαλε ο ίδιος ο δέκτης: αρχεία .nmea, πρόταση GGA, 1 Hz
  - πραγματική θέση: *_raw.txt, NovAtel SPAN-CPT + IE, 1 Hz, με στήλη ποιότητας Q
    (το README δεν ορίζει τι σημαίνει το Q· το κρατάμε στα αποτελέσματα)
Τα δεδομένα είναι στο gnss_sim/urbannav/<διαδρομή>/ (git-ignored, δεδομένα τρίτων).

Για κάθε δέκτη: ταιριάζουμε κάθε θέση του δέκτη με την πραγματική θέση της ίδιας
ώρας (UTC, ανά δευτερόλεπτο) και μετράμε το 2D λάθος, και κατά μήκος / εγκάρσια ως προς
την κατεύθυνση κίνησης (από την πραγματική τροχιά). Ο λόγος μετριέται μόνο όταν το όχημα
κινείται (> 2 m/s), αλλιώς η κατεύθυνση δεν ορίζεται.

    python gnss_sim/urbannav_errors.py            # deep (Whampoa)
    python gnss_sim/urbannav_errors.py medium     # ή harsh

TAU: η χρονική υστέρηση (s) όπου η αυτοσυσχέτιση του σφάλματος πέφτει κάτω από 1/e,
χωριστά για ανατολή και βορρά (σφάλμα χωρίς τη μέση τιμή του, ζεύγη χωρίς κενά).

Γράφει: gnss_sim/urbannav/<διαδρομή>/errors_<δέκτης>.csv, gnss_sim/urbannav_errors_<διαδρομή>.png
        και, μόνο για το deep, gnss_sim/urbannav_jumps.png (400 s: πραγματικοί δέκτες δίπλα στο gps_noise.py)
"""

import glob
import math
import os
import re
import sys

import matplotlib.pyplot as plt
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROUTES = {"medium": "Medium-Urban-1 (Tsim Sha Tsui)", "deep": "Deep-Urban-1 (Whampoa)",
          "harsh": "Harsh-Urban-1 (Mong Kok)"}
# όνομα δέκτη μέσα στο όνομα αρχείου (στο harsh δύο αρχεία έχουν άλλο πρόθεμα)
RECEIVER = re.compile(r"(google\.pixel4|huawei\.p40pro|samsung\.note8|xiaomi\.mi8|"
                      r"ublox\.f9p(?:\.splitter)?|ublox\.m8t\.\w+)\.nmea$")
R_EARTH = 6378137.0
MIN_SPEED = 2.0       # m/s — κάτω από αυτό δεν μετράμε κατά μήκος / εγκάρσια


def read_truth(data):
    """Πραγματική θέση: δευτερόλεπτο της ημέρας UTC → (lat, lon, Q). Μοίρες· Q η στήλη ποιότητας."""
    (path,) = glob.glob(os.path.join(data, "*_raw.txt"))
    truth = {}
    with open(path) as f:
        for line in f:
            p = line.split()
            if len(p) < 20 or not p[0][0].isdigit():
                continue
            t = round(float(p[0])) % 86400
            lat = float(p[3]) + float(p[4]) / 60 + float(p[5]) / 3600
            lon = float(p[6]) + float(p[7]) / 60 + float(p[8]) / 3600
            truth[t] = (lat, lon, int(p[19]))
    return truth


def read_gga(path):
    """Θέσεις του δέκτη από τις προτάσεις GGA: δευτερόλεπτο UTC → (lat, lon). Μόνο με θέση (quality > 0)."""
    fixes = {}
    with open(path, "rb") as f:
        for raw in f:
            line = raw.decode("ascii", errors="ignore").strip()
            i = line.find("GGA,")
            if i < 0:
                continue
            p = line[i:].split(",")
            if len(p) < 7 or not p[1] or not p[2] or not p[6] or p[6] == "0":
                continue
            try:
                hh, mm, ss = int(p[1][:2]), int(p[1][2:4]), float(p[1][4:])
                lat = int(p[2][:2]) + float(p[2][2:]) / 60
                lon = int(p[4][:3]) + float(p[4][3:]) / 60
            except ValueError:
                continue
            if p[3] == "S":
                lat = -lat
            if p[5] == "W":
                lon = -lon
            fixes[round(hh * 3600 + mm * 60 + ss) % 86400] = (lat, lon)
    return fixes


def to_xy(lat, lon, lat0, lon0):
    """Μοίρες → μέτρα (ανατολή, βορράς) γύρω από (lat0, lon0). Αρκεί για περιοχή λίγων km."""
    x = math.radians(lon - lon0) * R_EARTH * math.cos(math.radians(lat0))
    y = math.radians(lat - lat0) * R_EARTH
    return x, y


def analyse(truth, fixes):
    times = sorted(truth)
    lat0, lon0 = truth[times[0]][:2]
    txy = {t: to_xy(*truth[t][:2], lat0, lon0) for t in times}
    rows = []
    for t in times:
        if t - 1 not in txy or t + 1 not in txy:
            continue
        vx = (txy[t + 1][0] - txy[t - 1][0]) / 2
        vy = (txy[t + 1][1] - txy[t - 1][1]) / 2
        speed = math.hypot(vx, vy)
        q = truth[t][2]
        if t not in fixes:
            rows.append((t, np.nan, np.nan, np.nan, np.nan, speed, q))
            continue
        fx, fy = to_xy(*fixes[t], lat0, lon0)
        dx, dy = fx - txy[t][0], fy - txy[t][1]
        if speed > MIN_SPEED:
            ux, uy = vx / speed, vy / speed
            along, cross = dx * ux + dy * uy, -dx * uy + dy * ux
        else:
            along = cross = np.nan
        rows.append((t, dx, dy, along, cross, speed, q))
    return np.array(rows)


def tau(x, max_lag=600):
    """Υστέρηση (s) όπου η αυτοσυσχέτιση της σειράς x (1 Hz, με NaN για κενά) πέφτει κάτω από 1/e."""
    x = x - np.nanmean(x)
    var = np.nanmean(x * x)
    for lag in range(1, max_lag):
        a, b = x[:-lag], x[lag:]
        ok = ~np.isnan(a) & ~np.isnan(b)
        if np.mean(a[ok] * b[ok]) / var < 1 / math.e:
            return lag
    return None


def jumps_figure(results, t0=300, t1=700):
    """Ίδιο παράθυρο για 3 πραγματικούς δέκτες και για το gps_noise.py (ζώνες STRONG, MODERATE)."""
    sys.path.insert(0, os.path.dirname(HERE))
    from gps_noise import MODERATE, STRONG, GpsNoise

    def real(name):
        r = results[name]
        t = r[:, 0] - r[0, 0]
        m = (t >= t0) & (t < t1)
        return t[m], np.hypot(r[m, 1], r[m, 2])

    def ours(zone, seed=1):
        everywhere = [dict(zone, x_min=-math.inf, x_max=math.inf, y_min=-math.inf, y_max=math.inf)]
        noise = GpsNoise(zones=everywhere, dt=1.0, seed=seed)
        x, out = 0.0, []
        for _ in range(t1):
            x += 8.0
            fx, fy = noise.step(x, 0.0, 0.0)
            out.append(math.hypot(fx - x, fy))
        return np.arange(t0, t1), np.array(out[t0:t1])

    panels = [
        ("Πραγματικό: u-blox M8T (GEJ) — ο δέκτης του [S4]", real("ublox.m8t.GEJ")),
        ("Πραγματικό: κινητό Google Pixel 4", real("google.pixel4")),
        ("Πραγματικό: u-blox F9P", real("ublox.f9p")),
        ("Δικό μας gps_noise.py: ζώνη STRONG", ours(STRONG)),
        ("Δικό μας gps_noise.py: ζώνη MODERATE", ours(MODERATE)),
    ]
    fig, axes = plt.subplots(len(panels), 1, figsize=(13, 12), sharex=True)
    for ax, (title, (t, e)) in zip(axes, panels):
        ax.plot(t, e, lw=1.2, color="C3" if "Δικό" in title else "C0")
        ax.set_title(title, fontsize=10, loc="left")
        ax.set_ylabel("λάθος (m)")
        ax.set_ylim(0, max(20, np.nanmax(e) * 1.1))
        ax.grid(alpha=0.3)
    axes[-1].set_xlabel(f"χρόνος (s) — ίδιο παράθυρο {t1 - t0} s για όλους")
    fig.suptitle("Πώς μοιάζει το λάθος στον χρόνο: πραγματικοί δέκτες (Whampoa, UrbanNav) vs το δικό μας μοντέλο\n"
                 "ΠΡΟΣΟΧΗ: κάθε γραμμή έχει δική της κλίμακα στον κάθετο άξονα", fontsize=11)
    plt.tight_layout()
    plt.savefig(os.path.join(HERE, "urbannav_jumps.png"), dpi=80)


if __name__ == "__main__":
    route = sys.argv[1] if len(sys.argv) > 1 else "deep"
    data = os.path.join(HERE, "urbannav", route)
    truth = read_truth(data)
    qs = np.array([v[2] for v in truth.values()])
    print(f"{ROUTES[route]}: πραγματική θέση {len(truth)} δευτερόλεπτα, "
          f"Q: " + ", ".join(f"{q}={100 * np.mean(qs == q):.0f}%" for q in sorted(set(qs))) + "\n")
    print(f"{'δέκτης':<22} {'με θέση':>8} {'μέσο':>7} {'διάμ.':>7} {'STD':>7} {'RMS':>7} {'P95':>7} {'max':>7}  "
          f"|εγκ.|/|κ.μ.|  TAU αν./βορ.  μέσο(Q=1)")
    results = {}
    for path in sorted(glob.glob(os.path.join(data, "*.nmea"))):
        m = RECEIVER.search(os.path.basename(path))
        if not m:
            continue
        name = m.group(1)
        fixes = read_gga(path)
        if not fixes:
            print(f"{name:<22} καμία θέση GGA")
            continue
        r = analyse(truth, fixes)
        results[name] = r
        np.savetxt(os.path.join(data, f"errors_{name}.csv"), r, delimiter=",", fmt="%.3f",
                   header="utc_sec,err_east,err_north,err_along,err_cross,speed,q", comments="")
        e = np.hypot(r[:, 1], r[:, 2])
        ok = ~np.isnan(e)
        moving = ~np.isnan(r[:, 3])
        ratio = np.mean(np.abs(r[moving, 4])) / np.mean(np.abs(r[moving, 3]))
        q1 = ok & (r[:, 6] == 1)
        q1_mean = f"{e[q1].mean():6.1f}m ({q1.sum()} s)" if q1.any() else "—"
        print(f"{name:<22} {100 * ok.mean():7.0f}% {e[ok].mean():6.1f}m {np.median(e[ok]):6.1f}m {e[ok].std():6.1f}m "
              f"{math.sqrt(np.mean(e[ok] ** 2)):6.1f}m {np.percentile(e[ok], 95):6.1f}m {e[ok].max():6.0f}m  "
              f"{ratio:11.2f}  {str(tau(r[:, 1])):>4} / {str(tau(r[:, 2])):>4} s  {q1_mean}")
    print("\n[S4] canyon 1 (u-blox M8T, NMEA):   100%   31.0m       —   37.7m  ~48.8m       —    178m")

    fig, axes = plt.subplots(len(results), 1, figsize=(13, 2.2 * len(results)), sharex=True)
    for ax, (name, r) in zip(axes, results.items()):
        t = r[:, 0] - r[0, 0]
        ax.plot(t, np.hypot(r[:, 1], r[:, 2]), lw=0.8)
        ax.set_ylabel("m")
        ax.set_title(name, fontsize=9, loc="left")
        ax.set_ylim(0, 150)
    axes[-1].set_xlabel("χρόνος (s)")
    fig.suptitle(f"UrbanNav {ROUTES[route]}: 2D λάθος θέσης ανά δευτερόλεπτο")
    plt.tight_layout()
    out = f"urbannav_errors_{route}.png"
    plt.savefig(os.path.join(HERE, out), dpi=80)
    if route == "deep":
        jumps_figure(results)
        out += ", urbannav_jumps.png"
    print(f"Αποθηκεύτηκαν: {out}")
