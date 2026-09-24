"""
Δοκιμή του gps_noise.py χωρίς CARLA.

Ένα ψεύτικο όχημα κάνει γύρους σε μια διαδρομή πάνω στους δρόμους του Town10HD,
και το GpsNoise δίνει θορυβώδεις μετρήσεις. Βγάζει:
  - gps_noise_test.png: χάρτης (αλήθεια vs GPS) + σφάλμα στον χρόνο
  - στατιστικά σφάλματος ανά ζώνη στο τερματικό

Οι ζώνες εδώ είναι ΕΝΔΕΙΚΤΙΚΕΣ, μόνο για να φανεί η συμπεριφορά.
Οι πραγματικές θα οριστούν όταν έχουμε τα κτήρια (export_buildings.py).
"""

import math
import numpy as np
import matplotlib.pyplot as plt

from gps_noise import GpsNoise, zone_at

TEST_ZONES = [
    # στενός δρόμος με ψηλά κτήρια (τιμές από GPS_NOISE_MODEL.md §7)
    dict(name="canyon", x_min=-80, x_max=80, y_min=0, y_max=40,
         sigma_along=12.0, sigma_cross=25.0, nlos_rate=1 / 20, dropout=False),
    # σημείο χωρίς σήμα (π.χ. κάτω από γέφυρα)
    dict(name="dropout", x_min=100, x_max=115, y_min=60, y_max=80,
         sigma_along=2.5, sigma_cross=2.5, nlos_rate=0.0, dropout=True),
]

# Διαδρομή σε μέτρα CARLA: ο οριζόντιος δρόμος στο y≈20 και ο δεξιός στο x≈107.
ROUTE = [(-110, 20), (105, 20), (107, 130), (-110, 130), (-110, 20)]
SPEED = 8.0     # m/s (~30 km/h)
DT = 1.0        # s — δέκτης 1 Hz, όπως ο u-blox του [S4]
LAPS = 5


def drive(route, speed, dt, laps):
    """Θέσεις και yaw του οχήματος κάθε dt δευτερόλεπτα, πάνω στη διαδρομή."""
    points = []
    for _ in range(laps):
        for (x0, y0), (x1, y1) in zip(route[:-1], route[1:]):
            length = math.hypot(x1 - x0, y1 - y0)
            yaw = math.degrees(math.atan2(y1 - y0, x1 - x0))
            for s in np.arange(0, length, speed * dt):
                points.append((x0 + (x1 - x0) * s / length, y0 + (y1 - y0) * s / length, yaw))
    return points


def load_map():
    """Το occupancy grid, γυρισμένο σε συντεταγμένες CARLA (y_carla = -y_opendrive)."""
    grid = np.load("town10_grid.npy")
    x0, y0, res = np.load("town10_meta.npy")
    h, w = grid.shape
    extent = [x0, x0 + w * res, -(y0 + h * res), -y0]
    return np.flipud(grid), extent


if __name__ == "__main__":
    noise = GpsNoise(zones=TEST_ZONES, dt=DT, seed=42)
    truth = drive(ROUTE, SPEED, DT, LAPS)

    rows = []   # (t, x, y, yaw, zone, gps ή None)
    for i, (x, y, yaw) in enumerate(truth):
        zone = zone_at(x, y, TEST_ZONES).get("name", "open sky")
        rows.append((i * DT, x, y, yaw, zone, noise.step(x, y, yaw)))

    # Σφάλμα κατά μήκος / εγκάρσια για κάθε δείγμα με σήμα
    t_ok, err_along, err_cross, zones_ok = [], [], [], []
    for t, x, y, yaw, zone, gps in rows:
        if gps is None:
            continue
        dx, dy = gps[0] - x, gps[1] - y
        r = math.radians(yaw)
        t_ok.append(t)
        err_along.append(dx * math.cos(r) + dy * math.sin(r))
        err_cross.append(-dx * math.sin(r) + dy * math.cos(r))
        zones_ok.append(zone)
    err_along, err_cross, zones_ok = map(np.array, (err_along, err_cross, zones_ok))

    print(f"{len(rows)} δείγματα, {len(rows) - len(t_ok)} χωρίς σήμα\n")
    print(f"{'ζώνη':<10} {'δείγματα':>9} {'RMS κατά μήκος':>15} {'RMS εγκάρσια':>13} {'RMS 2D':>8} {'max 2D':>8}")
    for name in ["open sky", "canyon"]:
        m = zones_ok == name
        e2d = np.hypot(err_along[m], err_cross[m])
        print(f"{name:<10} {m.sum():>9} {np.sqrt(np.mean(err_along[m]**2)):>13.1f} m"
              f" {np.sqrt(np.mean(err_cross[m]**2)):>11.1f} m {np.sqrt(np.mean(e2d**2)):>6.1f} m {e2d.max():>6.1f} m")

    # --- Γραφήματα ---
    fig, (ax_map, ax_err) = plt.subplots(1, 2, figsize=(18, 8), gridspec_kw=dict(width_ratios=[1, 1.4]))

    grid, extent = load_map()
    ax_map.imshow(grid, cmap="gray", origin="lower", extent=extent, alpha=0.35)
    for z in TEST_ZONES:
        ax_map.add_patch(plt.Rectangle((z["x_min"], z["y_min"]), z["x_max"] - z["x_min"], z["y_max"] - z["y_min"],
                                       color="tab:red" if z["name"] == "canyon" else "tab:purple", alpha=0.2))
    ax_map.plot([r[1] for r in rows], [r[2] for r in rows], "k-", lw=1.5, label="αληθινή διαδρομή")
    gx = [r[5][0] for r in rows if r[5]]
    gy = [r[5][1] for r in rows if r[5]]
    ax_map.plot(gx, gy, ".", color="tab:blue", ms=3, label="GPS με θόρυβο")
    ax_map.set_xlabel("CARLA x (m)")
    ax_map.set_ylabel("CARLA y (m)")
    ax_map.set_title("Χάρτης (κόκκινο: ζώνη canyon, μωβ: χωρίς σήμα)")
    ax_map.legend(loc="lower right")
    ax_map.set_aspect("equal")

    for t, *_, zone, _gps in rows:
        if zone == "canyon":
            ax_err.axvspan(t, t + DT, color="tab:red", alpha=0.08, lw=0)
        elif zone == "dropout":
            ax_err.axvspan(t, t + DT, color="tab:purple", alpha=0.25, lw=0)
    ax_err.plot(t_ok, err_cross, ".-", ms=2, lw=0.7, label="εγκάρσια")
    ax_err.plot(t_ok, err_along, ".-", ms=2, lw=0.7, label="κατά μήκος")
    ax_err.axhline(0, color="k", lw=0.5)
    ax_err.set_xlabel("χρόνος (s)")
    ax_err.set_ylabel("σφάλμα (m)")
    ax_err.set_title("Σφάλμα στον χρόνο (κόκκινο φόντο: canyon, μωβ: χωρίς σήμα)")
    ax_err.legend()

    plt.tight_layout()
    plt.savefig("gps_noise_test.png", dpi=110)
    print("\nΑποθηκεύτηκε: gps_noise_test.png")
