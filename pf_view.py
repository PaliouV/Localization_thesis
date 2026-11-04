"""
Ζωντανή οπτικοποίηση του particle_filter.py πάνω στον χάρτη του Town10 (το καλεί το --show).

Δύο παράθυρα δίπλα δίπλα:
  αριστερά: όλος ο χάρτης
  δεξιά:    ζουμ (ZOOM_M μέτρα) που ακολουθεί το αυτοκίνητο
Σε κάθε βήμα ζωγραφίζει: το αληθινό αυτοκίνητο, την τελευταία μέτρηση GPS, την εκτίμηση του φίλτρου
και τα particles.

Φόντο: town10/town10_minimap.png (κάτοψη από το no_rendering_mode.py του CARLA), με τα νούμερα του
town10_minimap.json για τη μετατροπή μέτρα → pixels:  pixel = pixels_per_meter · (μέτρα − min).
"""

import json
import os

import matplotlib.pyplot as plt
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
MAP_PNG = os.path.join(HERE, "town10", "town10_minimap.png")
MAP_JSON = os.path.join(HERE, "town10", "town10_minimap.json")

ZOOM_M = 60          # m: πόσο μεγάλο είναι το δεξί παράθυρο
OVERVIEW_STEP = 4    # το αριστερό παράθυρο δείχνει κάθε 4ο pixel (πιο γρήγορο, αρκεί για όλο τον χάρτη)
NO_GPS_AFTER = 1.5   # s: αν η τελευταία μέτρηση GPS είναι πιο παλιά, το GPS «χάθηκε» (π.χ. γέφυρα)


def open_view(receiver):
    """Ανοίγει το παράθυρο και επιστρέφει ένα λεξικό με ό,τι χρειάζεται το update_view."""
    meta = json.load(open(MAP_JSON))
    image = plt.imread(MAP_PNG)
    ppm = meta["pixels_per_meter"]
    min_x, min_y = meta["min_x"], meta["min_y"]
    height, width = image.shape[:2]
    # extent = [αριστερά, δεξιά, κάτω, πάνω] σε μέτρα. Το y του CARLA μεγαλώνει προς τα κάτω στην εικόνα.
    full_extent = [min_x, min_x + width / ppm, min_y + height / ppm, min_y]

    plt.ion()
    fig, (ax_map, ax_zoom) = plt.subplots(1, 2, figsize=(14, 7))
    fig.suptitle(f"Particle filter, δέκτης {receiver}")
    ax_map.imshow(image[::OVERVIEW_STEP, ::OVERVIEW_STEP], extent=full_extent)
    zoom_image = ax_zoom.imshow(image[:2, :2], extent=full_extent)   # αλλάζει σε κάθε βήμα
    for ax in (ax_map, ax_zoom):
        ax.set_xlabel("x (m)")
        ax.set_ylabel("y (m)")

    view = {"fig": fig, "ax_map": ax_map, "ax_zoom": ax_zoom, "image": image, "zoom_image": zoom_image,
            "ppm": ppm, "min_x": min_x, "min_y": min_y}
    for name, ax in (("map", ax_map), ("zoom", ax_zoom)):
        view[name + "_particles"] = ax.scatter([], [], s=2, c="orange", alpha=0.4, label="particles")
        view[name + "_gps"] = ax.scatter([], [], s=80, c="red", marker="x", label="GPS")
        view[name + "_truth"] = ax.scatter([], [], s=60, c="lime", edgecolors="black", label="αλήθεια")
        view[name + "_estimate"] = ax.scatter([], [], s=60, c="blue", edgecolors="white", label="φίλτρο")
    ax_zoom.legend(loc="upper right")
    plt.show()
    return view


def update_view(view, particles, truth, estimate, last_fix, last_fix_time, time):
    """Ζωγραφίζει ένα βήμα. Επιστρέφει False αν ο χρήστης έκλεισε το παράθυρο."""
    if not plt.fignum_exists(view["fig"].number):
        return False
    gps_lost = last_fix is None or time - last_fix_time > NO_GPS_AFTER

    for name in ("map", "zoom"):
        view[name + "_particles"].set_offsets(particles[:, :2])
        view[name + "_truth"].set_offsets([truth])
        view[name + "_estimate"].set_offsets([estimate])
        if last_fix is not None:
            view[name + "_gps"].set_offsets([last_fix])
            view[name + "_gps"].set_color("gray" if gps_lost else "red")   # γκρι = παλιά μέτρηση, χωρίς σήμα

    # Δεξί παράθυρο: κόβουμε από τον χάρτη μόνο το κομμάτι γύρω από το αυτοκίνητο (πιο γρήγορο από όλη την εικόνα)
    x0, x1 = truth[0] - ZOOM_M / 2, truth[0] + ZOOM_M / 2
    y0, y1 = truth[1] - ZOOM_M / 2, truth[1] + ZOOM_M / 2
    ppm, min_x, min_y = view["ppm"], view["min_x"], view["min_y"]
    c0, c1 = int(ppm * (x0 - min_x)), int(ppm * (x1 - min_x))
    r0, r1 = int(ppm * (y0 - min_y)), int(ppm * (y1 - min_y))
    view["zoom_image"].set_data(view["image"][r0:r1, c0:c1])
    view["zoom_image"].set_extent([min_x + c0 / ppm, min_x + c1 / ppm, min_y + r1 / ppm, min_y + r0 / ppm])
    view["ax_zoom"].set_xlim(x0, x1)
    view["ax_zoom"].set_ylim(y1, y0)

    filter_error = np.hypot(estimate[0] - truth[0], estimate[1] - truth[1])
    gps_text = "χωρίς σήμα" if gps_lost else f"{np.hypot(last_fix[0] - truth[0], last_fix[1] - truth[1]):.1f} m"
    view["ax_zoom"].set_title(f"t = {time:.1f} s   |   λάθος φίλτρου {filter_error:.1f} m   |   λάθος GPS {gps_text}")

    view["fig"].canvas.draw_idle()
    plt.pause(0.001)
    return True
