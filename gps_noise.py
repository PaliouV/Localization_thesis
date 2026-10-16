"""
Μοντέλο θορύβου GPS για urban canyon, σε μέτρα x/y του CARLA.

    σφάλμα(t) = αργό_bias(t) + λευκός_θόρυβος(t) + άλμα_NLOS(t)

Τεκμηρίωση στο GPS_NOISE_MODEL.md: παράμετροι §4, ζώνες §5, ρύθμιση §6, λόγος και TAU §7.
ΠΑΡΑΔΟΧΗ = δική μας επιλογή χωρίς πηγή. ΡΥΘΜΙΣΗ = τιμή ρυθμισμένη ώστε το σφάλμα
θέσης να ταιριάζει με πραγματικές μετρήσεις (§6, calibrate_gps_noise.py).

Χρήση (μία φορά ανά δείγμα GPS):
    noise = GpsNoise(zones=ZONES, dt=1.0)                    # βάση: λόγος 2, TAU 30 s
    noise = GpsNoise(zones=ZONES, dt=1.0, ratio=1.2, tau=10) # άλλο τρέξιμο, χωρίς αλλαγή στο αρχείο
    fix = noise.step(x, y, yaw_deg)   # (x_noisy, y_noisy) ή None αν δεν υπάρχει σήμα
"""

import math
import numpy as np

# --- Ανοιχτός ουρανός: ισχύει όπου δεν υπάρχει ζώνη -----------------------
OPEN_SKY = dict(
    sigma_along=2.5,   # m — ΠΑΡΑΔΟΧΗ (GPS.gov: smartphones ~4.9 m, "typically")
    sigma_cross=2.5,   # m — ίδιο εγκάρσια: χωρίς κτήρια δεν υπάρχει ανισοτροπία
    nlos_rate=0.0,     # άλματα ανά δευτερόλεπτο — χωρίς κτήρια, κανένα NLOS
    dropout=False,     # υπάρχει σήμα
)

# --- Σταθερές κοινές για όλες τις ζώνες -----------------------------------
TAU = 30.0             # s — χρονική σταθερά του bias (προεπιλογή). ΠΑΡΑΔΟΧΗ — δεν βρέθηκε πηγή για κινούμενο όχημα.
                       #     Δοκιμάζεται με 10 / 30 / 120 s (§7β): δίνεται απ' έξω, GpsNoise(..., tau=...)
RATIO = 2.0            # λόγος σ εγκάρσια / σ κατά μήκος (προεπιλογή). Ποιοτικά [G], Wang 2012 (προσομοιώσεις):
                       #     εγκάρσια χειρότερο, χωρίς αριθμό. Suhr 2017 (πραγματικά): ~1.16. Δοκιμάζεται με
                       #     1.2 και 2 (§7α): δίνεται απ' έξω, GpsNoise(..., ratio=...). Ένας για όλες τις ζώνες.
WHITE_SIGMA = 1.0      # m — λευκός θόρυβος. ΠΑΡΑΔΟΧΗ
NLOS_SHAPE = 2.5       # σχήμα της κατανομής Γάμμα για το μέγεθος των αλμάτων. ΠΑΡΑΔΟΧΗ
NLOS_DURATION = (2.0, 10.0)   # s — πόσο κρατάει ένα άλμα. ΠΑΡΑΔΟΧΗ
REACQUIRE = (1.0, 5.0)        # s — επανακλείδωμα μετά από απώλεια σήματος. ΠΑΡΑΔΟΧΗ

# Οι ζώνες ορίζονται με το χέρι (GPS_NOISE_MODEL.md §5). Κάθε ζώνη:
#   dict(x_min, x_max, y_min, y_max, sigma, nlos_rate, nlos_scale, dropout)
#   sigma      = συνολικό σ του αργού bias (m). Ο λόγος (RATIO) το μοιράζει σε κατά μήκος /
#                εγκάρσια, με το ίδιο συνολικό: σ_κατά_μήκος² + σ_εγκάρσια² = sigma²
#   nlos_rate  = πόσα άλματα NLOS ανά δευτερόλεπτο
#   nlos_scale = κλίμακα (m) της Γάμμα για το μέγεθος του άλματος· μέσο άλμα = NLOS_SHAPE × nlos_scale
# Η πρώτη ζώνη που περιέχει το σημείο κερδίζει — γι' αυτό η σειρά μετράει.
#
# Τα όρια (μέτρα CARLA, Town10HD) διαλέχτηκαν πάνω στον χάρτη town10/town10_canyon.png
# (town10/canyon_map.py, από τον χάρτη κτηρίων LiDAR). Στα σχόλια: μέσο % του ορίζοντα που
# κρύβουν τα κτήρια πάνω από 15°.
#
# sigma, nlos_rate, nlos_scale: ΡΥΘΜΙΣΗ με πραγματικές μετρήσεις: STRONG ↔ [S4] Πίν. 2, 4,
# MODERATE / MILD ↔ Suhr 2017 Πίν. I–II (GPS_NOISE_MODEL.md §6). Ποια μέτρηση ταιριάζει
# με ποια ζώνη είναι ΠΑΡΑΔΟΧΗ (§6β). Έλεγχος: python calibrate_gps_noise.py
STRONG = dict(sigma=12.0, nlos_rate=1 / 10, nlos_scale=22.0, dropout=False)
MODERATE = dict(sigma=10.0, nlos_rate=1 / 30, nlos_scale=16.0, dropout=False)
MILD = dict(sigma=2.0, nlos_rate=1 / 30, nlos_scale=8.0, dropout=False)
NO_SIGNAL = dict(sigma_along=2.5, sigma_cross=2.5, nlos_rate=0.0, dropout=True)

ZONES = [
    dict(name="bridge", x_min=0, x_max=30, y_min=128, y_max=145, **NO_SIGNAL),             # γέφυρα πάνω από τον νότιο δρόμο
    dict(name="north road", x_min=-120, x_max=115, y_min=-80, y_max=-45, **STRONG),        # 83% — ουρανοξύστες 55-154 m
    dict(name="NW corner + west road", x_min=-120, x_max=-85, y_min=-80, y_max=25, **STRONG),  # 80%
    dict(name="inner road y~65", x_min=-50, x_max=90, y_min=58, y_max=75, **STRONG),       # 81%
    dict(name="SW corner + south road", x_min=-120, x_max=-40, y_min=85, y_max=145, **MILD),   # 45% — προς τη θάλασσα
    dict(name="rest of town", x_min=-130, x_max=125, y_min=-85, y_max=155, **MODERATE),    # 71%
]


def zone_at(x, y, zones):
    """Σε ποια ζώνη είναι το σημείο (x, y). Αν σε καμία, ανοιχτός ουρανός.

    Αυτή είναι η μόνη συνάρτηση που αλλάζει αν αργότερα ο θόρυβος
    υπολογίζεται από τα κτήρια αντί για ζώνες με το χέρι.
    """
    for z in zones:
        if z["x_min"] <= x <= z["x_max"] and z["y_min"] <= y <= z["y_max"]:
            return z
    return OPEN_SKY


def zone_sigmas(zone, ratio):
    """σ κατά μήκος και σ εγκάρσια μιας ζώνης.

    Ζώνη με ένα συνολικό "sigma": ο λόγος το μοιράζει, κρατώντας ίδιο το συνολικό
    (σ_κατά_μήκος² + σ_εγκάρσια² = sigma²), ώστε το μέγεθος του λάθους να μένει
    περίπου ίδιο με κάθε λόγο και να αλλάζει μόνο η κατεύθυνσή του.
    Ζώνη με δικά της sigma_along / sigma_cross (OPEN_SKY, NO_SIGNAL): μένουν όπως είναι.
    """
    if "sigma" in zone:
        along = zone["sigma"] / math.sqrt(1 + ratio**2)
        return along, ratio * along
    return zone["sigma_along"], zone["sigma_cross"]


class GpsNoise:
    def __init__(self, zones=ZONES, dt=1.0, seed=None, ratio=RATIO, tau=TAU):
        self.zones = zones
        self.dt = dt
        self.ratio = ratio
        self.tau = tau
        self.rng = np.random.default_rng(seed)
        self.alpha = math.exp(-dt / tau)   # πόσο "θυμάται" το bias από το προηγούμενο δείγμα

        self.bias = np.zeros(2)            # αργό bias σε x/y, μέτρα
        self.nlos_offset = np.zeros(2)     # τρέχον άλμα NLOS σε x/y, μέτρα
        self.nlos_left = 0.0               # πόσα δευτερόλεπτα μένουν στο άλμα
        self.no_fix_left = 0.0             # πόσα δευτερόλεπτα μένουν χωρίς σήμα

    def step(self, x, y, yaw_deg):
        """Ένα δείγμα GPS. x, y: αληθινή θέση (m). yaw_deg: προσανατολισμός οχήματος.

        Επιστρέφει (x_noisy, y_noisy), ή None όταν δεν υπάρχει σήμα.
        """
        zone = zone_at(x, y, self.zones)

        # Κατευθύνσεις του δρόμου σε x/y του CARLA.
        # Υπόθεση: ο δρόμος έχει την κατεύθυνση του οχήματος.
        yaw = math.radians(yaw_deg)
        along = np.array([math.cos(yaw), math.sin(yaw)])
        cross = np.array([-math.sin(yaw), math.cos(yaw)])

        # 1. Αργό bias — Gauss-Markov 1ης τάξης. Κάθε δείγμα κρατάει το α του
        #    προηγούμενου και προσθέτει λίγο καινούργιο. Ο παράγοντας sqrt(1-α²)
        #    κρατάει την τυπική απόκλιση ίση με sigma μακροπρόθεσμα.
        #    Το bias ζει σε x/y (δεν γυρίζει όταν στρίβει το όχημα)· μόνο το
        #    καινούργιο κομμάτι έχει την ανισοτροπία του δρόμου: εγκάρσια > κατά μήκος
        #    ποιοτικά [G], Wang 2012 (προσομοιώσεις)· ο λόγος είναι παράμετρος (self.ratio).
        #    Όταν αλλάζει ζώνη, το bias προσαρμόζεται σταδιακά (~tau), όχι απότομα.
        sigma_along, sigma_cross = zone_sigmas(zone, self.ratio)
        n_along, n_cross = self.rng.standard_normal(2)
        fresh = sigma_along * n_along * along + sigma_cross * n_cross * cross
        self.bias = self.alpha * self.bias + math.sqrt(1 - self.alpha**2) * fresh

        # 2. Λευκός θόρυβος — καινούργιος σε κάθε δείγμα, ίδιος προς όλες τις κατευθύνσεις.
        white = WHITE_SIGMA * self.rng.standard_normal(2)

        error = self.bias + white

        # 3. Άλμα NLOS — σπάνιο, μεγάλο, κρατάει λίγα δευτερόλεπτα.
        if self.nlos_left > 0:
            self.nlos_left -= self.dt
        elif self.rng.random() < zone["nlos_rate"] * self.dt:
            size = self.rng.gamma(NLOS_SHAPE, zone["nlos_scale"])
            angle = self.rng.uniform(0, 2 * math.pi)
            self.nlos_offset = size * np.array([math.cos(angle), math.sin(angle)])
            self.nlos_left = self.rng.uniform(*NLOS_DURATION)
        if zone["nlos_rate"] == 0:        # βγήκες από τα κτήρια → το NLOS σταματά
            self.nlos_left = 0.0
        if self.nlos_left <= 0:
            self.nlos_offset = np.zeros(2)
        error = error + self.nlos_offset

        # 4. Απώλεια σήματος — μέσα σε ζώνη dropout δεν υπάρχει μέτρηση, και
        #    μετά την έξοδο ο δέκτης θέλει λίγα δευτερόλεπτα να ξανακλειδώσει.
        if zone["dropout"]:
            self.no_fix_left = self.rng.uniform(*REACQUIRE)
            return None
        if self.no_fix_left > 0:
            self.no_fix_left -= self.dt
            return None

        return x + error[0], y + error[1]
