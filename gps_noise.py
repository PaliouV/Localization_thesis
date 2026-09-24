"""
Μοντέλο θορύβου GPS για urban canyon, σε μέτρα x/y του CARLA.

    σφάλμα(t) = αργό_bias(t) + λευκός_θόρυβος(t) + άλμα_NLOS(t)

Όλες οι τιμές έρχονται από το GPS_NOISE_MODEL.md (§7 ΤΕΛΙΚΟ ΜΟΝΤΕΛΟ, §9 ζώνες).
Οι ετικέτες [G], [S4], [S5], [GOV] είναι οι πηγές εκεί.
Ό,τι σημειώνεται ΠΑΡΑΔΟΧΗ δεν είναι δημοσιευμένη τιμή — δηλώνεται στη διπλωματική.

Χρήση (μία φορά ανά δείγμα GPS):
    noise = GpsNoise(zones=ZONES, dt=1.0)
    fix = noise.step(x, y, yaw_deg)   # (x_noisy, y_noisy) ή None αν δεν υπάρχει σήμα
"""

import math
import numpy as np

# --- Ανοιχτός ουρανός: ισχύει όπου δεν υπάρχει ζώνη -----------------------
OPEN_SKY = dict(
    sigma_along=2.5,   # m — bias κατά μήκος του δρόμου, [GOV] ~3 m (95%)
    sigma_cross=2.5,   # m — ίδιο εγκάρσια: χωρίς κτήρια δεν υπάρχει ανισοτροπία
    nlos_rate=0.0,     # άλματα ανά δευτερόλεπτο — χωρίς κτήρια, κανένα NLOS
    dropout=False,     # υπάρχει σήμα
)

# --- Σταθερές κοινές για όλες τις ζώνες -----------------------------------
TAU = 30.0             # s — χρονική σταθερά του bias. ΠΑΡΑΔΟΧΗ: 10-60 s από [S5] §4.3
WHITE_SIGMA = 1.0      # m — λευκός θόρυβος (γνήσιο multipath), [G] UERE
NLOS_SHAPE = 2.5       # σχήμα Γάμμα, [S5] Πίν. 4 (γ = 2.40-2.62)
NLOS_SCALE = 8.0       # m — ΠΑΡΑΔΟΧΗ: μέσο ~20 m, ώστε τα άλματα να πέφτουν στο 4-49 m του [S4] Πίν. 7-9
NLOS_DURATION = (2.0, 10.0)   # s — ΠΑΡΑΔΟΧΗ: όσο περνάς δίπλα από ένα κτήριο
REACQUIRE = (1.0, 5.0)        # s — επανακλείδωμα μετά από απώλεια, κατασκευαστές δεκτών

# Οι ζώνες ορίζονται με το χέρι (GPS_NOISE_MODEL.md §9). Κάθε ζώνη:
#   dict(x_min, x_max, y_min, y_max, sigma_along, sigma_cross, nlos_rate, dropout)
# Η πρώτη ζώνη που περιέχει το σημείο κερδίζει — γι' αυτό η σειρά μετράει.
#
# Τα όρια (μέτρα CARLA, Town10HD) διαλέχτηκαν πάνω στον χάρτη town10_canyon.png
# (canyon_map.py, από τον χάρτη κτηρίων LiDAR). Στα σχόλια: μέσο % του ορίζοντα που
# κρύβουν τα κτήρια πάνω από 15°.
#
# Τιμές σ: «ισχυρό» = canyon του GPS_NOISE_MODEL.md §7 ([S4], [G]).
# «Μέτριο» και «ήπιο» = ΠΑΡΑΔΟΧΗ: ενδιάμεσες τιμές ανάμεσα σε ανοιχτό ουρανό και canyon.
# nlos_rate = ΠΑΡΑΔΟΧΗ παντού: δεν βρέθηκε δημοσιευμένος ρυθμός (GPS_NOISE_MODEL.md §3).
STRONG = dict(sigma_along=12.0, sigma_cross=25.0, nlos_rate=1 / 30, dropout=False)
MODERATE = dict(sigma_along=7.0, sigma_cross=13.0, nlos_rate=1 / 60, dropout=False)
MILD = dict(sigma_along=4.0, sigma_cross=6.0, nlos_rate=1 / 180, dropout=False)
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


class GpsNoise:
    def __init__(self, zones=ZONES, dt=1.0, seed=None):
        self.zones = zones
        self.dt = dt
        self.rng = np.random.default_rng(seed)
        self.alpha = math.exp(-dt / TAU)   # πόσο "θυμάται" το bias από το προηγούμενο δείγμα

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
        #    καινούργιο κομμάτι έχει την ανισοτροπία του δρόμου (εγκάρσια > κατά μήκος, [G]).
        #    Όταν αλλάζει ζώνη, το bias προσαρμόζεται σταδιακά (~TAU), όχι απότομα.
        n_along, n_cross = self.rng.standard_normal(2)
        fresh = zone["sigma_along"] * n_along * along + zone["sigma_cross"] * n_cross * cross
        self.bias = self.alpha * self.bias + math.sqrt(1 - self.alpha**2) * fresh

        # 2. Λευκός θόρυβος — καινούργιος σε κάθε δείγμα, ίδιος προς όλες τις κατευθύνσεις.
        white = WHITE_SIGMA * self.rng.standard_normal(2)

        error = self.bias + white

        # 3. Άλμα NLOS — σπάνιο, μεγάλο, κρατάει λίγα δευτερόλεπτα.
        if self.nlos_left > 0:
            self.nlos_left -= self.dt
        elif self.rng.random() < zone["nlos_rate"] * self.dt:
            size = self.rng.gamma(NLOS_SHAPE, NLOS_SCALE)
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
