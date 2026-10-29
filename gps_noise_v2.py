"""
Μοντέλο θορύβου GPS, έκδοση 2 (υπό κατασκευή στο branch gps-noise-v2).

Δύο δέκτες οχήματος, με νούμερα από τα πραγματικά δεδομένα του UrbanNav (gnss_sim/):
  "f9p" — σύγχρονος (u-blox F9P): μόνο αργό bias, χωρίς πηδήματα
  "m8t" — παλιός (u-blox M8T): αργό bias + απότομα σκαλοπάτια + «σέρνεται και προλαβαίνει»
Το σφάλμα εξαρτάται από τη ζώνη του χάρτη (MILD / MODERATE / STRONG / NO_SIGNAL) και από το
αν το όχημα είναι σταματημένο ή κινείται.

Χρήση (μία κλήση ανά δείγμα GPS):
    noise = GpsNoise("f9p", seed=1)
    fix = noise.step(x, y, yaw_deg, speed)   # (x_gps, y_gps) ή None όταν δεν υπάρχει θέση

Τα νούμερα (sigma, tau κ.λπ.) βγαίνουν από τη ρύθμιση: calibrate_gps_noise_v2.py.
ΠΑΡΑΔΟΧΕΣ σημειώνονται με αυτή τη λέξη.
"""

import math

import numpy as np

STOPPED_BELOW = 0.5     # m/s — κάτω από αυτό: σταματημένο (όριο της ανάλυσης των δεδομένων)
MOVING_ABOVE = 2.0      # m/s — πάνω από αυτό: κινείται· ανάμεσα κρατάει την προηγούμενη κατάσταση
REACQUIRE = (1.0, 5.0)  # s — χωρίς θέση μετά την έξοδο από NO_SIGNAL. ΠΑΡΑΔΟΧΗ (από το παλιό μοντέλο)

# Κάθε κελί (δέκτης → ζώνη → κατάσταση) έχει:
#   sigma     συνολικό μέγεθος του αργού bias (m)
#   tau       πόσο αργά αλλάζει το bias (s)
#   ratio     εγκάρσιο / κατά μήκος (σ)
#   step_rate σκαλοπάτια ανά δευτερόλεπτο (μόνο M8T)
#   step_size μέσο μέγεθος σκαλοπατιού (m) (μόνο M8T)
#   crawl         πόσο από την κίνηση του οχήματος ακολουθεί η θέση του δέκτη σε κάθε δείγμα (0–1)
#   approach      τι μέρος της απόστασης από την πραγματική θέση «κλείνει» σιγά-σιγά σε κάθε δείγμα (0–1)
#   catchup_rate  πόσες φορές ανά δευτερόλεπτο ο δέκτης «προλαβαίνει» απότομα την πραγματική θέση
#                 (τα τρία μόνο M8T, μόνο στο "moving": είναι ιδιότητα του δέκτη, ισχύουν και στη στάση)
# Κοινά λεξικά = κοινές τιμές, όπου τα δεδομένα δεν δείχνουν διαφορά (αποφάσεις 16 Οκτ 2026).

# F9P: τιμές από calibrate_gps_noise_v2.py (19 Οκτ 2026). Ταιριάζουν στο διάμεσο (±5%)· το μέσο
# βγαίνει 8–17% μικρότερο (ο πραγματικός έχει λίγο «βαρύτερη ουρά») και το μοντέλο κινείται λίγο
# περισσότερο σε 1 s και λιγότερο σε 10 s (όριο του απλού Gauss–Markov). Σε στάση, tau = όριο 3600 s:
# στις σύντομες στάσεις το bias σχεδόν μόνο «γλιστράει».
F9P_IDENTICAL_ZONES = {
    "stopped" : {
        "sigma" : 7.51,
        "tau" : 3600.0,
        "ratio" : 1.0
    },
    "moving" : {
        "sigma" : 3.20,
        "tau" : 174.0,
        "ratio" : 1.18
    }
}

# M8T: τιμές από calibrate_gps_noise_v2.py m8t, κοινή ρύθμιση στάσης + κίνησης (19 Οκτ 2026).
# Σε κίνηση ταιριάζουν το μέσο σφάλμα και το «πόσο μένει πίσω»· η διάμεσος βγαίνει μεγαλύτερη και
# το σφάλμα αλλάζει λιγότερο από το πραγματικό σε 10–60 s. Σε στάση MODERATE: πολλά περισσότερα
# άλματα και μικρότερο μέσο. Η στάση MILD δεν κρίνεται (πολύ λίγα δεδομένα στάσης). Στη στάση
# MODERATE το step_rate είναι στο κάτω όριο: πρακτικά χωρίς σκαλοπάτια.
M8T_IDENTICAL_MOVING = {
    "sigma" : 4.31,
    "tau" : 128.0,
    "ratio" : 0.77,
    "step_rate" : 0.0240,
    "step_size" : 12.0,
    "crawl" : 0.439,
    "approach" : 0.00810,
    "catchup_rate" : 0.0203
}


RECEIVERS = {
    "f9p" : {
        "MILD" : F9P_IDENTICAL_ZONES,
        "MODERATE" : F9P_IDENTICAL_ZONES,
        "STRONG" : {
            "stopped" : {
                "sigma" : 1.77,
                "tau" : 122.0,
                "ratio" : 1.0
            },
            "moving" : {
                "sigma" : 11.2,
                "tau" : 197.0,
                "ratio" : 0.68
            },
        }
    },
    "m8t" : {
        "MILD" : {
            "stopped" : {
                "sigma" : 5.77,
                "tau" : 9.23,
                "ratio" : 1.0,
                "step_rate" : 0.000323,
                "step_size" : 3.52
            },
            "moving" : M8T_IDENTICAL_MOVING,
        },
        "MODERATE" : {
            "stopped" : {
                "sigma" : 19.4,
                "tau" : 94.4,
                "ratio" : 1.0,
                "step_rate" : 0.00001,
                "step_size" : 14.9
            },
            "moving" : M8T_IDENTICAL_MOVING,
        },
        "STRONG" : {
            "stopped" : {
                "sigma" : 13.8,
                "tau" : 37.2,
                "ratio" : 1.0,
                "step_rate" : 0.000566,
                "step_size" : 29.4
            },
            "moving" : {
                "sigma" : 0.677,
                "tau" : 272.0,
                "ratio" : 0.66,
                "step_rate" : 0.0433,
                "step_size" : 16.2,
                "crawl" : 0.161,
                "approach" : 0.00175,
                "catchup_rate" : 0.00200
            },
        }
    }
}
ZONES = [
    {
        "name" : "bridge",
        "x_min" : 0,
        "x_max" : 30,
        "y_min" : 128,
        "y_max" : 145,
        "level" : "NO_SIGNAL"
    },
    {
        "name" : "north road",
        "x_min" : -120,
        "x_max" : 115,
        "y_min" : -80,
        "y_max" : -45,
        "level" : "STRONG"
    },

    {
        "name" : "NW corner + west road",
        "x_min" : -120,
        "x_max" : -85,
        "y_min" : -80,
        "y_max" : 25,
        "level" : "STRONG"
    },

    {
        "name" : "inner road y~65",
        "x_min" : -50,
        "x_max" : 90,
        "y_min" : 58,
        "y_max" : 75,
        "level" : "STRONG"
    },

    {
        "name" : "SW corner + south road",
        "x_min" : -120,
        "x_max" : -40,
        "y_min" : 85,
        "y_max" : 145,
        "level" : "MILD"
    },

    {
        "name" : "rest of town",
        "x_min" : -130,
        "x_max" : 125,
        "y_min" : -85,
        "y_max" : 155,
        "level" : "MODERATE"
    }
]

def zone_level(x, y, zones=ZONES):
    for zone in zones:
        if ((zone["x_min"] <= x <=zone["x_max"]) and (zone["y_min"] <= y <= zone["y_max"])):
            return zone["level"]
    raise ValueError(f"Το σημείο ({x}, {y}) δεν είναι σε καμία ζώνη")


def motion_state(speed, previous):
    if speed < STOPPED_BELOW:
        current_state = "stopped"
        return current_state
    elif speed > MOVING_ABOVE:
        current_state = "moving"
        return current_state
    else: return previous


class GpsNoise:
    def __init__(self, receiver, seed=None, zones=ZONES, dt=1.0):
        self.profile = RECEIVERS[receiver]
        self.state = "stopped"
        self.rng = np.random.default_rng(seed)
        self.zones = zones
        self.dt = dt

        self.bias = np.zeros(2)       # αργό bias σε x/y (m)· εδώ μπαίνουν και τα σκαλοπάτια
        self.lag_pos = None           # η θέση που «σέρνεται» πίσω από την πραγματική (M8T)· None = χωρίς ιστορικό
        self.prev_truth = None        # πραγματική θέση του προηγούμενου δείγματος (για την κίνηση του οχήματος)
        self.no_fix_left = 0.0        # δευτερόλεπτα που μένουν χωρίς θέση (μετά από NO_SIGNAL)

    def step(self, x, y, yaw_deg, speed):
        """Ένα δείγμα GPS. x, y: πραγματική θέση (m)· yaw_deg: κατεύθυνση οχήματος· speed: m/s.

        Επιστρέφει (x_gps, y_gps), ή None όταν ο δέκτης δεν δίνει θέση.
        """
        self.state = motion_state(speed, self.state)
        level = zone_level(x, y, self.zones)

        # Απώλεια σήματος: κάτω από τη γέφυρα δεν υπάρχει θέση, και μετά την έξοδο ο δέκτης θέλει
        # λίγα δευτερόλεπτα να ξαναβρεί θέση. Όταν τη βρει, ξεκινάει από την αρχή (χωρίς ιστορικό).
        if level == "NO_SIGNAL":
            self.no_fix_left = self.rng.uniform(*REACQUIRE)
            self.lag_pos = None
            self.prev_truth = None
            return None
        if self.no_fix_left > 0:
            self.no_fix_left -= self.dt
            return None

        cell = self.profile[level][self.state]
        truth = np.array([x, y], dtype=float)

        # Κατευθύνσεις του δρόμου (υπόθεση: ο δρόμος έχει την κατεύθυνση του οχήματος)
        yaw = math.radians(yaw_deg)
        along = np.array([math.cos(yaw), math.sin(yaw)])
        cross = np.array([-math.sin(yaw), math.cos(yaw)])

        # 1. Αργό bias (Gauss–Markov 1ης τάξης). Κάθε δείγμα κρατάει το alpha του προηγούμενου και
        #    προσθέτει λίγο καινούργιο. Το καινούργιο έχει την ανισοτροπία του δρόμου: ο λόγος μοιράζει
        #    το sigma σε κατά μήκος / εγκάρσια, με ίδιο συνολικό (σ_κμ² + σ_εγκ² = sigma²).
        alpha = math.exp(-self.dt / cell["tau"])
        s_along = cell["sigma"] / math.sqrt(1 + cell["ratio"] ** 2)
        s_cross = cell["ratio"] * s_along
        n_along, n_cross = self.rng.standard_normal(2)
        fresh = s_along * n_along * along + s_cross * n_cross * cross
        self.bias = alpha * self.bias + math.sqrt(1 - alpha ** 2) * fresh

        # 2. Σκαλοπάτια (μόνο M8T): σπάνιο, απότομο άλμα που ΜΕΝΕΙ μέσα στο bias και σβήνει αργά
        #    μαζί του. Μέγεθος: εκθετική κατανομή με μέσο step_size (ΠΑΡΑΔΟΧΗ το σχήμα της).
        #    Κατεύθυνση: τυχαία, με τον ίδιο λόγο εγκάρσιο / κατά μήκος.
        #    Οι τυχαίοι αριθμοί τραβιούνται σε ΚΑΘΕ δείγμα (και όταν δεν γίνεται άλμα), ώστε το ίδιο
        #    seed να δίνει την ίδια σειρά, ό,τι κι αν είναι οι παράμετροι (χρειάζεται για τη ρύθμιση).
        if "step_rate" in cell:
            u, unit_size, n_a, n_c = self.rng.random(), self.rng.exponential(), *self.rng.standard_normal(2)
            if u < cell["step_rate"] * self.dt:
                d = n_a * along + cell["ratio"] * n_c * cross
                self.bias = self.bias + cell["step_size"] * unit_size * d / np.linalg.norm(d)

        # 3. «Σέρνεται, πλησιάζει και προλαβαίνει» (μόνο M8T). Όπως στα δεδομένα: η θέση του δέκτη
        #    ακολουθεί μόνο ένα μέρος (crawl) της κίνησης του οχήματος, οπότε μένει πίσω (ράμπα)· κλείνει
        #    σιγά-σιγά ένα μέρος (approach) της απόστασης, και έτσι σε στάση πλησιάζει την πραγματική θέση·
        #    κάπου-κάπου «προλαβαίνει» απότομα (catchup_rate). Ειδικές περιπτώσεις: approach = 0 → μόνο
        #    ράμπες· crawl = 0 → απλός «αργός ακόλουθος». Ένας τυχαίος αριθμός σε κάθε δείγμα.
        moving_cell = self.profile[level]["moving"]
        if "crawl" in moving_cell:
            catch = self.rng.random()
            if self.lag_pos is None or self.prev_truth is None:
                self.lag_pos = truth.copy()
            else:
                self.lag_pos = (self.lag_pos + moving_cell["crawl"] * (truth - self.prev_truth)
                                + moving_cell["approach"] * (truth - self.lag_pos))
                if catch < moving_cell["catchup_rate"] * self.dt:
                    self.lag_pos = truth.copy()
            base = self.lag_pos
        else:
            base = truth
        self.prev_truth = truth

        fix = base + self.bias
        return fix[0], fix[1]
