"""
Μοντέλο θορύβου GPS, έκδοση 2 (υπό κατασκευή στο branch gps-noise-v2).

Δύο δέκτες οχήματος, με νούμερα από τα πραγματικά δεδομένα του UrbanNav (gnss_sim/):
  "f9p" — σύγχρονος (u-blox F9P): μόνο αργό bias, χωρίς πηδήματα
  "m8t" — παλιός (u-blox M8T): αργό bias + απότομα σκαλοπάτια + καθυστέρηση («σέρνεται» πίσω)
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
#   lag       σταθερά χρόνου (s) με την οποία η θέση του δέκτη «κυνηγάει» την πραγματική
#             (μόνο M8T, μόνο στο "moving": είναι ιδιότητα του δέκτη, χρησιμοποιείται και στη στάση)
# Κοινά λεξικά = κοινές τιμές, όπου τα δεδομένα δεν δείχνουν διαφορά (αποφάσεις 16 Οκτ 2026).

F9P_IDENTICAL_ZONES = {
    "stopped" : {
        "sigma" : None,
        "tau" : None,
        "ratio" : None
    },
    "moving" : {
        "sigma" : None,
        "tau" : None,
        "ratio" : None
    }
}

M8T_IDENTICAL_MOVING = {
    "sigma" : None,
    "tau" : None,
    "ratio" : None,
    "step_rate" : None,
    "step_size" : None,
    "lag" : None
}


RECEIVERS = {
    "f9p" : {
        "MILD" : F9P_IDENTICAL_ZONES,
        "MODERATE" : F9P_IDENTICAL_ZONES,
        "STRONG" : {
            "stopped" : {
                "sigma" : None,
                "tau" : None,
                "ratio" : None
                },
                "moving" : {
                    "sigma" : None,
                    "tau" : None,
                    "ratio" : None
                    },
        }
    },
    "m8t" : {
        "MILD" : {
            "stopped" : {
                "sigma" : None,
                "tau" : None,
                "ratio" : None,
                "step_rate" : None,
                "step_size" : None
            },
            "moving" : M8T_IDENTICAL_MOVING,
        },
        "MODERATE" : {
            "stopped" : {
                "sigma" : None,
                "tau" : None,
                "ratio" : None,
                "step_rate" : None,
                "step_size" : None
            },
            "moving" : M8T_IDENTICAL_MOVING,
        },
        "STRONG" : {
            "stopped" : {
                "sigma" : None,
                "tau" : None,
                "ratio" : None,
                "step_rate" : None,
                "step_size" : None
            },
            "moving" : {
                "sigma" : None,
                "tau" : None,
                "ratio" : None,
                "step_rate" : None,
                "step_size" : None,
                "lag" : None
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
        self.lag_pos = None           # η θέση που «κυνηγάει» την πραγματική (M8T)· None = χωρίς ιστορικό
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
        if "step_rate" in cell and self.rng.random() < cell["step_rate"] * self.dt:
            size = self.rng.exponential(cell["step_size"])
            d = self.rng.standard_normal() * along + cell["ratio"] * self.rng.standard_normal() * cross
            self.bias = self.bias + size * d / np.linalg.norm(d)

        # 3. Καθυστέρηση (μόνο M8T): η θέση του δέκτη «κυνηγάει» την πραγματική με σταθερά χρόνου lag.
        #    Σε σταθερή ταχύτητα v μένει πίσω κατά ~v·lag· σε στάση πλησιάζει σιγά-σιγά την πραγματική.
        lag = self.profile[level]["moving"].get("lag")
        if lag:
            if self.lag_pos is None:
                self.lag_pos = truth.copy()
            else:
                self.lag_pos = self.lag_pos + (1 - math.exp(-self.dt / lag)) * (truth - self.lag_pos)
            base = self.lag_pos
        else:
            base = truth

        fix = base + self.bias
        return fix[0], fix[1]
