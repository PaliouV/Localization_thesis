"""
«Ψεύτικη» κάμερα λωρίδων (βήμα 3, 9 Νοε 2026): είναι κομμάτι του «κόσμου», όχι του φίλτρου.

Τι κάνει: στην ΑΛΗΘΙΝΗ θέση του αυτοκινήτου, βρίσκει από τον χάρτη (lane_map.py) πόσο απέχει από την αριστερή
και τη δεξιά γραμμή της λωρίδας, και το «χαλάει» όπως μια πραγματική κάμερα λωρίδων:
  - λάθος σε κάθε μέτρηση: τυπική απόκλιση 15 cm (Tao 2016: Mobileye σε αυτοκίνητο, ~10–22 cm)
  - σταθερό λάθος σε όλη τη διαδρομή, ένα για κάθε πλευρά: τυχαίο από −10 έως +10 cm (Tao 2016: μέσο λάθος
    από −0,10 έως +0,10 m, μαζί με το λάθος του χάρτη· ο δικός μας χάρτης είναι τέλειος, οπότε το προσθέτουμε)
  - «δεν είδα» (None) όπου η γραμμή δεν είναι βαμμένη (π.χ. μέσα σε διασταυρώσεις): η κάμερα βλέπει μόνο βαφή
  - προαιρετικά, επιπλέον τυχαίες απώλειες (για τα πειράματα: π.χ. 25% ή 50% των μετρήσεων)
Το φίλτρο ΔΕΝ ξέρει ούτε τα 15 cm ούτε το σταθερό λάθος (έχει το δικό του σ, όπως με το GPS).

Αργότερα τη θέση της παίρνει ο πραγματικός ανιχνευτής (CSV από το εργαστήριο, από τη semantic κάμερα).
Αυτός θα δίνει «τέλειες» μετρήσεις, και το ίδιο λάθος θα προστίθεται εδώ, στο Mac.

Απλοποιήσεις:
  - η απόσταση μετριέται κάθετα στον δρόμο (βλ. lane_map.expected_line_distances)
  - η γωνία λωρίδας και το είδος της γραμμής (10–11 Νοε 2026) βγαίνουν από ξεχωριστή συνάρτηση
    (measure_type_and_angle), με δικά τους «ζάρια», για να μην αλλάζουν οι μετρήσεις απόστασης
  - δεν «κρύβουν» αυτοκίνητα τις γραμμές: αυτό θα το έχει ο πραγματικός ανιχνευτής

Δοκιμή:
    python3 lane_camera.py
"""

import numpy as np

from lane_map import LINE_BROKEN, LINE_DOUBLE, LINE_SOLID, expected_line_distances, expected_line_types_and_angle

# ─── Είδος γραμμής και γωνία λωρίδας: ο «κόσμος» (10–11 Νοε 2026) ───────────────────────────────────────
ANGLE_NOISE_DEG = 1.0       # λάθος που προστίθεται στη γωνία σε κάθε μέτρηση (⚠️ η πηγή για το μέγεθος θέλει έλεγχο)
# Η ψεύτικη κάμερα μιμείται τον πραγματικό ανιχνευτή, όπως μετρήθηκε 10 Νοε 2026 (30 Οκτ 20:45 και 21:24):
SOLID_SEEN_AS_SOLID = 0.85      # τη συνεχή τη λέει συνεχή στο 85%...
SOLID_SEEN_AS_BROKEN = 0.08     # ...διακεκομμένη στο 8% (όταν φαίνεται μικρό κομμάτι της)· αλλιώς «δεν ξέρω»
BROKEN_SEEN_AS_BROKEN = 0.99    # τη διακεκομμένη σχεδόν πάντα σωστά· τη διπλή πάντα
ANGLE_AVAILABLE = 0.65          # γωνία υπάρχει στο 65% των εικόνων με γραμμή (όσο το angle_ok του ανιχνευτή)
ANGLE_TAIL_PROB, ANGLE_TAIL_DEG = 0.10, 5.0   # 1 στις 10 γωνίες έχει και ένα μεγάλο λάθος Ν(0, 5°): η «ουρά» του ανιχνευτή
# Ο ανιχνευτής δεν γράφει (ακόμα) αν η γραμμή είναι συνεχής ή διακεκομμένη. Το βγάζουμε από το πόσα σημεία της είδε
# (ένα ανά σειρά εικόνας, το πολύ ~293): πολλά = συνεχής, λίγα = διακεκομμένη, ενδιάμεσα = «δεν ξέρω».
SOLID_MIN_POINTS, BROKEN_MAX_POINTS = 230, 180


class FakeLaneCamera:
    def __init__(self, seed=None, noise_sigma=0.15, offset_max=0.10, extra_loss=0.0):
        """seed: τα «ζάρια» της κάμερας (δικά της, ανεξάρτητα από του GPS και του φίλτρου, με το ίδιο seed).
        noise_sigma: λάθος σε κάθε μέτρηση (m)· offset_max: όριο του σταθερού λάθους (m);
        extra_loss: πιθανότητα να χαθεί επιπλέον μια μέτρηση (0 = μόνο όπου δεν υπάρχει βαφή)."""
        # [seed, 2]: άλλη σειρά τυχαίων αριθμών από το GPS (που χρησιμοποιεί σκέτο seed), αλλιώς θα είχαν το ίδιο «ζάρι»
        self.rng = np.random.default_rng(None if seed is None else [seed, 2])
        self.noise_sigma = noise_sigma
        self.extra_loss = extra_loss
        # Σταθερό λάθος: μία φορά για όλη τη διαδρομή, ένα για κάθε πλευρά
        self.offset_left = self.rng.uniform(-offset_max, offset_max)
        self.offset_right = self.rng.uniform(-offset_max, offset_max)
        # Είδος και γωνία: άλλη σειρά τυχαίων αριθμών ([seed, 5]), ώστε οι μετρήσεις απόστασης να μένουν ίδιες με πριν
        self.rng_extra = np.random.default_rng(None if seed is None else [seed, 5])

    def measure_type_and_angle(self, x, y, yaw, frame=None):
        """Είδος της αριστερής και της δεξιάς γραμμής και γωνία της λωρίδας, στην ΑΛΗΘΙΝΗ θέση, με λάθη σαν του ανιχνευτή.
        Επιστρέφει (είδος αριστερά, είδος δεξιά, γωνία σε μοίρες). Είδος: LINE_BROKEN / LINE_SOLID / LINE_DOUBLE ή None
        («δεν ξέρω»). Γωνία: θετική = η λωρίδα πάει προς τα δεξιά του αυτοκινήτου· None αν δεν υπάρχει."""
        u = self.rng_extra.random(4)                         # τραβιούνται πάντα, για να μένει ίδια η σειρά
        noise, tail = self.rng_extra.normal(0, ANGLE_NOISE_DEG), self.rng_extra.normal(0, ANGLE_TAIL_DEG)
        left_type, right_type, angle = expected_line_types_and_angle(x, y, yaw)
        has_line = left_type[0] != 0 or right_type[0] != 0
        value = float(np.degrees(angle[0])) + noise + (tail if u[3] < ANGLE_TAIL_PROB else 0.0)
        return (self._degrade(int(left_type[0]), u[0]), self._degrade(int(right_type[0]), u[1]),
                value if (has_line and u[2] < ANGLE_AVAILABLE) else None)

    @staticmethod
    def _degrade(true_type, u):
        """Το είδος όπως θα το έλεγε ο ανιχνευτής (u: τυχαίος αριθμός 0–1)."""
        if true_type == LINE_SOLID:
            if u < SOLID_SEEN_AS_SOLID:
                return LINE_SOLID
            return LINE_BROKEN if u < SOLID_SEEN_AS_SOLID + SOLID_SEEN_AS_BROKEN else None
        if true_type == LINE_BROKEN:
            return LINE_BROKEN if u < BROKEN_SEEN_AS_BROKEN else None
        return LINE_DOUBLE if true_type == LINE_DOUBLE else None      # η άβαφη (0) δεν έχει είδος

    def measure(self, x, y, yaw, frame=None):
        """Μία εικόνα. x, y, yaw: η ΑΛΗΘΙΝΗ θέση και κατεύθυνση (yaw σε rad). frame: δεν χρησιμοποιείται εδώ
        (υπάρχει για να έχει την ίδια μορφή με την DetectorLaneCamera).
        Επιστρέφει (αριστερά m ή None, δεξιά m ή None). None = «δεν είδα γραμμή»."""
        left, right, left_painted, right_painted = expected_line_distances(x, y, yaw)
        # Οι τυχαίοι αριθμοί τραβιούνται σε ΚΑΘΕ εικόνα, για να μένει ίδια η σειρά τους ό,τι κι αν βγει
        noise_left, noise_right = self.rng.normal(0, self.noise_sigma, 2)
        lost_left, lost_right = self.rng.random(2) < self.extra_loss

        measured_left = left + self.offset_left + noise_left
        measured_right = right + self.offset_right + noise_right
        if not left_painted or lost_left:
            measured_left = None
        if not right_painted or lost_right:
            measured_right = None
        return measured_left, measured_right


class DetectorLaneCamera:
    """Η κάμερα λωρίδων από τον ΠΡΑΓΜΑΤΙΚΟ ανιχνευτή (lane_detector.py, στο εργαστήριο, από τη semantic κάμερα).
    Διαβάζει το drives/<καταγραφή>_lanes.csv: για κάθε εικόνα (frame), την απόσταση από την αριστερή και τη δεξιά
    γραμμή και αν τις είδε. Ο ανιχνευτής δίνει «τέλειες» μετρήσεις, οπότε εδώ, στο Mac, προστίθεται το ΙΔΙΟ λάθος
    με την FakeLaneCamera (15 cm + σταθερό έως ±10 cm + προαιρετικές επιπλέον απώλειες, απόφαση 9 Νοε 2026).
    «Δεν είδα» = ό,τι δεν είδε ο ανιχνευτής (διασταυρώσεις, αυτοκίνητα μπροστά, βέλη, διαβάσεις κ.λπ.)."""

    def __init__(self, drive_csv_path, seed=None, noise_sigma=0.15, offset_max=0.10, extra_loss=0.0):
        lanes_path = drive_csv_path[:-len(".csv")] + "_lanes.csv"
        det = np.genfromtxt(lanes_path, delimiter=",", names=True)
        # frame → (αριστερά, δεξιά), με None όπου ο ανιχνευτής δεν είδε γραμμή
        self.by_frame = {}
        self.extra_by_frame = {}     # frame → (είδος αριστερά, είδος δεξιά, γωνία αριστερά ή None, γωνία δεξιά ή None)
        has_extra = "left_kind" in det.dtype.names          # τα CSV πριν τις 9 Νοε 2026 δεν έχουν είδος και angle_ok
        # Από 11 Νοε 2026 ο ανιχνευτής γράφει ο ίδιος τον τύπο της γραμμής (left_type / right_type: 0 δεν ξέρω,
        # 1 διακεκομμένη, 2 συνεχής, 3 διπλή), από τα κενά της βαφής. Στα παλιότερα CSV τον βγάζουμε από τα σημεία.
        has_type = "left_type" in det.dtype.names
        for row in det:
            left = float(row["left_dist"]) if row["left_ok"] == 1 else None
            right = float(row["right_dist"]) if row["right_ok"] == 1 else None
            self.by_frame[int(row["frame"])] = (left, right)
            if has_extra:
                if has_type:
                    left_type, right_type = int(row["left_type"]) or None, int(row["right_type"]) or None
                else:
                    left_type = self._line_type(int(row["left_kind"]), row["left_points"])
                    right_type = self._line_type(int(row["right_kind"]), row["right_points"])
                self.extra_by_frame[int(row["frame"])] = (
                    left_type, right_type,
                    float(row["left_angle_deg"]) if row["left_angle_ok"] == 1 else None,
                    float(row["right_angle_deg"]) if row["right_angle_ok"] == 1 else None)
        # Ίδια «ζάρια» και ίδιο λάθος με την FakeLaneCamera
        self.rng = np.random.default_rng(None if seed is None else [seed, 2])
        self.noise_sigma = noise_sigma
        self.extra_loss = extra_loss
        self.offset_left = self.rng.uniform(-offset_max, offset_max)
        self.offset_right = self.rng.uniform(-offset_max, offset_max)
        self.rng_extra = np.random.default_rng(None if seed is None else [seed, 5])   # για το λάθος της γωνίας

    @staticmethod
    def _line_type(kind, points):
        """ΜΟΝΟ για τα CSV πριν τις 11 Νοε 2026 (χωρίς left_type / right_type).
        Είδος γραμμής από το CSV του ανιχνευτή: kind 2 = διπλή· kind 1 = μία γραμμή, συνεχής ή διακεκομμένη
        ανάλογα με το πόσα σημεία της είδε· αλλιώς None («δεν ξέρω»)."""
        if kind == 2:
            return LINE_DOUBLE
        if kind == 1 and points >= SOLID_MIN_POINTS:
            return LINE_SOLID
        if kind == 1 and points <= BROKEN_MAX_POINTS:
            return LINE_BROKEN
        return None

    def measure_type_and_angle(self, x, y, yaw, frame=None):
        """Είδος των δύο γραμμών και γωνία της λωρίδας από τον ανιχνευτή, στο frame (ή στο διπλανό). x, y, yaw ΔΕΝ
        χρησιμοποιούνται. Στη γωνία προστίθεται λάθος Ν(0, ANGLE_NOISE_DEG) ανά πλευρά· με δύο πλευρές, ο μέσος όρος τους.
        Επιστρέφει (είδος αριστερά, είδος δεξιά, γωνία σε μοίρες ή None), όπως η FakeLaneCamera."""
        noise = self.rng_extra.normal(0, ANGLE_NOISE_DEG, 2)     # τραβιούνται πάντα, για να μένει ίδια η σειρά
        found = next((self.extra_by_frame[f] for f in (frame, frame + 1, frame - 1) if f in self.extra_by_frame), None)
        if found is None:
            return None, None, None
        left_type, right_type, left_angle, right_angle = found
        angles = [a + n for a, n in ((left_angle, noise[0]), (right_angle, noise[1])) if a is not None]
        return left_type, right_type, (float(np.mean(angles)) if angles else None)

    def measure(self, x, y, yaw, frame=None):
        """Μία εικόνα: η μέτρηση του ανιχνευτή στο frame (ή στο διπλανό: η κάμερα βγάζει εικόνα κάθε 2ο βήμα).
        x, y, yaw ΔΕΝ χρησιμοποιούνται (ο ανιχνευτής δεν ξέρει την αλήθεια). Επιστρέφει (αριστερά ή None, δεξιά ή None)."""
        found = None
        for f in (frame, frame + 1, frame - 1):
            if f in self.by_frame:
                found = self.by_frame[f]
                break
        noise_left, noise_right = self.rng.normal(0, self.noise_sigma, 2)
        lost_left, lost_right = self.rng.random(2) < self.extra_loss
        if found is None:
            return None, None
        left, right = found
        measured_left = None if (left is None or lost_left) else left + self.offset_left + noise_left
        measured_right = None if (right is None or lost_right) else right + self.offset_right + noise_right
        return measured_left, measured_right


if __name__ == "__main__":
    import os

    drive = os.path.join(os.path.dirname(os.path.abspath(__file__)), "drives", "drive_2026-10-30_20-45-55.csv")
    data = np.genfromtxt(drive, delimiter=",", names=True)
    rows = range(0, len(data), 2)          # 10 εικόνες το δευτερόλεπτο = κάθε 2η γραμμή της καταγραφής (20 Hz)

    for extra_loss in [0.0, 0.5]:
        camera = FakeLaneCamera(seed=0, extra_loss=extra_loss)
        errors_left, errors_right, seen_left, seen_right = [], [], [], []
        for i in rows:
            x, y, yaw = data["gt_x"][i], data["gt_y"][i], np.radians(data["gt_yaw"][i])
            true_left, true_right, _, _ = expected_line_distances(x, y, yaw)
            m_left, m_right = camera.measure(x, y, yaw)
            seen_left.append(m_left is not None)
            seen_right.append(m_right is not None)
            if m_left is not None:
                errors_left.append(m_left - true_left)
            if m_right is not None:
                errors_right.append(m_right - true_right)
        errors_left, errors_right = np.array(errors_left), np.array(errors_right)
        print(f"\nεπιπλέον απώλειες {100 * extra_loss:.0f}% ({len(rows)} εικόνες):")
        print(f"  σταθερό λάθος που διαλέχτηκε: αριστερά {100 * camera.offset_left:+.1f} cm, "
              f"δεξιά {100 * camera.offset_right:+.1f} cm")
        print(f"  είδε γραμμή: αριστερά {100 * np.mean(seen_left):.0f}%, δεξιά {100 * np.mean(seen_right):.0f}% των εικόνων")
        print(f"  λάθος (μέτρηση − αλήθεια): αριστερά μέσο {100 * errors_left.mean():+.1f} cm, "
              f"τυπ. απόκλιση {100 * errors_left.std():.1f} cm· δεξιά μέσο {100 * errors_right.mean():+.1f} cm, "
              f"τυπ. απόκλιση {100 * errors_right.std():.1f} cm")
