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
  - δεν δίνει γωνία λωρίδας (το φίλτρο ξεκινάει μόνο με την απόσταση)
  - δεν «κρύβουν» αυτοκίνητα τις γραμμές: αυτό θα το έχει ο πραγματικός ανιχνευτής

Δοκιμή:
    python3 lane_camera.py
"""

import numpy as np

from lane_map import expected_line_distances


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

    def measure(self, x, y, yaw):
        """Μία εικόνα. x, y, yaw: η ΑΛΗΘΙΝΗ θέση και κατεύθυνση (yaw σε rad).
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
