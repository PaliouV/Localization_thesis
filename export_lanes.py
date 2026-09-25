"""
Εξάγει τη γεωμετρία όλων των λωρίδων του χάρτη σε town10_lanes.csv, για να
δουλεύουμε τις λωρίδες χωρίς CARLA.

Ένα waypoint κάθε STEP μέτρα, σε κάθε λωρίδα οδήγησης. Για κάθε waypoint:
  - θέση κέντρου λωρίδας (x, y, z) και κατεύθυνση (yaw) — μέτρα/μοίρες CARLA
  - πλάτος λωρίδας
  - αριστερό και δεξί όριο της λωρίδας (x, y) — εκεί είναι οι γραμμές
  - τι γραμμή έχει αριστερά/δεξιά (Solid, Broken, SolidSolid, NONE, ...) και χρώμα
  - road_id, section_id, lane_id, s — ποιος δρόμος/λωρίδα και πόσο μέσα στον δρόμο
  - τύπος λωρίδας και αν είναι μέσα σε διασταύρωση

Τρέχει στο Linux desktop, με ανοιχτό το CARLA στο Town10HD:
    ~/carla-env/bin/python export_lanes.py
"""

import csv
import os

import carla

HERE = os.path.dirname(os.path.abspath(__file__))
STEP = 1.0   # m ανάμεσα σε διαδοχικά waypoints

client = carla.Client("localhost", 2000)
client.set_timeout(20.0)
carla_map = client.get_world().get_map()

# λωρίδες οδήγησης — αυτές μας ενδιαφέρουν για το ταίριασμα με την κάμερα
waypoints = carla_map.generate_waypoints(STEP)

path = os.path.join(HERE, "town10_lanes.csv")
with open(path, "w", newline="") as f:
    writer = csv.writer(f)
    writer.writerow([
        "road_id", "section_id", "lane_id", "s", "lane_type", "is_junction",
        "x", "y", "z", "yaw", "width",
        "left_x", "left_y", "right_x", "right_y",
        "left_marking", "left_color", "right_marking", "right_color",
    ])
    for w in waypoints:
        t = w.transform
        right = t.get_right_vector()          # μοναδιαίο διάνυσμα προς τα δεξιά της λωρίδας
        half = w.lane_width / 2
        left_m, right_m = w.left_lane_marking, w.right_lane_marking
        writer.writerow([
            w.road_id, w.section_id, w.lane_id, round(w.s, 3), str(w.lane_type), w.is_junction,
            round(t.location.x, 3), round(t.location.y, 3), round(t.location.z, 3),
            round(t.rotation.yaw, 3), round(w.lane_width, 3),
            round(t.location.x - right.x * half, 3), round(t.location.y - right.y * half, 3),
            round(t.location.x + right.x * half, 3), round(t.location.y + right.y * half, 3),
            str(left_m.type), str(left_m.color), str(right_m.type), str(right_m.color),
        ])

lanes = {(w.road_id, w.section_id, w.lane_id) for w in waypoints}
print(f"Χάρτης: {carla_map.name}")
print(f"{len(waypoints)} waypoints σε {len(lanes)} λωρίδες")
print(f"Αποθηκεύτηκε: {path}")
