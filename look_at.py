"""
Μετακινεί την κάμερα του CARLA (spectator) ώστε να κοιτάει ένα σημείο του χάρτη.
Για να ελέγχουμε με τα μάτια τι υπάρχει εκεί (π.χ. αν ένα κτήριο είναι ψηλό).

Με το CARLA ανοιχτό:
    ~/carla-env/bin/python look_at.py 144 22          # λοξή ματιά από νότια, φαίνεται το ύψος
    ~/carla-env/bin/python look_at.py 144 22 --top    # κάτοψη από ψηλά

x, y σε μέτρα του CARLA. Η κάμερα μένει εκεί μέχρι να τη μετακινήσεις με το ποντίκι.
"""

import argparse

import carla

parser = argparse.ArgumentParser()
parser.add_argument("x", type=float)
parser.add_argument("y", type=float)
parser.add_argument("--top", action="store_true", help="κάτοψη αντί για λοξή ματιά")
parser.add_argument("--dist", type=float, default=80.0, help="απόσταση κάμερας (m)")
args = parser.parse_args()

client = carla.Client("localhost", 2000)
client.set_timeout(10.0)
spectator = client.get_world().get_spectator()

if args.top:
    # από πάνω, κοιτάει κάθετα κάτω
    camera = carla.Transform(carla.Location(x=args.x, y=args.y, z=args.dist),
                             carla.Rotation(pitch=-90.0))
else:
    # από τα νότια (+y), λίγο ψηλά, κοιτάει προς τα βόρεια (−y) με κλίση προς τα κάτω
    camera = carla.Transform(carla.Location(x=args.x, y=args.y + args.dist, z=args.dist * 0.4),
                             carla.Rotation(pitch=-20.0, yaw=-90.0))

spectator.set_transform(camera)
print(f"Κάμερα στο {camera.location}, κοιτάει προς ({args.x}, {args.y})")
