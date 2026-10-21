"""
Φορτώνει μια πόλη στο CARLA (αν δεν είναι ήδη φορτωμένη) και σώζει τον χάρτη δρόμων της
(OpenDRIVE) ως .xodr στον φάκελο town13/. Το .xodr το χρειάζεται το parse_xodr.py.

Για το Town10 το Town10HD_Opt.xodr το είχαμε ήδη· για νέα πόλη το παίρνουμε από το ίδιο το CARLA.
Τα υπόλοιπα scripts δουλεύουν στον χάρτη που είναι ήδη φορτωμένος, οπότε αυτό τρέχει ΠΡΩΤΟ.

Με το CARLA ανοιχτό:
    ~/carla-env/bin/python town13/export_xodr.py           (φορτώνει το Town13)
"""

import os
import sys

import carla

HERE = os.path.dirname(os.path.abspath(__file__))   # ο φάκελος town13/
TOWN = sys.argv[1] if len(sys.argv) > 1 else "Town13"

client = carla.Client("localhost", 2000)
client.set_timeout(120.0)                            # οι μεγάλοι χάρτες αργούν να φορτώσουν
world = client.get_world()
if not world.get_map().name.endswith(TOWN):
    print(f"Φόρτωση {TOWN} ...")
    world = client.load_world(TOWN)
print(f"Χάρτης: {world.get_map().name}")

path = os.path.join(HERE, f"{TOWN}.xodr")
with open(path, "w") as f:
    f.write(world.get_map().to_opendrive())
print(f"Αποθηκεύτηκε: {path}")
