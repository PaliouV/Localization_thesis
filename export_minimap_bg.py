"""
Φτιάχνει το φόντο για τη δική μας κάτοψη από τον χάρτη του no_rendering_mode.py.

Το no_rendering_mode.py αποθηκεύει τον χάρτη ως εικόνα (.tga), αλλά ΟΧΙ τα νούμερα
που χρειάζονται για να αντιστοιχίσεις μέτρα → pixels. Αυτό το script τα υπολογίζει
με τον ίδιο τρόπο (MapImage.__init__, γραμμές 428-447) και γράφει:
  - town10_minimap.png        η εικόνα του χάρτη
  - town10_minimap.json       min_x, min_y, pixels_per_meter

Μετατροπή (ίδια με world_to_pixel του no_rendering_mode.py, γραμμές 847-850):
    pixel_x = pixels_per_meter * (x - min_x)
    pixel_y = pixels_per_meter * (y - min_y)

Τρέχει στο Linux desktop, με ανοιχτό το CARLA, ΑΦΟΥ έχει τρέξει μία φορά
το no_rendering_mode.py (για να υπάρχει το .tga):
    ~/carla-env/bin/python export_minimap_bg.py /path/to/cache/no_rendering_mode
"""

import glob
import json
import os
import sys

import carla
import pygame

MARGIN = 50              # ίδιο με το no_rendering_mode.py
PIXELS_PER_METER = 12    # ίδιο με το no_rendering_mode.py

cache_dir = sys.argv[1] if len(sys.argv) > 1 else "cache/no_rendering_mode"

client = carla.Client("localhost", 2000)
client.set_timeout(10.0)
carla_map = client.get_world().get_map()
map_name = carla_map.name.split("/")[-1]

# Ίδιος υπολογισμός με το MapImage.__init__
waypoints = carla_map.generate_waypoints(2)
xs = [w.transform.location.x for w in waypoints]
ys = [w.transform.location.y for w in waypoints]
min_x, max_x = min(xs) - MARGIN, max(xs) + MARGIN
min_y, max_y = min(ys) - MARGIN, max(ys) + MARGIN
width = max(max_x - min_x, max_y - min_y)
pixels_per_meter = min(int(((1 << 14) - 1) / width), PIXELS_PER_METER)

files = glob.glob(os.path.join(cache_dir, map_name + "_*.tga"))
if not files:
    sys.exit(f"Δεν βρέθηκε {map_name}_*.tga στο {cache_dir}. Τρέξε πρώτα το no_rendering_mode.py.")

image = pygame.image.load(files[0])
expected = int(pixels_per_meter * width)
print(f"Εικόνα: {files[0]} — {image.get_width()}x{image.get_height()} px (αναμενόταν {expected}x{expected})")
if image.get_width() != expected:
    print("⚠️ Το μέγεθος δεν ταιριάζει — πιθανόν άλλος χάρτης ή άλλη έκδοση του script.")

pygame.image.save(image, "town10_minimap.png")
with open("town10_minimap.json", "w") as f:
    json.dump(dict(map=map_name, min_x=min_x, min_y=min_y, pixels_per_meter=pixels_per_meter), f, indent=1)

print(f"min_x={min_x:.2f}, min_y={min_y:.2f}, pixels_per_meter={pixels_per_meter}")
print("Αποθηκεύτηκαν: town10_minimap.png, town10_minimap.json")
