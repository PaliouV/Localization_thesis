"""
Εξάγει όλα τα κτήρια του τρέχοντος χάρτη του CARLA σε town12_buildings.json.

Τρέχει στο Linux desktop, με το CARLA ανοιχτό (Town12 φορτωμένο):
    ~/carla-env/bin/python town12/export_buildings.py

Για κάθε κτήριο γράφει κέντρο, μισές διαστάσεις (extent), περιστροφή και ύψος,
σε μέτρα x/y/z του CARLA. Από αυτά θα διαλέξουμε τις ζώνες θορύβου
(GPS_NOISE_MODEL.md §5) και θα μετρήσουμε τον λόγο ύψος κτηρίου / πλάτος δρόμου.

Αν υπάρχει ήδη όχημα με role_name "hero" (π.χ. από το vehicle_spawn.py),
τυπώνει και τη θέση του — για να ελέγξουμε τον άξονα y του χάρτη.
"""

import json
import os

import carla

HERE = os.path.dirname(os.path.abspath(__file__))   # ο φάκελος town12/

client = carla.Client("localhost", 2000)
client.set_timeout(10.0)
world = client.get_world()
map_name = world.get_map().name
print(f"Χάρτης: {map_name}")

buildings = []
for obj in world.get_environment_objects(carla.CityObjectLabel.Buildings):
    box = obj.bounding_box            # ήδη σε συντεταγμένες κόσμου
    buildings.append(dict(
        id=obj.id,
        name=obj.name,
        x=box.location.x,
        y=box.location.y,
        z=box.location.z,
        extent_x=box.extent.x,        # μισό μήκος
        extent_y=box.extent.y,        # μισό πλάτος
        extent_z=box.extent.z,        # μισό ύψος
        yaw=box.rotation.yaw,
        height=2 * box.extent.z,
        top_z=box.location.z + box.extent.z,
    ))

with open(os.path.join(HERE, "town12_buildings.json"), "w") as f:
    json.dump(dict(map=map_name, buildings=buildings), f, indent=1)

heights = sorted(b["height"] for b in buildings)
print(f"Κτήρια: {len(buildings)}")
if heights:
    print(f"Ύψος: min {heights[0]:.1f} m, διάμεσος {heights[len(heights) // 2]:.1f} m, max {heights[-1]:.1f} m")
print("Αποθηκεύτηκε: town12_buildings.json")

for actor in world.get_actors().filter("vehicle.*"):
    if actor.attributes.get("role_name") == "hero":
        loc = actor.get_location()
        print(f"\nΌχημα hero στο x={loc.x:.1f}, y={loc.y:.1f} — πες μου και σε ποιον δρόμο είναι.")
