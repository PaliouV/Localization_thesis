"""
Εξάγει τις κολόνες, τα φανάρια και τις πινακίδες του τρέχοντος χάρτη του CARLA σε town10_poles.json (12 Νοε 2026).
Ίδιος τρόπος με το export_buildings.py (world.get_environment_objects), άλλες κατηγορίες.

Τρέχει στον σταθερό, με το CARLA ανοιχτό (Town10HD φορτωμένο):
    ~/carla-env/bin/python town10/export_poles.py

Για κάθε αντικείμενο γράφει είδος, κέντρο, μισές διαστάσεις (extent) και περιστροφή του κουτιού του, σε μέτρα του CARLA.
Γράφει χωριστά και τους actors των φαναριών (traffic.traffic_light), για σύγκριση.
⚠️ Δεν ξέρουμε ακόμα αν κάθε «αντικείμενο» είναι ΜΙΑ κολόνα ή ολόκληρη ομάδα· το δείχνουν οι διαστάσεις που τυπώνει.
"""

import json
import os

import carla

HERE = os.path.dirname(os.path.abspath(__file__))   # ο φάκελος town10/

# το όνομα που γράφουμε → πιθανά ονόματα της κατηγορίας στο carla.CityObjectLabel (δοκιμάζονται με τη σειρά)
LABELS = {"pole": ["Poles", "Pole"], "light": ["TrafficLight", "TrafficLights"], "sign": ["TrafficSigns", "TrafficSign"]}

client = carla.Client("localhost", 2000)
client.set_timeout(10.0)
world = client.get_world()
map_name = world.get_map().name
print(f"Χάρτης: {map_name}")
print("Κατηγορίες του CityObjectLabel:", ", ".join(n for n in dir(carla.CityObjectLabel) if n[0].isupper()))

objects = []
for kind, names in LABELS.items():
    label = next((getattr(carla.CityObjectLabel, n) for n in names if hasattr(carla.CityObjectLabel, n)), None)
    if label is None:
        print(f"⚠️ {kind}: καμία από τις κατηγορίες {names} δεν υπάρχει σε αυτή την έκδοση. Παραλείπεται.")
        continue
    found = world.get_environment_objects(label)
    for obj in found:
        box = obj.bounding_box            # ήδη σε συντεταγμένες κόσμου
        objects.append(dict(kind=kind, id=obj.id, name=obj.name,
                            x=box.location.x, y=box.location.y, z=box.location.z,
                            extent_x=box.extent.x, extent_y=box.extent.y, extent_z=box.extent.z,
                            yaw=box.rotation.yaw))
    print(f"{kind}: {len(found)} αντικείμενα")
    if found:
        width = sorted(2 * max(o.bounding_box.extent.x, o.bounding_box.extent.y) for o in found)
        height = sorted(2 * o.bounding_box.extent.z for o in found)
        mid = len(found) // 2
        print(f"   πλάτος κουτιού: min {width[0]:.2f}, διάμεσος {width[mid]:.2f}, max {width[-1]:.2f} m · "
              f"ύψος: min {height[0]:.2f}, διάμεσος {height[mid]:.2f}, max {height[-1]:.2f} m")
        print("   παραδείγματα ονομάτων:", ", ".join(sorted(set(o.name for o in found))[:6]))

light_actors = []
for actor in world.get_actors().filter("traffic.traffic_light*"):
    loc = actor.get_transform().location
    light_actors.append(dict(id=actor.id, x=loc.x, y=loc.y, z=loc.z, yaw=actor.get_transform().rotation.yaw))
print(f"actors φαναριών: {len(light_actors)}")

with open(os.path.join(HERE, "town10_poles.json"), "w") as f:
    json.dump(dict(map=map_name, objects=objects, light_actors=light_actors), f, indent=1)
print("Αποθηκεύτηκε: town10/town10_poles.json")
