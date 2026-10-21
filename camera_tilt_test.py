"""
Δοκιμή θέσης και κλίσης καμερών (21 Οκτ 2026), πριν γραφτεί το BMW_spawn.py --cameras.

Βάζει 4 κάμερες στις άκρες της οροφής (μπρος, πίσω, αριστερά, δεξιά), η καθεμία σε 3 κλίσεις,
αφήνει το αυτοκίνητο να οδηγήσει με autopilot και βγάζει φωτογραφίες σε 3 στιγμές.
Έτσι συγκρίνεις με τα μάτια: πόσο καπό/οροφή φαίνεται, αν φαίνονται οι γραμμές, πόσο κτίριο.

Τυπώνει και τις διαστάσεις του BMW (bounding box), για να διορθωθούν οι θέσεις αν χρειάζεται.

    ~/carla-env/bin/python camera_tilt_test.py        (με το CARLA ανοιχτό στο Town10HD_Opt)
Εικόνες: camera_tilt_test/<στιγμή>_<κάμερα>_<κλίση>.png
"""

import os
import queue
import random

import carla

DT = 0.05
SHOTS = (5.0, 15.0, 25.0)                     # s — πότε βγαίνουν φωτογραφίες
PITCHES = {"front": (0, -5, -10), "rear": (0, -5, -10), "left": (0, -15, -30), "right": (0, -15, -30)}
YAW = {"front": 0, "right": 90, "rear": 180, "left": -90}   # CARLA: yaw 90 = προς τα δεξιά του οχήματος
OUT = "camera_tilt_test"

client = carla.Client('localhost', 2000)
client.set_timeout(10.0)
world = client.get_world()
print("Χάρτης:", world.get_map().name)

lib = world.get_blueprint_library()
vehicle = world.try_spawn_actor(lib.find('vehicle.bmw.grandtourer'), random.choice(world.get_map().get_spawn_points()))
if vehicle is None:
    raise RuntimeError("Could not spawn vehicle - spawn point may be occupied.")

box = vehicle.bounding_box
print(f"BMW: μήκος {2*box.extent.x:.2f} m, πλάτος {2*box.extent.y:.2f} m, ύψος {2*box.extent.z:.2f} m, "
      f"κέντρο κουτιού ({box.location.x:.2f}, {box.location.y:.2f}, {box.location.z:.2f})")

# Θέσεις στην οροφή, ως κλάσματα του κουτιού (ΕΚΤΙΜΗΣΗ — η οροφή είναι μικρότερη από το κουτί).
# Αν οι εικόνες δείχνουν πολύ οροφή/καπό ή κάμερα «στον αέρα», αλλάζουν αυτά τα νούμερα.
roof_z = box.location.z + box.extent.z + 0.05
POS = {"front": (box.location.x + 0.25 * box.extent.x, 0.0),
       "rear": (box.location.x - 0.70 * box.extent.x, 0.0),
       "left": (box.location.x - 0.30 * box.extent.x, -0.85 * box.extent.y),
       "right": (box.location.x - 0.30 * box.extent.x, 0.85 * box.extent.y)}
for name, (x, y) in POS.items():
    print(f"  κάμερα {name:<5}: x = {x:+.2f}, y = {y:+.2f}, z = {roof_z:.2f} m")

tm = client.get_trafficmanager()
original_settings = world.get_settings()
settings = world.get_settings()
settings.synchronous_mode = True
settings.fixed_delta_seconds = DT
world.apply_settings(settings)
tm.set_synchronous_mode(True)
vehicle.set_autopilot(True, tm.get_port())
tm.vehicle_percentage_speed_difference(vehicle, 30)

cam_bp = lib.find('sensor.camera.rgb')
cam_bp.set_attribute('image_size_x', '1280')
cam_bp.set_attribute('image_size_y', '960')
cam_bp.set_attribute('fov', '90')

cameras = []                                  # (όνομα, κλίση, αισθητήρας, ουρά)
for name, pitches in PITCHES.items():
    x, y = POS[name]
    for p in pitches:
        tf = carla.Transform(carla.Location(x=x, y=y, z=roof_z), carla.Rotation(pitch=p, yaw=YAW[name]))
        cam = world.spawn_actor(cam_bp, tf, attach_to=vehicle)
        q = queue.Queue()
        cam.listen(q.put)
        cameras.append((name, p, cam, q))

os.makedirs(OUT, exist_ok=True)
try:
    shots = list(SHOTS)
    t0 = None
    while shots:
        frame = world.tick()
        t = world.get_snapshot().timestamp.elapsed_seconds
        t0 = t if t0 is None else t0
        take = t - t0 >= shots[0]
        for name, p, cam, q in cameras:      # αδειάζουμε τις ουρές σε κάθε tick
            img = q.get(timeout=5.0)
            while img.frame < frame:
                img = q.get(timeout=5.0)
            if take:
                img.save_to_disk(os.path.join(OUT, f"t{shots[0]:02.0f}_{name}_{p:+d}.png"))
        if take:
            print(f"Φωτογραφίες στα {shots[0]:.0f} s")
            shots.pop(0)
finally:
    world.apply_settings(original_settings)
    tm.set_synchronous_mode(False)
    for _, _, cam, _ in cameras:
        cam.destroy()
    vehicle.destroy()

print(f"Έτοιμο: {OUT}/ ({len(SHOTS) * len(cameras)} εικόνες)")
