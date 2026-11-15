"""
Χαρτογράφηση κτηρίων με semantic LiDAR: ένα όχημα με autopilot γυρίζει την πόλη
και κρατάει μόνο τα σημεία που χτυπάνε κτήριο / τοίχο / γέφυρα, σε συντεταγμένες χάρτη.

Γιατί semantic LiDAR: κάθε σημείο έχει ακριβή θέση 3D ΚΑΙ ετικέτα (τι χτύπησε),
οπότε παίρνουμε τις πραγματικές προσόψεις — όχι τα κουτιά του export_buildings.py.

Τρέχει στο Linux desktop, με ανοιχτό το CARLA στο Town10HD και ΚΑΝΕΝΑ άλλο script
(βάζει το CARLA σε synchronous mode — το επαναφέρει στο τέλος):
    ~/carla-env/bin/python town10/lidar_mapping.py --minutes 10                (όπως πριν: autopilot)
    ~/carla-env/bin/python town10/lidar_mapping.py --sweep --out town10_lidar_map_sweep.npz

--sweep (προσθήκη 9 Νοε 2026): ΧΩΡΙΣ οδήγηση. Το αυτοκίνητο μεταφέρεται σε ΚΑΘΕ σημείο ΚΑΘΕ λωρίδας του χάρτη
(ανά --step m, από το map.generate_waypoints) και σε καθένα παίρνει μια πλήρη σάρωση 360°. Έτσι καλύπτονται σίγουρα
όλοι οι δρόμοι (με το autopilot, 10 λεπτά κάλυψαν κυρίως τα εσωτερικά τετράγωνα). Χωρίς φυσική (δεν «πέφτει»).
--out: όνομα αρχείου, για να μη γράφει πάνω στο παλιό town10_lidar_map.npz.

Ctrl+C σταματάει νωρίτερα και σώζει κανονικά ό,τι έχει μαζευτεί.
Έξοδος: town10_lidar_map.npz
    points  (N, 3)  κέντρα voxel σε μέτρα CARLA
    tag     (N,)    ετικέτα CARLA (Buildings / Walls / Bridge)
    obj     (N,)    id αντικειμένου που χτυπήθηκε
    traj    (M, 2)  x, y του οχήματος — για να δούμε ποιοι δρόμοι καλύφθηκαν
"""

import argparse
import os
import queue
import random

import carla
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
DT = 0.05            # s ανά βήμα προσομοίωσης (20 Hz)
VOXEL = 0.25         # m — σημεία στο ίδιο κυβάκι 25 cm κρατιούνται μία φορά
KEEP = [carla.CityObjectLabel.Buildings, carla.CityObjectLabel.Walls, carla.CityObjectLabel.Bridge]
KEEP_TAGS = np.array([int(t) for t in KEEP], dtype=np.uint32)

POINT = np.dtype([("x", "f4"), ("y", "f4"), ("z", "f4"), ("cos", "f4"), ("obj", "u4"), ("tag", "u4")])

parser = argparse.ArgumentParser()
parser.add_argument("--minutes", type=float, default=10.0, help="χρόνος οδήγησης (προσομοίωσης)")
parser.add_argument("--sweep", action="store_true", help="τηλεμεταφορά σε όλα τα σημεία όλων των λωρίδων, χωρίς οδήγηση")
parser.add_argument("--step", type=float, default=2.0, help="με --sweep: απόσταση ανάμεσα στα σημεία (m)")
parser.add_argument("--out", default="town10_lidar_map.npz", help="όνομα αρχείου εξόδου (στον φάκελο town10/)")
args = parser.parse_args()

client = carla.Client("localhost", 2000)
client.set_timeout(20.0)
world = client.get_world()
tm = client.get_trafficmanager()
original_settings = world.get_settings()

actors = []
kept = np.empty((0, 5), dtype=np.int64)   # [kx, ky, kz, tag, obj] — ένα ανά voxel
pending, traj = [], []
try:
    # --- synchronous mode: ο κόσμος προχωράει μόνο με world.tick() ---
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = DT
    world.apply_settings(settings)
    tm.set_synchronous_mode(True)

    lib = world.get_blueprint_library()
    vehicle = None
    for spawn in random.sample(world.get_map().get_spawn_points(), 10):
        vehicle = world.try_spawn_actor(lib.find("vehicle.bmw.grandtourer"), spawn)
        if vehicle is not None:
            break
    if vehicle is None:
        raise RuntimeError("Could not spawn vehicle.")
    actors.append(vehicle)
    if args.sweep:
        vehicle.set_simulate_physics(False)       # το μεταφέρουμε εμείς· χωρίς φυσική δεν πέφτει και δεν αναπηδά
        route = [w.transform for w in world.get_map().generate_waypoints(args.step)]
    else:
        vehicle.set_autopilot(True, tm.get_port())
        tm.ignore_lights_percentage(vehicle, 100)   # χωρίς άλλη κίνηση — δεν χρειάζεται να σταματάει, καλύπτει γρηγορότερα

    lidar_bp = lib.find("sensor.lidar.ray_cast_semantic")
    lidar_bp.set_attribute("channels", "64")
    lidar_bp.set_attribute("range", "120")
    lidar_bp.set_attribute("points_per_second", "500000")
    lidar_bp.set_attribute("rotation_frequency", str(1 / DT))   # μία πλήρης περιστροφή ανά βήμα
    lidar_bp.set_attribute("upper_fov", "60")    # ψηλά, για να φτάνει στις προσόψεις
    lidar_bp.set_attribute("lower_fov", "-15")   # το έδαφος δεν μας ενδιαφέρει
    lidar = world.spawn_actor(lidar_bp, carla.Transform(carla.Location(z=2.5)), attach_to=vehicle)
    actors.append(lidar)

    measurements = queue.Queue()
    lidar.listen(measurements.put)
    spectator = world.get_spectator()

    if args.sweep:
        steps = len(route)
        print(f"Σάρωση σε {steps} σημεία λωρίδων (ανά {args.step} m). Ctrl+C για νωρίτερο τέλος.")
    else:
        steps = int(args.minutes * 60 / DT)
        print(f"Οδήγηση για {args.minutes} λεπτά προσομοίωσης ({steps} βήματα). Ctrl+C για νωρίτερο τέλος.")

    for step in range(steps):
        if args.sweep:
            vehicle.set_transform(route[step])    # η σάρωση μετά το tick είναι από αυτή τη θέση (m.transform)
        world.tick()
        m = measurements.get(timeout=5.0)

        pts = np.frombuffer(m.raw_data, dtype=POINT)
        pts = pts[np.isin(pts["tag"], KEEP_TAGS)]
        if len(pts):
            # από συντεταγμένες αισθητήρα σε συντεταγμένες χάρτη, με τη θέση
            # του αισθητήρα ΤΗ ΣΤΙΓΜΗ της λήψης (m.transform)
            local = np.stack([pts["x"], pts["y"], pts["z"], np.ones(len(pts), "f4")])
            world_xyz = (np.array(m.transform.get_matrix()) @ local)[:3].T
            keys = np.floor(world_xyz / VOXEL).astype(np.int64)
            pending.append(np.column_stack([keys, pts["tag"], pts["obj"]]))

        if args.sweep or step % 10 == 0:
            loc = vehicle.get_location()
            traj.append((loc.x, loc.y))
            spectator.set_transform(carla.Transform(loc + carla.Location(z=70), carla.Rotation(pitch=-90)))

        if step % 100 == 0 and pending:
            # κρατάμε ένα σημείο ανά voxel για να μη γεμίσει η μνήμη
            everything = np.vstack([kept] + pending)
            _, first = np.unique(everything[:, :4], axis=0, return_index=True)
            kept, pending = everything[first], []
            print(f"  {step * DT:6.0f} s   voxels: {len(kept):8d}")

except KeyboardInterrupt:
    print("\nΣταμάτησε από τον χρήστη — σώζω ό,τι μαζεύτηκε.")
finally:
    world.apply_settings(original_settings)
    tm.set_synchronous_mode(False)
    for a in reversed(actors):
        a.destroy()

if pending:
    everything = np.vstack([kept] + pending)
    _, first = np.unique(everything[:, :4], axis=0, return_index=True)
    kept = everything[first]

np.savez_compressed(os.path.join(HERE, args.out),
                    points=((kept[:, :3] + 0.5) * VOXEL).astype(np.float32),
                    tag=kept[:, 3].astype(np.uint8),
                    obj=kept[:, 4].astype(np.uint32),
                    traj=np.array(traj, dtype=np.float32))
size_mb = os.path.getsize(os.path.join(HERE, args.out)) / 1e6
print(f"Αποθηκεύτηκε: {args.out} — {len(kept)} voxels, {size_mb:.1f} MB")
