"""
Δοκιμή γυροσκοπίου (21 Οκτ 2026): γιατί το gyro_z των καταγραφών δεν «αθροίζεται» στη γωνία;

Στο drive_2026-09-25_15-07-08.csv το άθροισμα gyro_z × dt δίνει 16° σε 10 s ευθεία (αλήθεια −0.6°).
Δύο πιθανές αιτίες, που ελέγχει αυτό το script:
  1. το BMW_spawn.py γράφει γραμμή κάθε 2 ticks (0.1 s), άρα κρατάει μία στις δύο στιγμιαίες τιμές·
  2. το BMW_spawn.py γράφει την τελευταία τιμή που έφτασε στο callback, που μπορεί να είναι από άλλο frame.

Γράφει γραμμή σε ΚΑΘΕ tick (0.05 s) με δύο ίδια IMU στο ίδιο σημείο:
  - «q»: τα δεδομένα του ΙΔΙΟΥ frame με το tick (μέσω ουράς, ο ενδεδειγμένος τρόπος σε synchronous mode)
  - «s»: όπως το BMW_spawn.py (τελευταία τιμή του callback)
Σταματάει μόνο του μετά από DURATION δευτερόλεπτα προσομοίωσης και τυπώνει τη σύγκριση.

    ~/carla-env/bin/python gyro_test.py        (με το CARLA ήδη ανοιχτό)
"""

import csv
import math
import queue
import random
import time

import carla
import numpy as np

DURATION = 120.0   # s προσομοίωσης
DT = 0.05          # ίδιο με το BMW_spawn.py

client = carla.Client('localhost', 2000)
client.set_timeout(10.0)
world = client.get_world()
print("Χάρτης:", world.get_map().name)

blueprint_library = world.get_blueprint_library()
car = blueprint_library.find('vehicle.bmw.grandtourer')
imu = blueprint_library.find('sensor.other.imu')

vehicle = world.try_spawn_actor(car, random.choice(world.get_map().get_spawn_points()))
if vehicle is None:
    raise RuntimeError("Could not spawn vehicle - spawn point may be occupied.")

tm = client.get_trafficmanager()
original_settings = world.get_settings()
settings = world.get_settings()
settings.synchronous_mode = True
settings.fixed_delta_seconds = DT
world.apply_settings(settings)
tm.set_synchronous_mode(True)

vehicle.set_autopilot(True, tm.get_port())
tm.vehicle_percentage_speed_difference(vehicle, 30)   # ίδιες ρυθμίσεις με το BMW_spawn.py
tm.distance_to_leading_vehicle(vehicle, 5.0)
tm.ignore_lights_percentage(vehicle, 100)
tm.ignore_signs_percentage(vehicle, 100)

sensor_transform = carla.Transform(carla.Location(x=1.0, z=2.0))
imu_q = world.try_spawn_actor(imu, sensor_transform, attach_to=vehicle)
imu_s = world.try_spawn_actor(imu, sensor_transform, attach_to=vehicle)
if imu_q is None or imu_s is None:
    raise RuntimeError("Could not spawn IMU sensor.")

q = queue.Queue()
imu_q.listen(q.put)

state = {}
def on_imu(event):                                     # όπως στο BMW_spawn.py
    state["s"] = (event.frame, event.gyroscope.z)
imu_s.listen(on_imu)

csv_name = time.strftime("gyro_test_%Y-%m-%d_%H-%M-%S.csv")
rows = []
try:
    start = None
    while True:
        frame = world.tick()
        sim_time = world.get_snapshot().timestamp.elapsed_seconds
        if start is None:
            start = sim_time
        if sim_time - start > DURATION:
            break

        ev = q.get(timeout=2.0)                        # δεδομένα ακριβώς αυτού του frame
        while ev.frame < frame:
            ev = q.get(timeout=2.0)

        t = vehicle.get_transform()
        v = vehicle.get_velocity()
        s_frame, s_gyro_z = state.get("s", (-1, float("nan")))
        rows.append([frame, sim_time, t.location.x, t.location.y, t.rotation.yaw,
                     math.sqrt(v.x**2 + v.y**2 + v.z**2),
                     ev.frame, ev.gyroscope.z, ev.compass, s_frame, s_gyro_z])
finally:
    world.apply_settings(original_settings)
    tm.set_synchronous_mode(False)
    imu_q.destroy()
    imu_s.destroy()
    vehicle.destroy()

with open(csv_name, "w", newline="") as f:
    w = csv.writer(f)
    w.writerow(["frame", "sim_time", "gt_x", "gt_y", "gt_yaw", "vel",
                "q_frame", "q_gyro_z", "q_compass", "s_frame", "s_gyro_z"])
    w.writerows(rows)
print("Γράφτηκε:", csv_name, f"({len(rows)} γραμμές)")

# ─── σύγκριση: αθροίζεται ο γυροσκόπος στη γωνία; ─────────────────────────────
a = np.array(rows, dtype=float)
yaw = np.degrees(np.unwrap(np.radians(a[:, 4])))
dt = np.diff(a[:, 1])

def summed(gyro, step):
    """Άθροισμα gyro × dt, κρατώντας μία τιμή κάθε `step` ticks (1 = όλες, 2 = όπως το BMW_spawn.py)."""
    g = np.repeat(gyro[::step], step)[:len(gyro)]
    return np.degrees(np.concatenate([[0], np.cumsum(g[1:] * dt)]))

ok = ~np.isnan(a[:, 10])
lag = (a[ok, 9] - a[ok, 0]).astype(int)
print("\n«s» (σαν το BMW_spawn.py): frame γυροσκοπίου − frame του tick:",
      {int(k): int(n) for k, n in zip(*np.unique(lag, return_counts=True))})

print("\n  t (s)   αλήθεια   q κάθε tick   q κάθε 2ο   s κάθε 2ο   (μοίρες)")
q_all, q_half = summed(a[:, 7], 1), summed(a[:, 7], 2)
s_half = summed(np.nan_to_num(a[:, 10]), 2)
for sec in (10, 30, 60, 120):
    k = min(int(sec / DT), len(a) - 1)
    print(f"  {sec:5d}  {yaw[k] - yaw[0]:8.1f}  {q_all[k]:12.1f}  {q_half[k]:10.1f}  {s_half[k]:10.1f}")
print("\nΑν η στήλη «q κάθε tick» ≈ «αλήθεια», ο γυροσκόπος είναι σωστός και φταίει ο τρόπος καταγραφής.")
