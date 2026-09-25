import time
import random
import threading
import carla
import csv
import math
import numpy as np
import cv2 as cv2

client = carla.Client('localhost', 2000)
client.set_timeout(10.0)
world = client.get_world()


blueprint_library = world.get_blueprint_library()
car = blueprint_library.find('vehicle.bmw.grandtourer')
gps = blueprint_library.find('sensor.other.gnss')
imu = blueprint_library.find('sensor.other.imu')

spawn_points = world.get_map().get_spawn_points()
random_spawn = random.choice(spawn_points)

vehicle = world.try_spawn_actor(car, random_spawn)
if vehicle is None:
    raise RuntimeError("Could not spawn vehicle - spawn point may be occupied.")

tm = client.get_trafficmanager()
vehicle.set_autopilot(True, tm.get_port())
tm.vehicle_percentage_speed_difference(vehicle, 10)   # 10% πιο αργά από το όριο ταχύτητας
tm.distance_to_leading_vehicle(vehicle, 5.0)           # μεγαλύτερη απόσταση ασφαλείας
tm.ignore_lights_percentage(vehicle, 100)                # παντα αγνοεί φανάρι
tm.ignore_signs_percentage(vehicle, 100)                 # παντα αγνοεί πινακίδα


state = {"gps": "...", "imu": "..."}
render_lock = threading.Lock()
last_render_time = 0
RENDER_INTERVAL = 0.2  # seconds between redraws

def render():
    global last_render_time
    with render_lock:
        now = time.time()
        if now - last_render_time < RENDER_INTERVAL:
            return
        last_render_time = now
        print("\033[2A\033[KGPS: " + state["gps"])
        print("\033[K" + "IMU: " + state["imu"], flush=True)

def on_gnss(event):
    state["gps"] = f"lat={event.latitude:+.6f}, lon={event.longitude:+.6f}"
    state["gps_raw"] = (event.latitude, event.longitude, event.altitude)

    render()

def on_imu(event):
    state["imu"] = f"accel=({event.accelerometer.x:+6.2f}, {event.accelerometer.y:+6.2f}, {event.accelerometer.z:+6.2f})"
    state["imu_raw"] = (event.accelerometer.x, event.accelerometer.y, event.accelerometer.z, event.gyroscope.x, event.gyroscope.y, event.gyroscope.z, event.compass)

    render()

print()
print()

sensor_transform = carla.Transform(carla.Location(x=1.0, z=2.0))
gnss_sensor = world.try_spawn_actor(gps, sensor_transform, attach_to=vehicle)
if gnss_sensor is None:
    raise RuntimeError("Could not spawn GNSS sensor.")
gnss_sensor.listen(on_gnss)

imu_sensor = world.try_spawn_actor(imu, sensor_transform, attach_to=vehicle)
if imu_sensor is None:
    raise RuntimeError("Could not spawn IMU sensor.")
imu_sensor.listen(on_imu)

csv_name = time.strftime("drive_%Y-%m-%d_%H-%M-%S.csv")   # π.χ. drive_2026-09-24_18-05-12.csv
csv_file = open(csv_name, "w", newline="")
writer = csv.writer (csv_file)
writer.writerow(["sim_time", "gt_x", "gt_y", "gt_yaw", "vel", "gps_lat", "gps_lon", "gps_alt", "acc_x",
                 "acc_y", "acc_z", "gyro_x", "gyro_y", "gyro_z", "compass"])

try:
    while True:
        sim_time = world.get_snapshot().timestamp.elapsed_seconds
        t = vehicle.get_transform()
        v = vehicle.get_velocity()
        speed = math.sqrt(v.x**2 + v.y**2 + v.z**2)

        if "gps_raw" not in state or "imu_raw" not in state:
            time.sleep(0.1)
            continue
        writer.writerow([sim_time, t.location.x, t.location.y, t.rotation.yaw, speed] + list(state["gps_raw"]) + list(state["imu_raw"]))
        time.sleep(0.1)

except KeyboardInterrupt:
    pass
finally:
    csv_file.close()
    gnss_sensor.destroy()
    imu_sensor.destroy()
    vehicle.destroy()