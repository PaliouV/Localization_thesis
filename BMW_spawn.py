import time
import random
import threading
import carla
import csv
import math
import numpy as np
import cv2 as cv2
import argparse
import os
import queue

parser = argparse.ArgumentParser()
parser.add_argument("--cameras", action="store_true", help="CAMERAS MALAKA")
args = parser.parse_args()
if args.cameras:
    print("BROKOLOKO EVALES KAMERA")


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

original_settings = world.get_settings()

settings = world.get_settings()
settings.synchronous_mode = True
settings.fixed_delta_seconds = 0.05
world.apply_settings(settings)
tm.set_synchronous_mode(True)

vehicle.set_autopilot(True, tm.get_port())
tm.vehicle_percentage_speed_difference(vehicle, 30)   # 10% πιο αργά από το όριο ταχύτητας
tm.distance_to_leading_vehicle(vehicle, 5.0)           # μεγαλύτερη απόσταση ασφαλείας
tm.ignore_lights_percentage(vehicle, 0)                  # σταματάει στα φανάρια (στάσεις για το GPS «σταματημένο»)
tm.ignore_signs_percentage(vehicle, 0)                   # υπακούει στις πινακίδες


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

cameras = []                                      # (κατεύθυνση, είδος, αισθητήρας, ουρά) — άδεια χωρίς --cameras
if args.cameras:
    box = vehicle.bounding_box
    roof_z = box.location.z + box.extent.z + 0.05
    POS = {"front": (box.location.x + 0.25 * box.extent.x, 0.0),
        "rear": (box.location.x - 0.70 * box.extent.x, 0.0),
        "left": (box.location.x - 0.30 * box.extent.x, -0.85 * box.extent.y),
        "right": (box.location.x - 0.30 * box.extent.x, 0.85 * box.extent.y)}
    YAW = {"front": 0, "right": 90, "rear": 180, "left": -90}   # CARLA: yaw 90 = προς τα δεξιά του οχήματος

    camera_bps = {"rgb": blueprint_library.find('sensor.camera.rgb'),
                  "semantic": blueprint_library.find('sensor.camera.semantic_segmentation')}
    for bp in camera_bps.values():
        bp.set_attribute('image_size_x', '1280')
        bp.set_attribute('image_size_y', '960')
        bp.set_attribute('fov', '90')
        bp.set_attribute('sensor_tick', '0.1')       # 10 εικόνες το δευτερόλεπτο

    for name in POS:
        x, y = POS[name]
        tf = carla.Transform(carla.Location(x=x, y=y, z=roof_z), carla.Rotation(pitch=0, yaw=YAW[name]))
        for kind, bp in camera_bps.items():           # κανονική και «αλήθειας» στο ίδιο ακριβώς σημείο
            cam = world.spawn_actor(bp, tf, attach_to=vehicle)
            q = queue.Queue()
            cam.listen(q.put)
            cameras.append((name, kind, cam, q))


csv_name = time.strftime("drive_%Y-%m-%d_%H-%M-%S.csv")   # π.χ. drive_2026-09-24_18-05-12.csv
csv_file = open(csv_name, "w", newline="")
writer = csv.writer (csv_file)
writer.writerow(["frame", "sim_time", "gt_x", "gt_y", "gt_yaw", "vel", "gps_lat", "gps_lon", "gps_alt", "acc_x",
                 "acc_y", "acc_z", "gyro_x", "gyro_y", "gyro_z", "compass"])

# Εικόνες: φάκελος με το ίδιο όνομα με το CSV, και μέσα ένας υποφάκελος ανά κάμερα (π.χ. front_rgb/).
img_dir = csv_name[:-len(".csv")]
for name, kind, cam, q in cameras:
    os.makedirs(os.path.join(img_dir, f"{name}_{kind}"), exist_ok=True)


def save_images():
    """Σώζει όσες εικόνες έχουν φτάσει στις ουρές. Όνομα αρχείου = frame της εικόνας (ίδιο με τη στήλη frame του CSV)."""
    for name, kind, cam, q in cameras:
        while not q.empty():                              # μόνο ό,τι έχει ήδη φτάσει — δεν περιμένουμε
            image = q.get()
            pixels = np.frombuffer(image.raw_data, dtype=np.uint8).reshape((image.height, image.width, 4))   # B, G, R, A
            folder = os.path.join(img_dir, f"{name}_{kind}")
            if kind == "rgb":
                bgr = np.ascontiguousarray(pixels[:, :, :3])
                cv2.imwrite(os.path.join(folder, f"{image.frame}.jpg"), bgr, [cv2.IMWRITE_JPEG_QUALITY, 95])
            else:                                         # «αλήθειας»: ο αριθμός κατηγορίας είναι στο κόκκινο κανάλι
                tags = np.ascontiguousarray(pixels[:, :, 2])
                cv2.imwrite(os.path.join(folder, f"{image.frame}.png"), tags)

try:
    while True:
        frame = world.tick()
        save_images()                                     # χωρίς --cameras η λίστα είναι άδεια: δεν κάνει τίποτα
        sim_time = world.get_snapshot().timestamp.elapsed_seconds
        t = vehicle.get_transform()
        v = vehicle.get_velocity()
        speed = math.sqrt(v.x**2 + v.y**2 + v.z**2)

        if "gps_raw" not in state or "imu_raw" not in state:
            #time.sleep(0.1)
            continue
        writer.writerow([frame, sim_time, t.location.x, t.location.y, t.rotation.yaw, speed] + list(state["gps_raw"]) + list(state["imu_raw"]))
        #time.sleep(0.1)

except KeyboardInterrupt:
    pass
finally:
    # Κάμερες: πρώτα σταματάνε (αλλιώς, μόλις το CARLA γυρίσει σε asynchronous, θα έβγαζαν συνέχεια
    # νέες εικόνες), μετά σώζονται όσες έμειναν στις ουρές, και στο τέλος καταστρέφονται.
    for name, kind, cam, q in cameras:
        cam.stop()
    save_images()
    for name, kind, cam, q in cameras:
        cam.destroy()

    world.apply_settings(original_settings)   # το CARLA γυρίζει όπως ήταν (asynchronous)
    tm.set_synchronous_mode(False)            # και το autopilot
    csv_file.close()
    gnss_sensor.destroy()
    imu_sensor.destroy()
    vehicle.destroy()
