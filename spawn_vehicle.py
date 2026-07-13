import time
import random
import threading
import carla
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


state = {"gps": "...", "imu": "..."}
render_lock = threading.Lock()
last_render_time = 0
RENDER_INTERVAL = 0.2  # seconds between redraws

def render():
    global last_render_time
    now = time.time()
    if now - last_render_time < RENDER_INTERVAL:
        return
    last_render_time = now
    with render_lock:
        print("\033[2A\033[KGPS: " + state["gps"])
        print("\033[K" + "IMU: " + state["imu"], flush=True)

def on_gnss(event):
    state["gps"] = f"lat={event.latitude:+.6f}, lon={event.longitude:+.6f}"
    render()

def on_imu(event):
    state["imu"] = f"accel=({event.accelerometer.x:+6.2f}, {event.accelerometer.y:+6.2f}, {event.accelerometer.z:+6.2f})"
    render()

print()
print()

sensor_transform = carla.Transform(carla.Location(x=1.0, z=2.0))
gnss_sensor = world.try_spawn_actor(gps, sensor_transform, attach_to=vehicle)
gnss_sensor.listen(on_gnss)
imu_sensor = world.try_spawn_actor(imu, sensor_transform, attach_to=vehicle)
imu_sensor.listen(on_imu)

try:
    while True:
        time.sleep(0.1)
except KeyboardInterrupt:
    pass
finally:
    gnss_sensor.destroy()
    imu_sensor.destroy()
    vehicle.destroy()