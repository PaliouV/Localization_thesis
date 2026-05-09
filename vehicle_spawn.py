import carla
import time
import signal
import sys
import pygame
import numpy as np

# --- Σύνδεση ---
client = carla.Client('localhost', 2000)
client.set_timeout(10.0)
world = client.get_world()
blueprint_library = world.get_blueprint_library()

# --- Αυτοκίνητο ---
vehicle_bp = blueprint_library.find('vehicle.mercedes.coupe_2020')
spawn_point = world.get_map().get_spawn_points()[0]
vehicle = world.spawn_actor(vehicle_bp, spawn_point)
print(f"Spawned: {vehicle.type_id}")

# --- Autopilot ---
vehicle.set_autopilot(True, 8001)
traffic_manager = client.get_trafficmanager(8001)
traffic_manager.vehicle_percentage_speed_difference(vehicle, 50)
traffic_manager.distance_to_leading_vehicle(vehicle, 5)
traffic_manager.auto_lane_change(vehicle, False)

# --- GPS ---
gps_bp = blueprint_library.find('sensor.other.gnss')
gps_sensor = world.spawn_actor(gps_bp, carla.Transform(carla.Location(x=0, z=2)), attach_to=vehicle)

gps_data = {'lat': 0.0, 'lon': 0.0, 'alt': 0.0}

def gps_callback(data):
    gps_data['lat'] = data.latitude
    gps_data['lon'] = data.longitude
    gps_data['alt'] = data.altitude

gps_sensor.listen(gps_callback)

# --- Κάμερα ---
camera_bp = blueprint_library.find('sensor.camera.rgb')
camera_bp.set_attribute('image_size_x', '800')
camera_bp.set_attribute('image_size_y', '600')
camera_bp.set_attribute('fov', '90')
camera = world.spawn_actor(camera_bp, carla.Transform(carla.Location(x=1.5, z=2.0)), attach_to=vehicle)

latest_frame = [None]  # Αποθηκεύουμε το τελευταίο frame εδώ

def camera_callback(image):
    array = np.frombuffer(image.raw_data, dtype=np.uint8)
    array = array.reshape((image.height, image.width, 4))[:, :, :3]
    latest_frame[0] = array

camera.listen(camera_callback)

# --- Pygame ---
os_environ_set = __import__('os').environ.setdefault('SDL_VIDEODRIVER', 'x11')
pygame.init()
display = pygame.display.set_mode((800, 600))
pygame.display.set_caption('CARLA Camera')
font = pygame.font.SysFont('monospace', 18)

# --- Spectator ---
spectator = world.get_spectator()

# --- Cleanup ---
actors = [camera, gps_sensor, vehicle]

def cleanup(sig, frame):
    print("\nΚαθαρισμός...")
    for actor in actors:
        try:
            actor.stop() if hasattr(actor, 'stop') else None
            actor.destroy()
        except:
            pass
    pygame.quit()
    print("Bye!")
    sys.exit(0)

signal.signal(signal.SIGINT, cleanup)
signal.signal(signal.SIGTERM, cleanup)

print("Πάτα Ctrl+C για να κλείσεις\n")

# --- Main Loop ---
while True:
    # Spectator ακολουθεί το αυτοκίνητο
    transform = vehicle.get_transform()
    spectator.set_transform(carla.Transform(
        transform.location + carla.Location(x=-8, z=6),
        carla.Rotation(pitch=-25, yaw=transform.rotation.yaw)
    ))

    # Εμφάνιση κάμερας
    if latest_frame[0] is not None:
        surface = pygame.surfarray.make_surface(latest_frame[0].swapaxes(0, 1))
        display.blit(surface, (0, 0))

    # GPS overlay πάνω στο παράθυρο
    gps_text = font.render(
        f"GPS  lat: {gps_data['lat']:.6f}   lon: {gps_data['lon']:.6f}   alt: {gps_data['alt']:.2f}",
        True, (0, 255, 0)
    )
    display.blit(gps_text, (10, 10))
    pygame.display.flip()

    # Pygame events
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            cleanup(None, None)

    time.sleep(0.05)