#!/usr/bin/env python3
"""
CARLA Mercedes Coupe 2020 manual driving with RGB, Depth, Semantic cameras,
GNSS/GPS and IMU logging.

Controls
--------
W / Up      : throttle
S / Down    : brake
A / Left    : steer left
D / Right   : steer right
Space       : hand brake
Q           : toggle reverse
R           : reset vehicle at a new spawn point
ESC         : quit

Run
---
python mercedes_multisensor_drive_v2.py --host 127.0.0.1 --port 2000 --res 1440x480
python mercedes_multisensor_drive_v2.py --sync --fps 30
"""

import argparse
import csv
import math
import os
import random
import weakref
from datetime import datetime

import carla
import numpy as np
import pygame
from carla import ColorConverter as cc
from pygame.locals import (
    K_ESCAPE, K_q, K_r, K_w, K_s, K_a, K_d,
    K_UP, K_DOWN, K_LEFT, K_RIGHT, K_SPACE
)

CAMERA_TYPES = [
    ("rgb", "sensor.camera.rgb", cc.Raw, "RGB"),
    ("depth", "sensor.camera.depth", cc.LogarithmicDepth, "Depth"),
    ("semantic", "sensor.camera.semantic_segmentation", cc.CityScapesPalette, "Semantic"),
]


class SensorState:
    def __init__(self):
        self.gps = {"frame": 0, "timestamp": 0.0, "lat": 0.0, "lon": 0.0, "alt": 0.0}
        self.imu = {
            "frame": 0, "timestamp": 0.0,
            "accel_x": 0.0, "accel_y": 0.0, "accel_z": 0.0,
            "gyro_x": 0.0, "gyro_y": 0.0, "gyro_z": 0.0,
            "compass": 0.0,
        }

    def update_gps(self, event):
        self.gps.update({
            "frame": event.frame,
            "timestamp": event.timestamp,
            "lat": event.latitude,
            "lon": event.longitude,
            "alt": event.altitude,
        })

    def update_imu(self, event):
        self.imu.update({
            "frame": event.frame,
            "timestamp": event.timestamp,
            "accel_x": event.accelerometer.x,
            "accel_y": event.accelerometer.y,
            "accel_z": event.accelerometer.z,
            "gyro_x": math.degrees(event.gyroscope.x),
            "gyro_y": math.degrees(event.gyroscope.y),
            "gyro_z": math.degrees(event.gyroscope.z),
            "compass": math.degrees(event.compass),
        })


class CsvLogger:
    def __init__(self, path):
        self.file = open(path, "w", newline="", encoding="utf-8")
        self.writer = csv.DictWriter(self.file, fieldnames=[
            "wall_time", "carla_frame",
            "gps_timestamp", "latitude", "longitude", "altitude",
            "imu_timestamp", "accel_x", "accel_y", "accel_z",
            "gyro_x_deg_s", "gyro_y_deg_s", "gyro_z_deg_s", "compass_deg",
            "speed_kmh", "throttle", "brake", "steer", "reverse", "hand_brake",
        ])
        self.writer.writeheader()
        self.file.flush()

    def write(self, frame, state, vehicle, control):
        v = vehicle.get_velocity()
        speed_kmh = 3.6 * math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z)
        self.writer.writerow({
            "wall_time": datetime.now().isoformat(timespec="milliseconds"),
            "carla_frame": frame,
            "gps_timestamp": state.gps["timestamp"],
            "latitude": state.gps["lat"],
            "longitude": state.gps["lon"],
            "altitude": state.gps["alt"],
            "imu_timestamp": state.imu["timestamp"],
            "accel_x": state.imu["accel_x"],
            "accel_y": state.imu["accel_y"],
            "accel_z": state.imu["accel_z"],
            "gyro_x_deg_s": state.imu["gyro_x"],
            "gyro_y_deg_s": state.imu["gyro_y"],
            "gyro_z_deg_s": state.imu["gyro_z"],
            "compass_deg": state.imu["compass"],
            "speed_kmh": speed_kmh,
            "throttle": control.throttle,
            "brake": control.brake,
            "steer": control.steer,
            "reverse": control.reverse,
            "hand_brake": control.hand_brake,
        })
        self.file.flush()

    def close(self):
        self.file.close()


class ManualController:
    def __init__(self):
        self.reverse = False
        self.steer_cache = 0.0
        self.quit = False
        self.reset = False

    def parse_events(self, clock):
        self.reset = False
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.quit = True
            elif event.type == pygame.KEYUP:
                if event.key == K_ESCAPE:
                    self.quit = True
                elif event.key == K_q:
                    self.reverse = not self.reverse
                elif event.key == K_r:
                    self.reset = True

        keys = pygame.key.get_pressed()
        milliseconds = max(clock.get_time(), 1)

        control = carla.VehicleControl()
        control.manual_gear_shift = False
        control.reverse = self.reverse

        if keys[K_UP] or keys[K_w]:
            control.throttle = 1.0
            control.brake = 0.0
        elif keys[K_DOWN] or keys[K_s]:
            control.throttle = 0.0
            control.brake = 1.0
        else:
            control.throttle = 0.0
            control.brake = 0.0

        steer_increment = 0.0012 * milliseconds
        if keys[K_LEFT] or keys[K_a]:
            self.steer_cache -= steer_increment
        elif keys[K_RIGHT] or keys[K_d]:
            self.steer_cache += steer_increment
        else:
            self.steer_cache *= 0.65
            if abs(self.steer_cache) < 0.01:
                self.steer_cache = 0.0

        self.steer_cache = max(-1.0, min(1.0, self.steer_cache))
        control.steer = round(self.steer_cache, 3)
        control.hand_brake = bool(keys[K_SPACE])
        return control


class MultiCameraDisplay:
    def __init__(self, world, vehicle, width, height, gamma=2.2):
        self.width = width
        self.height = height
        self.single_width = width // 3
        self.single_height = height
        self.sensors = []
        self.surfaces = {name: None for name, *_ in CAMERA_TYPES}

        camera_transform = carla.Transform(carla.Location(x=1.6, z=1.65), carla.Rotation(pitch=0.0))
        bp_lib = world.get_blueprint_library()

        for name, bp_id, converter, _label in CAMERA_TYPES:
            bp = bp_lib.find(bp_id)
            bp.set_attribute("image_size_x", str(self.single_width))
            bp.set_attribute("image_size_y", str(self.single_height))
            bp.set_attribute("fov", "90")
            if bp.has_attribute("gamma"):
                bp.set_attribute("gamma", str(gamma))

            sensor = world.spawn_actor(bp, camera_transform, attach_to=vehicle, attachment_type=carla.AttachmentType.Rigid)
            weak_self = weakref.ref(self)
            sensor.listen(lambda image, n=name, c=converter: MultiCameraDisplay._parse_image(weak_self, image, n, c))
            self.sensors.append(sensor)

    @staticmethod
    def _parse_image(weak_self, image, name, converter):
        self = weak_self()
        if self is None:
            return
        image.convert(converter)
        array = np.frombuffer(image.raw_data, dtype=np.uint8)
        array = np.reshape(array, (image.height, image.width, 4))[:, :, :3]
        array = array[:, :, ::-1]
        self.surfaces[name] = pygame.surfarray.make_surface(array.swapaxes(0, 1))

    def render(self, display, font):
        x = 0
        for name, _bp_id, _converter, label in CAMERA_TYPES:
            surface = self.surfaces.get(name)
            if surface is not None:
                display.blit(surface, (x, 0))
            draw_label(display, font, label, x + 8, 8)
            x += self.single_width

    def destroy(self):
        for sensor in self.sensors:
            if sensor is not None and sensor.is_alive:
                sensor.stop()
                sensor.destroy()
        self.sensors = []


def draw_label(display, font, text, x, y):
    rendered = font.render(text, True, (255, 255, 255))
    bg = pygame.Surface((rendered.get_width() + 12, rendered.get_height() + 8))
    bg.set_alpha(150)
    bg.fill((0, 0, 0))
    display.blit(bg, (x, y))
    display.blit(rendered, (x + 6, y + 4))


def draw_text(display, font, lines, x=12, y=42):
    for line in lines:
        draw_label(display, font, line, x, y)
        y += font.get_height() + 10


def find_vehicle_blueprint(world, preferred_id):
    bp_lib = world.get_blueprint_library()
    matches = bp_lib.filter(preferred_id)
    if matches:
        return matches[0]

    fallback = bp_lib.filter("*mercedes*")
    if fallback:
        print(f"WARNING: '{preferred_id}' not found. Using '{fallback[0].id}' instead.")
        return fallback[0]

    raise RuntimeError(f"No Mercedes blueprint found. Tried: {preferred_id}")


def spawn_vehicle(world, blueprint_id):
    bp = find_vehicle_blueprint(world, blueprint_id)
    bp.set_attribute("role_name", "hero")
    if bp.has_attribute("color"):
        bp.set_attribute("color", random.choice(bp.get_attribute("color").recommended_values))

    spawn_points = list(world.get_map().get_spawn_points())
    random.shuffle(spawn_points)
    for spawn_point in spawn_points:
        vehicle = world.try_spawn_actor(bp, spawn_point)
        if vehicle is not None:
            vehicle.set_autopilot(False)
            vehicle.set_simulate_physics(True)
            vehicle.apply_control(carla.VehicleControl(throttle=0.0, brake=0.0, hand_brake=False, manual_gear_shift=False))
            print(f"Spawned: {bp.id}")
            return vehicle
    raise RuntimeError("Could not spawn vehicle. Try another map or clear existing actors.")


def attach_gps_imu(world, vehicle, state):
    sensors = []
    bp_lib = world.get_blueprint_library()

    gps_bp = bp_lib.find("sensor.other.gnss")
    gps = world.spawn_actor(gps_bp, carla.Transform(carla.Location(x=1.0, z=2.0)), attach_to=vehicle)
    gps.listen(lambda event: state.update_gps(event))
    sensors.append(gps)

    imu_bp = bp_lib.find("sensor.other.imu")
    imu = world.spawn_actor(imu_bp, carla.Transform(carla.Location(x=0.0, z=1.6)), attach_to=vehicle)
    imu.listen(lambda event: state.update_imu(event))
    sensors.append(imu)

    return sensors


def destroy_actors(camera_display, sensors, vehicle):
    if camera_display is not None:
        camera_display.destroy()
    for sensor in sensors:
        if sensor is not None and sensor.is_alive:
            sensor.stop()
            sensor.destroy()
    if vehicle is not None and vehicle.is_alive:
        vehicle.destroy()


def main():
    parser = argparse.ArgumentParser(description="Manual Mercedes Coupe 2020 with RGB/Depth/Semantic, GPS and IMU")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("-p", "--port", default=2000, type=int)
    parser.add_argument("--res", default="1440x480", help="Window resolution WIDTHxHEIGHT")
    parser.add_argument("--vehicle", default="vehicle.mercedes.coupe_2020")
    parser.add_argument("--csv", default="sensor_data.csv")
    parser.add_argument("--sync", action="store_true")
    parser.add_argument("--fps", default=30, type=int)
    args = parser.parse_args()

    width, height = [int(v) for v in args.res.lower().split("x")]
    width = (width // 3) * 3

    pygame.init()
    pygame.font.init()
    display = pygame.display.set_mode((width, height), pygame.HWSURFACE | pygame.DOUBLEBUF)
    pygame.display.set_caption("Click here first - Mercedes Coupe 2020 Manual Control")
    font = pygame.font.Font(pygame.font.get_default_font(), 18)
    clock = pygame.time.Clock()

    client = carla.Client(args.host, args.port)
    client.set_timeout(10.0)
    world = client.get_world()
    original_settings = world.get_settings()

    vehicle = None
    camera_display = None
    sensors = []
    logger = None

    try:
        if args.sync:
            settings = world.get_settings()
            settings.synchronous_mode = True
            settings.fixed_delta_seconds = 1.0 / float(args.fps)
            world.apply_settings(settings)

        vehicle = spawn_vehicle(world, args.vehicle)
        state = SensorState()
        sensors = attach_gps_imu(world, vehicle, state)
        camera_display = MultiCameraDisplay(world, vehicle, width, height)
        controller = ManualController()
        logger = CsvLogger(args.csv)

        print("IMPORTANT: Click the pygame window once, then drive with W/A/S/D or arrows.")
        print("Controls: W/S/A/D or arrows, Space handbrake, Q reverse, R respawn, ESC quit")
        print(f"Saving CSV to: {os.path.abspath(args.csv)}")

        frame = 0
        while True:
            clock.tick_busy_loop(args.fps if args.sync else 60)
            control = controller.parse_events(clock)
            if controller.quit:
                break

            if controller.reset:
                destroy_actors(camera_display, sensors, vehicle)
                vehicle = spawn_vehicle(world, args.vehicle)
                sensors = attach_gps_imu(world, vehicle, state)
                camera_display = MultiCameraDisplay(world, vehicle, width, height)
                controller.steer_cache = 0.0

            # The important fix: apply control before ticking/waiting for the next frame.
            vehicle.set_autopilot(False)
            vehicle.apply_control(control)

            if args.sync:
                frame = world.tick()
            else:
                snapshot = world.wait_for_tick()
                frame = snapshot.frame

            logger.write(frame, state, vehicle, control)

            display.fill((0, 0, 0))
            camera_display.render(display, font)

            v = vehicle.get_velocity()
            speed_kmh = 3.6 * math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z)
            draw_text(display, font, [
                "CLICK WINDOW FIRST | W/S/A/D or arrows | Space handbrake | Q reverse | R respawn | ESC quit",
                f"Speed: {speed_kmh:6.2f} km/h | throttle={control.throttle:.1f} brake={control.brake:.1f} steer={control.steer:.2f} reverse={control.reverse}",
                f"GPS: lat={state.gps['lat']:.8f}, lon={state.gps['lon']:.8f}, alt={state.gps['alt']:.2f}",
                f"IMU accel: x={state.imu['accel_x']:.3f}, y={state.imu['accel_y']:.3f}, z={state.imu['accel_z']:.3f} m/s²",
                f"IMU gyro: x={state.imu['gyro_x']:.3f}, y={state.imu['gyro_y']:.3f}, z={state.imu['gyro_z']:.3f} deg/s | compass={state.imu['compass']:.2f}°",
            ])
            pygame.display.flip()

    finally:
        if logger is not None:
            logger.close()
        destroy_actors(camera_display, sensors, vehicle)
        world.apply_settings(original_settings)
        pygame.quit()
        print("Done.")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nCancelled by user.")