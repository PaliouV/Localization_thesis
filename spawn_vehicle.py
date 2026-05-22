import time 
import random 
import carla 
import numpy as np
import cv2 as cv2

client = carla.Client('localhost', 2000)
client.set_timeout(10.0)
world = client.get_world()


blueprint_library = world.blueprint_library()
car = blueprint_library.find('vehicle.bmw.grandtourer')

spawn_points = world.get_map().get_spawn_points()
random_spawn = random.choice(spawn_points)

vehicle = world.try_spawn_actor(car, random_spawn)