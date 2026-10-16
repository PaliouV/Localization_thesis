import pygame 
import os
import json

HERE = os.path.dirname(os.path.abspath(__file__))   # ο φάκελος όπου είναι το minimap.py

def load_map():
    png_path = os.path.join(HERE, "town10_minimap.png")
    image = pygame.image.load(png_path)
    jsonpath = os.path.join(HERE, "town10_minimap.json")
    with open(jsonpath) as f:
        data = json.load(f)
    
    return image, data["min_x"], data["min_y"], data["pixels_per_meter"]


