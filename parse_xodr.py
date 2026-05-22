import xml.etree.ElementTree as ET
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
from matplotlib.collections import PatchCollection
import math

def sample_road_geometry(geom, num_samples=200):
    """Sample (x,y) points from an OpenDRIVE geometry element."""
    x0 = float(geom.get('x'))
    y0 = float(geom.get('y'))
    hdg = float(geom.get('hdg'))
    length = float(geom.get('length'))
    s0 = float(geom.get('s'))

    line = geom.find('line')
    arc = geom.find('arc')
    spiral = geom.find('spiral')

    points = []
    ss = np.linspace(0, length, num_samples)

    if line is not None:
        for ds in ss:
            px = x0 + ds * math.cos(hdg)
            py = y0 + ds * math.sin(hdg)
            points.append((px, py))

    elif arc is not None:
        curvature = float(arc.get('curvature'))
        radius = 1.0 / curvature if curvature != 0 else 1e9
        for ds in ss:
            angle = hdg + ds * curvature
            px = x0 + radius * (math.sin(angle) - math.sin(hdg))
            py = y0 - radius * (math.cos(angle) - math.cos(hdg))
            points.append((px, py))

    else:  # spiral or unknown - approximate as line
        for ds in ss:
            px = x0 + ds * math.cos(hdg)
            py = y0 + ds * math.sin(hdg)
            points.append((px, py))

    return points


def parse_xodr(filepath):
    tree = ET.parse(filepath)
    root = tree.getroot()

    all_points = []

    for road in root.findall('road'):
        plan_view = road.find('planView')
        if plan_view is None:
            continue
        for geom in plan_view.findall('geometry'):
            pts = sample_road_geometry(geom)
            all_points.extend(pts)

    return np.array(all_points)


def create_occupancy_grid(points, resolution=1.0, road_width=4.0):
    """Create a 2D occupancy grid from road centerline points."""
    xs, ys = points[:, 0], points[:, 1]

    x_min, x_max = xs.min() - 20, xs.max() + 20
    y_min, y_max = ys.min() - 20, ys.max() + 20

    grid_w = int((x_max - x_min) / resolution)
    grid_h = int((y_max - y_min) / resolution)

    grid = np.zeros((grid_h, grid_w), dtype=np.uint8)  # 0 = obstacle

    half_w = int(road_width / resolution)

    for x, y in zip(xs, ys):
        gx = int((x - x_min) / resolution)
        gy = int((y - y_min) / resolution)
        # Mark road as free (1)
        y1 = max(0, gy - half_w)
        y2 = min(grid_h, gy + half_w)
        x1 = max(0, gx - half_w)
        x2 = min(grid_w, gx + half_w)
        grid[y1:y2, x1:x2] = 1

    origin = (x_min, y_min)
    return grid, origin, resolution


if __name__ == '__main__':
    print("Parsing Town10HD_Opt.xodr ...")
    points = parse_xodr('Town10HD_Opt.xodr')
    print(f"  → {len(points)} road centerline points")

   # Άλλαξε αυτές τις παραμέτρους:
    grid, origin, res = create_occupancy_grid(points, resolution=0.25, road_width=3.5)
    print(f"  → Grid size: {grid.shape} | Origin: {origin} | Resolution: {res}m")

    # Save
    np.save('town10_grid.npy', grid)
    np.save('town10_meta.npy', np.array([origin[0], origin[1], res]))
    print("  → Saved: town10_grid.npy, town10_meta.npy")

    # Visualize
    plt.figure(figsize=(12, 12))
    plt.imshow(grid, cmap='gray', origin='lower')
    plt.title('Town10 Occupancy Grid')
    plt.colorbar(label='0=obstacle, 1=free')
    plt.savefig('town10_occupancy.png', dpi=150, bbox_inches='tight')
    plt.show()
    print("  → Saved: town10_occupancy.png")