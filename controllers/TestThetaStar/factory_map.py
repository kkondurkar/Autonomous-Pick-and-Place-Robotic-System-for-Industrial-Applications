# factory_map.py

import re
from pathlib import Path
import math
import matplotlib
import numpy as np


from pathlib import Path

THIS_FILE = Path(__file__).resolve()
PROJECT_ROOT = THIS_FILE.parents[2]

"""
def add_obstacles_from_wbt(grid_map, wbt_path, min_size=0.05):
    ""
    Parse simple Box obstacles from a Webots .wbt file and add them
    as rectangular X-Y footprints to the occupancy grid.

    Assumes:
      planner x = Webots X
      planner y = Webots Y
    ""
    text = Path(wbt_path).read_text(encoding="utf-8", errors="ignore")

    blocks = re.findall(
        r"(?:Solid|Transform)\s*\{.*?\n\}",
        text,
        flags=re.DOTALL
    )

    count = 0

    for block in blocks:
        t = re.search(
            r"translation\s+([-.\deE]+)\s+([-.\deE]+)\s+([-.\deE]+)",
            block
        )

        s = re.search(
            r"Box\s*\{\s*size\s+([-.\deE]+)\s+([-.\deE]+)\s+([-.\deE]+)",
            block,
            flags=re.DOTALL
        )

        if not t or not s:
            continue

        cx = float(t.group(1))  # Webots X
        cy = float(t.group(2))  # Webots Y

        sx = float(s.group(1))  # size in X
        sy = float(s.group(2))  # size in Y

        if sx < min_size or sy < min_size:
            continue

        x_min = cx - sx / 2.0
        x_max = cx + sx / 2.0
        y_min = cy - sy / 2.0
        y_max = cy + sy / 2.0

        grid_map.add_rect_obstacle(x_min, y_min, x_max, y_max)
        count += 1

    print(f"Added {count} rectangular obstacles from {wbt_path}")
"""

def add_rotated_box_as_aabb(grid_map, size, translation, rotation_z):
    """
    Adds a rotated rectangular box footprint as an axis-aligned bounding box.

    Coordinates:
      planner x = Webots X
      planner y = Webots Y

    size = [sx, sy, sz]
    translation = [cx, cy, cz]
    rotation_z = radians about Webots Z axis
    """
    sx, sy, _ = size
    cx, cy, _ = translation

    c = abs(math.cos(rotation_z))
    s = abs(math.sin(rotation_z))

    # Axis-aligned bounding box dimensions after rotation
    footprint_x = sx * c + sy * s
    footprint_y = sx * s + sy * c

    x_min = cx - footprint_x / 2.0
    x_max = cx + footprint_x / 2.0
    y_min = cy - footprint_y / 2.0
    y_max = cy + footprint_y / 2.0

    grid_map.add_rect_obstacle(x_min, y_min, x_max, y_max)


def build_factory_map(grid_map):
    """
    Build the occupancy map for the Webots factory world.
    """

    # Adjust this path if needed.
    wbt_path = PROJECT_ROOT / "worlds" / "factory3.wbt"

    # Outer walls
    add_rotated_box_as_aabb(
        grid_map,
        size=[16.5, 0.2, 7.0],          # relative size of object, [x y z] dimensions
        translation=[10.0, -4.3, 0.0],  # center of object (in X-Y plane) relative to 
                                        #  center of floor [0.0 -4.3 0.0], [x y] coordinates (2D map-->ignore z coordinate)
        rotation_z=-1.5708              # rotation about z-axis in radians
    )

    add_rotated_box_as_aabb(
        grid_map,
        size=[20.0, 0.2, 1.0],
        translation=[0.0, 3.9, 0.0],
        rotation_z=0.0
    )

    add_rotated_box_as_aabb(
        grid_map,
        size=[20.0, 0.2, 1.0],
        translation=[0.0, -12.5, 0.0],
        rotation_z=0.0
    )

    add_rotated_box_as_aabb(
        grid_map,
        size=[16.5, 0.2, 1.0],
        translation=[-10.0, -4.3, 0.0],
        rotation_z=1.5708
    )

    # Shelves / racks


    # Boxes / work areas
    
    add_rotated_box_as_aabb(
        grid_map,
        size=[0.6, 0.6, 0.6],
        translation=[1.90, -3.80, 0.30],
        rotation_z=0.0
    )

    add_rotated_box_as_aabb(
        grid_map,
        size=[0.6, 0.6, 0.6],
        translation=[0.84, -0.90, 0.90],
        rotation_z=0.0  
    )
    
    add_rotated_box_as_aabb(
        grid_map,
        size=[0.6, 0.6, 0.6],
        translation=[-1.42, -3.36, 0.30],
        rotation_z=0.0
    )
    
    #add_obstacles_from_wbt(grid_map, wbt_path)
    
def visualize_factory_map(grid_map, path_world=None, start=None, goal=None):

    import matplotlib.pyplot as plt
    grid = np.array(grid_map.grid)

    extent = [
        grid_map.origin_x,
        grid_map.origin_x + grid_map.width * grid_map.resolution,
        grid_map.origin_y,
        grid_map.origin_y + grid_map.height * grid_map.resolution,
    ]

    plt.figure(figsize=(8, 8))
    plt.imshow(
        grid,
        origin="lower",
        extent=extent,
        interpolation="nearest",
        cmap="gray_r"
    )

    if path_world:
        xs = [p[0] for p in path_world]
        ys = [p[1] for p in path_world]
        plt.plot(xs, ys, linewidth=2, label="Theta* path")

    if start:
        plt.scatter(start[0], start[1], marker="o", s=80, label="Start")

    if goal:
        plt.scatter(goal[0], goal[1], marker="x", s=100, label="Goal")

    plt.xlabel("Webots / Planner X-axis")
    plt.ylabel("Webots / Planner Y-axis")
    plt.title("Generated Factory Occupancy Map")
    plt.axis("equal")
    plt.grid(True)
    plt.legend()
    plt.show()
#    plt.savefig("factory_map_debug.png", dpi=200)
#    plt.close()