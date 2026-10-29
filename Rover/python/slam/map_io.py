"""Export OccupancyGrid as the PGM/YAML pair understood by ROS map_server."""
import math
from pathlib import Path

import numpy as np
import yaml


def occupancy_image(grid):
    cells = np.asarray(grid.data, dtype=np.int16).reshape(grid.info.height, grid.info.width)
    image = np.full(cells.shape, 205, dtype=np.uint8)
    image[(cells >= 0) & (cells <= 19)] = 254
    image[cells >= 65] = 0
    return image


def save_map(grid, directory):
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    image = occupancy_image(grid)
    pgm = directory / 'map.pgm'
    pgm.write_bytes(('P5\n%d %d\n255\n' % (grid.info.width, grid.info.height)).encode('ascii')
                    + np.flipud(image).tobytes())
    p, q = grid.info.origin.position, grid.info.origin.orientation
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y*q.y + q.z*q.z))
    metadata = {'image': 'map.pgm', 'resolution': float(grid.info.resolution),
                'origin': [float(p.x), float(p.y), float(yaw)], 'negate': 0,
                'occupied_thresh': 0.65, 'free_thresh': 0.196}
    (directory / 'map.yaml').write_text(yaml.safe_dump(metadata, sort_keys=False))
    return pgm, directory / 'map.yaml'
