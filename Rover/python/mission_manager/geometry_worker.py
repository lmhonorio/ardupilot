"""CPU-only worker: no ROS node, MAVLink connection or actuator commands."""
from docking_lidar.geometria import estimar, pontos_3d


def fit_and_sweep(xyz, prior, dimensions, base_height, max_points):
    points=pontos_3d(xyz,corrected=True)
    pose=estimar(points,prior,corrected=True,dimensions=dimensions,
                 base_height=base_height,max_points=max_points,prepared=True)
    return pose,points
