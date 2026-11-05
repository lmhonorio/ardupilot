"""Projection of a MID360 cloud in aligned ROS FLU coordinates; unknown is NaN."""
import numpy as np


def project(points, offset=(-.003, 0., .434), height=(-.08, .9),
            self_box=(.30, .26), min_range=.5, max_range=25., sectors=72):
    p = np.asarray(points, dtype=float).reshape(-1, 3)
    if sectors != 72 or min_range <= 0 or max_range <= min_range:
        raise ValueError('Invalid sector or range configuration')
    p = p[np.isfinite(p).all(axis=1) & (np.linalg.norm(p, axis=1) > .001)]
    p = p + np.asarray(offset)
    r = np.hypot(p[:, 0], p[:, 1])
    outside_self = (np.abs(p[:, 0]) > self_box[0]) | (np.abs(p[:, 1]) > self_box[1])
    # Points closer than minimum are conservatively represented at minimum;
    # dropping them would erase a real close obstacle.
    keep = outside_self & (p[:, 2] >= height[0]) & (p[:, 2] <= height[1]) & (r <= max_range)
    p, r = p[keep], r[keep]
    index = np.floor((np.arctan2(p[:, 1], p[:, 0]) + np.pi/sectors) * sectors/(2*np.pi)).astype(int) % sectors
    result = np.full(sectors, np.inf)
    np.minimum.at(result, index, np.maximum(r, min_range))
    result[~np.isfinite(result)] = np.nan
    return result


def mav_distances(ranges, min_range=.5, max_range=25.):
    """ROS CCW bins -> MAVLink BODY_FRD clockwise, unknown=UINT16_MAX."""
    a = np.asarray(ranges, dtype=float)
    if a.shape != (72,): raise ValueError('Expected 72 distances')
    out = []
    for i in range(72):
        d = a[(-i) % 72]
        out.append(int(round(max(min_range, min(d, max_range))*100)) if np.isfinite(d) and d >= 0 else 65535)
    return out
