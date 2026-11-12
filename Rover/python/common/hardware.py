"""Explicit calibration for companion hardware; simulator defaults remain isolated."""
import os
from pathlib import Path
import numpy as np
import yaml


def calibration(path=None):
    path=path or os.environ.get('ROVER_HARDWARE_CALIBRATION')
    if not path:return None
    data=yaml.safe_load(Path(path).read_text())
    if not data.get('validated',False):raise ValueError('Calibracao fisica ainda nao validada')
    for name in ('base_to_imu','lidar_to_imu_translation'):
        a=np.asarray(data[name],dtype=float)
        if a.shape!=(3,) or not np.isfinite(a).all():raise ValueError('Translacao invalida: '+name)
    r=np.asarray(data['lidar_to_imu_rotation'],dtype=float).reshape(3,3)
    if not np.isfinite(r).all() or not np.allclose(r.T@r,np.eye(3),atol=1e-5) or abs(np.linalg.det(r)-1)>1e-5:
        raise ValueError('Rotacao LiDAR/IMU invalida')
    if not data.get('imu_axes_aligned_to_base',False):
        raise ValueError('Este perfil exige eixos da IMU alinhados aos eixos da base')
    return data


def base_to_imu():
    c=calibration()
    return np.asarray(c['base_to_imu'] if c else [-.003,0.,.434],dtype=float)
