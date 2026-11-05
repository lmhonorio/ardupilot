"""QR camera pose -> docking target relative to rover, simulation calibration."""
import math
import cv2
import numpy as np

CODE_SIZE_M = .36 * (397.0/592.0)


def solve(corners, camera_matrix, distortion=None, tilt_rad=-.3, wall_offset=.5335):
    p=np.asarray(corners,dtype=np.float64).reshape(4,2)
    K=np.asarray(camera_matrix,dtype=np.float64).reshape(3,3)
    if not np.isfinite(p).all() or not np.isfinite(K).all() or K[0,0]<=0:
        raise ValueError('Camera ou pontos invalidos')
    h=CODE_SIZE_M/2
    obj=np.array([[-h,h,0],[h,h,0],[h,-h,0],[-h,-h,0]],dtype=np.float64)
    ok,rvec,tvec=cv2.solvePnP(obj,p,K,np.zeros(5) if distortion is None else np.asarray(distortion,dtype=float),flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok: raise ValueError('Pose QR indisponivel')
    proj,_=cv2.projectPoints(obj,rvec,tvec,K,np.zeros(5) if distortion is None else np.asarray(distortion,dtype=float))
    error=float(np.sqrt(np.mean((proj.reshape(4,2)-p)**2)))
    t=tvec.reshape(3);R,_=cv2.Rodrigues(rvec);n=R[:,2]
    if t[2]<=0 or not np.isfinite(t).all() or error>4: raise ValueError('Pose QR inconsistente')
    if np.dot(n,t)>0:n=-n
    # Plane normal points from QR towards the rover. The desired target is
    # wall_offset metres in front of QR, inside the docking box.
    destination=t+wall_offset*n
    def optical_to_body(v):
        # optical right/down/forward -> ROS camera forward/left/up, then tilt.
        f,l,u=v[2],-v[0],-v[1]
        return np.array([math.cos(tilt_rad)*f+math.sin(tilt_rad)*u,l,
                         -math.sin(tilt_rad)*f+math.cos(tilt_rad)*u])
    # base-to-pan x=.05; camera is 0.07 m below tilt joint at z=.25.
    cam=np.array([.05-.07*math.sin(tilt_rad),0,.25-.07*math.cos(tilt_rad)])
    pos=optical_to_body(destination)+cam
    marker=optical_to_body(t)+cam
    normal=optical_to_body(n)
    # Convert ROS FLU to body FRD.
    return {'forward':float(pos[0]),'right':float(-pos[1]),'down':float(-pos[2]),
            'marker_forward':float(marker[0]),'marker_right':float(-marker[1]),
            'normal_yaw':math.atan2(-normal[1],-normal[0]),'reprojection_px':error,
            'camera_range':float(np.linalg.norm(t))}
