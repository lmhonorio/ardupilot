"""Upright QR pose with measured pan/tilt, for the vertical docking wall."""
import math
import cv2
import numpy as np
from scipy.optimize import least_squares

CODE_SIZE_M = .36 * (397.0/592.0)


def camera_transform(pan, tilt):
    cp,sp=math.cos(pan),math.sin(pan);ct,st=math.cos(tilt),math.sin(tilt)
    rz=np.array([[cp,-sp,0],[sp,cp,0],[0,0,1.]])
    ry=np.array([[ct,0,st],[0,1.,0],[-st,0,ct]])
    optical=np.array([[0,0,1.],[-1.,0,0],[0,-1.,0]])
    # URDF: base->pan (.05,0,.08), pan->tilt (0,0,.17),
    # tilt->camera (0,0,-.17), sensor pose (0,0,.10).
    center=np.array([.05,0,0])+rz@(np.array([0,0,.25])+ry@np.array([0,0,-.07]))
    return rz@ry@optical,center


def project_marker(marker, heading, K, distortion, pan=0., tilt=-.3):
    rotation,center=camera_transform(pan,tilt)
    right=np.array([math.sin(heading),-math.cos(heading),0.])
    up=np.array([0.,0.,1.]);h=CODE_SIZE_M/2
    body=np.array([marker-h*right+h*up,marker+h*right+h*up,
                   marker+h*right-h*up,marker-h*right-h*up])
    optical=(body-center)@rotation
    pixels,_=cv2.projectPoints(optical,np.zeros(3),np.zeros(3),K,distortion)
    return pixels.reshape(4,2),optical


def solve(corners, camera_matrix, distortion=None, tilt_rad=-.3, wall_offset=.499,
          pan_rad=0., heading_prior=None):
    p=np.asarray(corners,dtype=np.float64).reshape(4,2)
    K=np.asarray(camera_matrix,dtype=np.float64).reshape(3,3)
    D=np.zeros(5) if distortion is None else np.asarray(distortion,dtype=float)
    if (not np.isfinite(p).all() or not np.isfinite(K).all() or K[0,0]<=0
            or K[1,1]<=0 or not np.isfinite(D).all()):
        raise ValueError('Camera ou pontos invalidos')
    h=CODE_SIZE_M/2
    obj=np.array([[-h,h,0],[h,h,0],[h,-h,0],[-h,-h,0]],dtype=np.float64)
    ok,rv,tv=cv2.solvePnP(obj,p,K,D,flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok or tv[2,0]<=0:raise ValueError('Pose QR indisponivel')
    rotation,center=camera_transform(pan_rad,tilt_rad)
    initial=rotation@tv.reshape(3)+center
    normal=rotation@cv2.Rodrigues(rv)[0][:,2]
    if np.dot(normal,initial-center)>0:normal=-normal
    heading=math.atan2(-normal[1],-normal[0])
    # The wall is vertical, so do not allow PnP noise to tilt its normal up/down.
    # A weak orientation prior resolves distant planar ambiguity; pixels still
    # determine the pose and dominate when the marker grows in the image.
    def residual(x):
        projected,optical=project_marker(x[:3],x[3],K,D,pan_rad,tilt_rad)
        delta=(projected-p).ravel()
        if np.any(optical[:,2]<=.05):delta=delta+1000
        if heading_prior is not None:
            dh=(x[3]-heading_prior+math.pi)%(2*math.pi)-math.pi
            delta=np.append(delta,.4*dh/math.radians(15))
        return delta
    starts=[heading]
    if heading_prior is not None:starts.append(heading_prior)
    fits=[least_squares(residual,np.r_[initial,angle],loss='soft_l1',f_scale=.75,
                        max_nfev=35) for angle in starts]
    fit=min(fits,key=lambda f:f.cost)
    marker=fit.x[:3];heading=(fit.x[3]+math.pi)%(2*math.pi)-math.pi
    projected,optical=project_marker(marker,heading,K,D,pan_rad,tilt_rad)
    error=float(np.sqrt(np.mean((projected-p)**2)))
    if (not np.isfinite(fit.x).all() or np.any(optical[:,2]<=0) or error>2
            or np.linalg.norm(marker-center)>12):
        raise ValueError('Pose QR vertical inconsistente')
    target=marker-wall_offset*np.array([math.cos(heading),math.sin(heading),0.])
    return dict(forward=float(target[0]),right=float(-target[1]),down=float(-target[2]),
                marker_forward=float(marker[0]),marker_right=float(-marker[1]),
                normal_yaw=heading,reprojection_px=error,
                camera_range=float(np.linalg.norm(marker-center)))
