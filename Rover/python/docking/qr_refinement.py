"""Subpixel registration of the decoded station QR texture (not just 4 corners)."""
from pathlib import Path
import cv2
import numpy as np


class QRRefiner:
    def __init__(self, payload):
        path=Path(__file__).resolve().parents[2]/'ros2/assets/models/docking_station/materials/textures/docking_qr.png'
        ref=cv2.imread(str(path),0) if path.exists() and payload=='ROVER_DOCKING_STATION_01' else None
        if ref is None:ref=cv2.QRCodeEncoder_create().encode(payload)
        y,x=np.where(ref<128)
        ref=ref[y.min():y.max()+1,x.min():x.max()+1]
        self.size=160
        self.template=cv2.resize(ref,(self.size,self.size),interpolation=cv2.INTER_AREA).astype(np.float32)/255.
        self.corners=np.float32([[0,0],[self.size-1,0],[self.size-1,self.size-1],[0,self.size-1]])

    def refine(self, frame, corners):
        p=np.asarray(corners,dtype=np.float32).reshape(4,2)
        side=float(np.ptp(p,axis=0).max())
        if side<40:return corners,0.
        gray=cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY) if frame.ndim==3 else frame
        H=cv2.getPerspectiveTransform(p,self.corners)
        warped=cv2.warpPerspective(gray,H,(self.size,self.size)).astype(np.float32)/255.
        try:
            cc,w=cv2.findTransformECC(self.template,warped,np.eye(3,dtype=np.float32),
                cv2.MOTION_HOMOGRAPHY,(cv2.TERM_CRITERIA_COUNT|cv2.TERM_CRITERIA_EPS,40,1e-5),None,3)
            result=cv2.perspectiveTransform(self.corners.reshape(1,4,2),np.linalg.inv(H)@w)
        except (cv2.error,np.linalg.LinAlgError):return corners,0.
        if (cc<.90 or not np.isfinite(result).all()
                or np.max(np.linalg.norm(result.reshape(4,2)-p,axis=1))>max(3.,side*.04)):
            return corners,float(cc)
        return result,float(cc)
