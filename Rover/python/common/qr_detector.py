"""Decode small rendered QR codes without changing the camera calibration."""
import cv2
import numpy as np


class QRDetector:
    def __init__(self):
        self.detector = cv2.QRCodeDetector()
        self.status = 'QR ainda nao avaliado'

    def detectAndDecode(self, frame):
        """Return corners in ORIGINAL image pixels, including after upsampling.

        A detected square alone never authorizes docking: the QR payload must
        decode successfully and the caller must match it against the station.
        """
        text, points, straight = self.detector.detectAndDecode(frame)
        if text:
            self.status = 'QR decodificado na imagem original'
            return text, points, straight
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if frame.ndim == 3 else frame
        if points is None:
            # Only detection uses the enlarged full frame. Decoding is limited
            # to a small region, avoiding repeated large image processing.
            scale = min(3., 1920./max(gray.shape))
            enlarged = cv2.resize(gray, None, fx=scale, fy=scale, interpolation=cv2.INTER_CUBIC)
            found, candidate = self.detector.detect(enlarged)
            if found and candidate is not None:
                sx, sy = enlarged.shape[1]/gray.shape[1], enlarged.shape[0]/gray.shape[0]
                points = (candidate+.5)/np.array([sx,sy])-.5
        if points is None:
            self.status = 'QR nao localizado na imagem'
            return '', None, None
        p = np.asarray(points).reshape(4,2)
        side = float(np.ptp(p,axis=0).max())
        self.status = 'QR localizado (~%.0f px), mas conteudo nao decodificado' % side
        # Generous context/quiet zone improves OpenCV's binarization at ~1 pixel/module.
        lo = np.maximum(np.floor(p.min(axis=0)-.6*side).astype(int),0)
        hi = np.minimum(np.ceil(p.max(axis=0)+.6*side).astype(int),[gray.shape[1],gray.shape[0]])
        crop = gray[lo[1]:hi[1],lo[0]:hi[0]]
        if crop.size == 0 or side < 15 or side > 180:
            return '', points, None
        for requested_scale in (6.,8.):
            scale = min(requested_scale,800./max(crop.shape))
            enlarged = cv2.resize(crop,None,fx=scale,fy=scale,interpolation=cv2.INTER_CUBIC)
            sx, sy = enlarged.shape[1]/crop.shape[1], enlarged.shape[0]/crop.shape[0]
            for image in (enlarged, cv2.threshold(enlarged,0,255,cv2.THRESH_BINARY|cv2.THRESH_OTSU)[1]):
                text, corners, straight = self.detector.detectAndDecode(image)
                if text and corners is not None:
                    corners = ((corners+.5)/np.array([sx,sy])-.5+lo).astype(np.float32)
                    self.status = 'QR decodificado por ampliacao local (~%.0f px originais)' % side
                    return text, corners, straight
        return '', points, None
