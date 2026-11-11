"""Temporal validation of the fixed docking wall, independent of transport.

Filter wall orientation in an inertial yaw frame, not in the rotating rover
frame. A rejected measurement never becomes a motion command.
"""
from collections import deque
import math
import statistics


def wrap(a):
    return (a+math.pi)%(2*math.pi)-math.pi


class PosePending(RuntimeError):
    """Stop and acquire another fresh image; do not search or advance."""


class PoseTracker:
    def __init__(self, wall_offset=.499):
        self.offset=wall_offset
        self.samples=deque(maxlen=15)
        self.last_stamp=None
        self.axis=None
        self.last_marker=None
        self.accepted=0

    def update(self, raw, stamp, yaw):
        values=[stamp,yaw,raw['normal_yaw'],raw['marker_forward'],raw['marker_right']]
        if not all(math.isfinite(v) for v in values):
            raise PosePending('Pose/atitude nao finita')
        if self.last_stamp is not None and stamp<=self.last_stamp:
            if stamp<self.last_stamp:
                raise RuntimeError('Relogio retrocedeu; reinicie o docking')
            raise PosePending('Aguardando nova imagem (timestamp repetido)')
        self.last_stamp=stamp
        # normal_yaw increases with vehicle yaw; the wall itself is stationary.
        axis=wrap(raw['normal_yaw']-yaw)
        if self.axis is not None and abs(wrap(axis-self.axis))>math.radians(10):
            raise PosePending('Orientacao QR inconsistente com atitude: salto >10 graus')
        mf,mr=raw['marker_forward'],raw['marker_right']
        if self.last_marker is not None:
            t,f,r,old_yaw=self.last_marker
            dt=stamp-t;dy=wrap(yaw-old_yaw)
            predicted_f=math.cos(dy)*f+math.sin(dy)*r
            predicted_r=-math.sin(dy)*f+math.cos(dy)*r
            if dt<1 and math.hypot(mf-predicted_f,mr-predicted_r)>.08+.25*dt:
                raise PosePending('Posicao QR mudou mais que o movimento permitido')
        self.samples.append((stamp,axis))
        while self.samples and stamp-self.samples[0][0]>2.:
            self.samples.popleft()
        if len(self.samples)<5 or stamp-self.samples[0][0]<.6:
            raise PosePending('Confirmando pose em pelo menos 5 imagens / 0,6 s')
        reference=self.axis if self.axis is not None else self.samples[0][1]
        offsets=[wrap(a-reference) for _,a in self.samples]
        middle=statistics.median(offsets)
        mad=statistics.median(abs(a-middle) for a in offsets)
        if mad>math.radians(4):
            raise PosePending('Orientacao QR ainda dispersa; aguardando estabilidade')
        if abs(wrap(axis-reference)-middle)>math.radians(8):
            raise PosePending('Amostra QR fora do consenso temporal (>8 graus)')
        estimate=wrap(reference+middle)
        if self.axis is None:
            self.axis=estimate
        else:
            # The fixed wall cannot rotate when the vehicle turns. Slow updates
            # prevent a one-frame PnP ambiguity from rotating the staging point.
            self.axis=wrap(self.axis+.12*wrap(estimate-self.axis))
        self.last_marker=(stamp,mf,mr,yaw)
        heading=wrap(self.axis+yaw)
        result=dict(raw)
        result.update(normal_yaw=heading,
                      forward=mf-self.offset*math.cos(heading),
                      right=mr+self.offset*math.sin(heading),
                      raw_normal_yaw=raw['normal_yaw'],
                      heading_mad=mad, stamp=stamp)
        self.accepted+=1
        return result
