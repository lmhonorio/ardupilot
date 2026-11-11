"""QR-guided docking with turn/translate separation for the skid-steer rover."""
import math


def clamp(value, low, high):
    return min(max(value, low), high)


def pivot_rate(heading, gain=.65):
    return -math.copysign(clamp(abs(heading)*gain,.12,.22),heading)


def distancia_frenagem(speed):
    """Harmonic braking distance, including pose and command latency margins."""
    if speed is None or not math.isfinite(speed) or speed < 0:
        return .30
    return clamp(.025 + .2*speed + speed*speed/(2*.3), .025, .30)


def deve_frear(pose, speed):
    forward=pose.get('forward')
    if forward is None or not math.isfinite(forward):
        raise ValueError('Distancia visual invalida')
    return forward <= distancia_frenagem(speed)


class VisualAlignment:
    """Reach an outside staging point, turn onto the axis, then enter.

    Skid friction prevented small yaw-rate corrections during translation.
    Stop before correcting heading instead of accumulating lateral overshoot.
    The pan camera follows the QR during those turns.
    """
    def __init__(self, speed=.25, staging_distance=1.2, allow_rear_target=False, pivot_speed_tolerance=.04):
        self.speed=speed
        self.staging_distance=staging_distance
        self.allow_rear_target=allow_rear_target
        self.pivot_speed_tolerance=pivot_speed_tolerance
        self.phase='aproximacao'
        self.stable_since=None
        self.violation_since=None
        self.time=0.
        self.previous_speed=0.
        self.retreats=0
        self.turning=False
        self.settling=False
        self.settled_since=None

    def _orient(self,error,pose,now,tolerance=math.radians(2)):
        """Zero translation during pivot; brake angular motion before the target."""
        rate=pose.get('yawrate',0.)
        speed=pose.get('groundspeed',0.)
        if abs(error)>tolerance*2:self.turning=True
        if not self.turning:return True,0.
        if speed>self.pivot_speed_tolerance:return False,0.
        predicted=abs(rate)*.25+math.radians(1)
        if abs(error)<=tolerance or (error*rate>0 and abs(error)<predicted):
            self.settling=True
        if self.settling:
            if abs(rate)<.025:
                if self.settled_since is None:self.settled_since=now
                if now-self.settled_since>=.4:
                    self.settling=False;self.settled_since=None
                    if abs(error)<=tolerance*1.5:
                        self.turning=False
                        return True,0.
            else:self.settled_since=None
            return False,0.
        return False,math.copysign(clamp(abs(error)*.8,.20,.25),error)

    def step(self, pose, now=None):
        current=self.time+.1 if now is None else now
        dt=clamp(current-self.time,0.,.3);self.time=current
        f=pose['forward'];r=pose['right'];heading=pose['normal_yaw']
        if not all(math.isfinite(v) for v in (f,r,heading)):
            raise ValueError('Pose visual invalida')
        along=f*math.cos(heading)-r*math.sin(heading)
        lateral=f*math.sin(heading)+r*math.cos(heading)
        # A 360-degree LiDAR may see the station behind the rover before its
        # initial pivot. Overshoot is then measured along the STATION axis.
        if math.hypot(f,r)>9 or (along if self.allow_rear_target else f)<-.03:
            raise ValueError('Estacao fora da zona permitida ou alvo ultrapassado')
        def output(v,w,done=False):
            if v>self.previous_speed and v>0:
                v=min(v,max(0.,self.previous_speed)+.25*dt)
            self.previous_speed=v
            return self.phase,v,w,done
        if self.phase=='aproximacao':
            remaining=along-self.staging_distance
            # This is only the outside staging gate, not permission to enter.
            # Verify lateral accuracy again after facing the wall squarely.
            if along<=self.staging_distance+.10 and abs(lateral)<.10:
                self.phase='alinhar';self.stable_since=None;self.turning=True
            elif remaining<.04:
                if along<.75 or self.retreats>=2:
                    raise ValueError('Sem espaco para realinhar antes da entrada')
                self.phase='recuar_alinhamento';self.retreats+=1;self.turning=True
            else:
                desired=math.atan2(lateral,remaining)
                ready,rate=self._orient(desired-heading,pose,current)
                if not ready:return output(0.,rate)
                return output(min(self.speed,max(.035,remaining*.4)),0.)
        if self.phase=='recuar_alinhamento':
            ready,rate=self._orient(-heading,pose,current)
            if not ready:return output(0.,rate)
            if along>=self.staging_distance+1.:
                self.phase='aproximacao';self.turning=True
                return output(0.,0.)
            return output(-min(.10,self.speed),0.)
        if self.phase=='alinhar':
            ready,rate=self._orient(-heading,pose,current,math.radians(1.5))
            if not ready:
                self.stable_since=None
                return output(0.,rate)
            if pose.get('groundspeed',0.)>.04:
                self.stable_since=None
                return output(0.,0.)
            if self.stable_since is None:self.stable_since=current
            if current-self.stable_since<.6:return output(0.,0.)
            if abs(lateral)>.03:
                if along<.75 or self.retreats>=2:
                    raise ValueError('Erro lateral confirmado fora da entrada')
                self.phase='recuar_alinhamento';self.retreats+=1
                return output(0.,0.)
            self.phase='entrada'
        if self.phase=='entrada':
            if abs(lateral)>.20 or abs(heading)>math.radians(25):
                if self.violation_since is None:self.violation_since=current
                if current-self.violation_since>=1.:
                    raise ValueError('Alinhamento fora do limite confirmado: lateral=%.3f m angulo=%.1f graus' %
                                     (lateral,math.degrees(heading)))
                return output(0.,0.)
            self.violation_since=None
            if abs(f)<=.015 and abs(r)<=.025 and abs(heading)<=math.radians(2):
                return output(0.,0.,True)
            desired=clamp(math.atan2(lateral,.60),-math.radians(5),math.radians(5))
            desired*=clamp((along-.15)/.60,0.,1.)
            tolerance=math.radians(.75 if along<.20 else 1.5)
            ready,rate=self._orient(desired-heading,pose,current,tolerance)
            if not ready:return output(0.,rate)
            return output(min(self.speed,.18,max(.035,along*.22)),0.)
        raise ValueError('Fase de aproximacao invalida')
