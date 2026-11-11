"""Planejamento local de aproximacao usando a pose do QR no referencial FRD."""
import math


def clamp(value, low, high):
    return min(max(value, low), high)


def distancia_frenagem(speed):
    """Margem conservadora para latencia e desaceleracao estimada no SITL."""
    if speed is None or not math.isfinite(speed) or speed < 0:
        speed=.2
    return clamp(.05 + .4*speed + speed*speed/(2*.15), .05, .30)


def deve_frear(pose, speed):
    """Nao manda correcao depois de entrar na zona de parada longitudinal."""
    forward=pose.get('forward')
    if forward is None or not math.isfinite(forward):
        raise ValueError('Distancia visual invalida')
    return forward <= distancia_frenagem(speed)


class VisualAlignment:
    """Alcanca um ponto fora da entrada, alinha o rumo e segue ate o centro."""
    def __init__(self, speed=.08, staging_distance=1.8):
        self.speed=speed
        self.staging_distance=staging_distance
        self.phase='aproximacao'

    def step(self, pose):
        f=pose['forward']; r=pose['right']; heading=pose['normal_yaw']
        if not all(math.isfinite(v) for v in (f,r,heading)):
            raise ValueError('Pose visual invalida')
        if math.hypot(f,r)>9 or f<-.015:
            raise ValueError('Estacao fora da zona permitida ou alvo ultrapassado')
        # solve() reports the opposite sign to BODY_NED yaw rate.
        c=math.cos(heading); s=-math.sin(heading)
        along=f*c+r*s
        lateral=-f*s+r*c
        if self.phase=='aproximacao':
            stage_f=f-self.staging_distance*c
            stage_r=r-self.staging_distance*s
            dist=math.hypot(stage_f,stage_r)
            if ((dist<.12 or .30<along<self.staging_distance+.12) and abs(lateral)<.12):
                self.phase='alinhar'
            else:
                bearing=math.atan2(stage_r,stage_f)
                # Nao dirigir para tras nem prosseguir quando o ponto esta lateral.
                speed=min(self.speed,dist*.15) if abs(bearing)<math.radians(18) else 0.
                return self.phase,speed,clamp(bearing*.8,-.22,.22),False
        if self.phase=='alinhar':
            # A correcao final com giro so e segura se estiver na linha de entrada.
            if abs(lateral)>.12:
                self.phase='aproximacao'
                return self.phase,0.,0.,False
            if abs(heading)>math.radians(4):
                return self.phase,0.,clamp(-heading*.7,-.18,.18),False
            self.phase='entrada'
        if self.phase=='entrada':
            if abs(lateral)>.20 or abs(heading)>math.radians(25):
                raise ValueError('Alinhamento fora do limite durante a entrada')
            done=abs(f)<=.015 and abs(r)<=.025 and abs(heading)<=math.radians(2)
            if done: return self.phase,0.,0.,True
            if f<=.015 and abs(r)>.025:
                raise ValueError('Erro lateral nao pode ser corrigido com seguranca no alvo')
            if abs(heading)>math.radians(5):
                return self.phase,0.,clamp(-heading*.65,-.18,.18),False
            speed=min(self.speed,max(0.,f*.22))
            rate=clamp(math.atan2(r,max(.4,f))*.8-heading*.65,-.18,.18)
            return self.phase,speed,rate,False
        raise ValueError('Fase de aproximacao invalida')
