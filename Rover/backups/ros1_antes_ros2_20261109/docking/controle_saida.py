"""Geometria da saida visual de re e do giro, independente de ROS."""
import math


def clamp(v,lo,hi): return min(max(v,lo),hi)


def alinhamento(pose):
    f,r,h=(float(pose[k]) for k in ('forward','right','normal_yaw'))
    if not all(math.isfinite(v) for v in (f,r,h)):
        raise ValueError('Pose do QR invalida')
    # Eixo normal a parede do QR; aumento indica movimento para tras.
    return f*math.cos(h)-r*math.sin(h), f*math.sin(h)+r*math.cos(h), h


def validar_inicio(pose):
    along,lateral,heading=alinhamento(pose)
    if not (-.30<=along<=.30 and abs(lateral)<=.30 and abs(heading)<=math.radians(12)):
        raise ValueError('Pose nao confirma rover dentro e alinhado no centro da estacao')
    return along


def recuo(pose, inicio, velocidade=.08):
    along,lateral,heading=alinhamento(pose)
    distance=along-inicio
    if abs(lateral)>.18 or abs(heading)>math.radians(18) or distance<-.15:
        raise ValueError('Recuo desalinhado ou movimento na direcao incorreta')
    # Freia antes do alvo; confirmacao posterior usa a pose com o rover parado.
    if distance>=1.96:
        return 0.,0.,distance,True
    remaining=2.-distance
    speed=-min(velocidade,max(.015,remaining*.15))
    # QR e observado a frente durante o recuo; sinais no referencial BODY_NED.
    yaw=clamp(-lateral*.35-heading*.6,-.12,.12)
    return speed,yaw,distance,False


class Giro180:
    def __init__(self, yaw):
        self.previous=yaw
        self.progresso=0.

    def step(self, yaw):
        if not math.isfinite(yaw): raise ValueError('Rumo invalido')
        delta=(yaw-self.previous+math.pi)%(2*math.pi)-math.pi
        if abs(delta)>math.radians(45): raise ValueError('Salto invalido no rumo')
        self.progresso+=delta; self.previous=yaw
        error=math.pi-self.progresso
        if error < -math.radians(10): raise ValueError('Giro ultrapassou a tolerancia')
        if abs(error)<=math.radians(3): return 0.,True
        return clamp(error*.5,-.15,.15),False
