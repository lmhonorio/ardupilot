"""Pure operational state machine and explicitly simulated battery."""
from enum import Enum
import math

class State(str, Enum):
    WAIT = 'ESPERA'
    MISSION = 'MISSAO'
    REQUESTED = 'DOCKING_SOLICITADO'
    ALIGN = 'ALINHAMENTO'
    APPROACH = 'APROXIMACAO'
    PARKED = 'ESTACIONADO'
    CHARGING = 'RECARGA'
    CHARGED = 'RECARGA_CONCLUIDA'
    EXIT = 'SAIDA'
    PREPARE = 'PREPARAR_MISSAO'
    FAULT = 'FALHA'

ALLOWED = {
    State.WAIT: {State.MISSION, State.REQUESTED, State.PARKED, State.PREPARE},
    State.MISSION: {State.WAIT, State.REQUESTED},
    State.REQUESTED: {State.ALIGN, State.PARKED},
    State.ALIGN: {State.APPROACH},
    State.APPROACH: {State.ALIGN, State.PARKED},
    State.PARKED: {State.CHARGING, State.PREPARE},
    State.CHARGING: {State.CHARGED},
    State.CHARGED: {State.PREPARE},
    State.PREPARE: {State.EXIT, State.MISSION, State.PARKED, State.CHARGED, State.WAIT},
    State.EXIT: {State.PREPARE},
    State.FAULT: {State.WAIT, State.PARKED},
}

class Machine:
    def __init__(self, emit=lambda **kw: None):
        self.state = State.WAIT
        self.pending = False
        self.inside = None  # unknown until reconciled with sensors
        self.emit = emit

    def move(self, state, reason):
        state = State(state)
        if state == self.state: return
        if state != State.FAULT and state not in ALLOWED[self.state]:
            raise RuntimeError('Transicao invalida: %s -> %s' % (self.state, state))
        previous = self.state
        self.state = state
        self.emit(kind='transition', previous=previous.value, state=state.value, reason=reason)

    def request_start(self, valid, sufficient):
        if self.pending:
            return False, 'Solicitacao ja pendente'
        if self.state not in (State.WAIT, State.PARKED, State.CHARGING, State.CHARGED):
            return False, 'Operacao em andamento ou falha; reconciliar primeiro'
        if self.inside is None or not valid:
            return False, 'Localizacao desconhecida ou missao invalida'
        if not sufficient and self.state != State.CHARGING:
            return False, 'Bateria insuficiente ou indisponivel'
        self.pending = True
        return True, 'Missao pendente; nao iniciada ainda'

    def confirmed_outside(self):
        self.inside = False

    def confirmed_parked(self):
        self.inside = True
        self.move(State.PARKED, 'Geometria final, parada e desarmamento confirmados')

class Battery:
    """Never writes BATTERY_STATUS, voltage/current or autopilot parameters."""
    def __init__(self, simulated=False, initial=50., minimum=30., discharge_per_s=.02, native=False):
        if not all(math.isfinite(v) for v in (initial, minimum, discharge_per_s)):
            raise ValueError('Bateria: valores nao finitos')
        if not 0<=initial<=100 or not 0<minimum<=100 or discharge_per_s<0:
            raise ValueError('Bateria: limites invalidos')
        self.simulated, self.soc, self.minimum = simulated, initial, minimum
        self.native = native
        self.rate = discharge_per_s
        self.real = None
        self.real_wall = 0.
        self.started = None
        self.initial = initial
        self.last = None

    def observe(self, value, now):
        self.real = float(value) if value is not None and math.isfinite(value) and 0<=value<=100 else None
        self.real_wall = now

    def begin_charge(self, now, sitl, parked):
        if not self.simulated or not sitl or not parked:
            raise ValueError('Recarga simulada exige habilitacao, SITL e docking confirmado')
        if self.native:
            value=self.operational(now)
            if value is None:raise ValueError('BATTERY_STATUS recente necessario para recarga SITL')
            self.soc=value
        self.started, self.initial = now, self.soc

    def tick(self, now, running=False):
        done = False
        if self.started is not None:
            fraction = min(1., max(0., (now-self.started)/15.))
            self.soc = self.initial + (100.-self.initial)*fraction
            if fraction == 1.:
                done = True
                self.started = None
        elif self.simulated and not self.native and running and self.last is not None:
            self.soc = max(0., self.soc-self.rate*max(0., now-self.last))
        self.last = now
        return done

    def operational(self, now):
        if self.native:return self.real if now-self.real_wall<5 else None
        return self.soc if self.simulated else self.real if now-self.real_wall<5 else None

    def sufficient(self, now):
        soc = self.operational(now)
        return soc is not None and soc>=self.minimum

    def status(self, now):
        return dict(source='sitl_battery_status' if self.native else 'simulated' if self.simulated else 'battery_status',
                    operational_soc=self.operational(now), real_soc=self.real,
                    real_fresh=now-self.real_wall<5, simulated_soc=self.operational(now) if self.native else self.soc if self.simulated else None,
                    charge_target_soc=self.soc if self.native and self.started is not None else None,
                    minimum=self.minimum, charging=self.started is not None)
