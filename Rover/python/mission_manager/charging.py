"""Update SITL's own primary battery estimator; never forge telemetry packets."""
import math
from pymavlink import mavutil


class SITLCharger:
    def __init__(self, owner):
        self.owner=owner
        self.validated=False
        self.last_update=None

    def guard(self):
        o=self.owner
        hb=o.link.latest.get('HEARTBEAT')
        if (not o.args.simulate_charge or o.machine.inside is not True or not o.fresh()
                or not hb or hb.custom_mode!=4 or hb.base_mode&128):
            raise RuntimeError('Recarga SITL exige geometria estacionada, HOLD, desarmamento e telemetria fresca')
        speed,rate=o.motion_state()
        if abs(speed)>.05 or abs(rate)>.05:
            raise RuntimeError('Recarga SITL exige rover completamente parado')

    def prepare(self):
        self.guard()
        link=self.owner.link
        # SIM_* parameters plus fresh component-1 SIMSTATE identify the tested
        # setup. Do not reset an unsupported external/smart battery backend.
        voltage=link.param('SIM_BATT_VOLTAGE')
        capacity=link.param('SIM_BATT_CAP_AH')
        monitor=link.param('BATT_MONITOR')
        mah=link.param('BATT_CAPACITY')
        if not math.isfinite(voltage) or voltage<=0 or capacity!=0 or monitor!=4 or mah<=10:
            raise RuntimeError('Recarga testada exige SITL com SIM_BATT_CAP_AH=0, BATT_MONITOR=4 e BATT_CAPACITY>10')
        self.validated=True;self.last_update=None

    def sync(self, percentage, now, final=False):
        if not self.validated:raise RuntimeError('Recarga SITL nao preparada')
        if not math.isfinite(percentage) or not 0<=percentage<=100:
            raise ValueError('Percentual de recarga invalido')
        self.guard()
        if not final and self.last_update is not None and now-self.last_update<1.:
            return
        link=self.owner.link
        link.command(mavutil.mavlink.MAV_CMD_BATTERY_RESET,1,float(percentage))
        # Rounding to an integer % happens in the autopilot, not in this node.
        msg=link.wait('BATTERY_STATUS',lambda m:m.id==0 and 0<=m.battery_remaining<=100
                      and (m.battery_remaining==100 if final else abs(m.battery_remaining-percentage)<=1.1),timeout=5)
        if final:
            link.command(512,1)  # request SYS_STATUS, also used by ground stations
            link.wait('SYS_STATUS',lambda m:m.battery_remaining==100,timeout=5)
        self.last_update=now
        self.owner.event(kind='sitl_charge_update',target_soc=percentage,
                         confirmed_soc=msg.battery_remaining,consumed_mah=msg.current_consumed,final=final)
