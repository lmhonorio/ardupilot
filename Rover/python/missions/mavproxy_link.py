"""MAVLink transactions through a MAVProxy --out UDP endpoint, never serial/TCP."""
import json
import math
import time
from pathlib import Path

from pymavlink import mavutil


def json_safe(value):
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {k: json_safe(v) for k, v in value.items()}
    if isinstance(value, (tuple, list)):
        return [json_safe(v) for v in value]
    return value


class Link:
    def __init__(self, endpoint, sysid, directory, timeout=12, source_system=246):
        if not endpoint.startswith('udpin:'):
            raise ValueError('Use udpin:IP:PORTA encaminhado pelo MAVProxy --out')
        self.source_system, self.source_component = source_system, 191
        self.conn = mavutil.mavlink_connection(endpoint, source_system=self.source_system,
                                             source_component=self.source_component,
                                             dialect='ardupilotmega')
        self.sysid, self.compid = sysid, 1
        self.conn.target_system = sysid
        self.conn.target_component = self.compid
        self.timeout = timeout
        self.latest, self.last_seen = {}, {}
        self.heartbeat_sent = 0
        self.log = (Path(directory) / 'telemetry.jsonl').open('w')
        self.trial = None
        self.seq = 0
        self.phase = 'idle'
        self.clock = 0
        self.records = []
        self.origin = None
        self.interval_backup = {}
        self.param_types = {}

    def tick(self):
        if time.monotonic() - self.heartbeat_sent > 1:
            self.conn.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            self.heartbeat_sent = time.monotonic()

    def recv(self, timeout=0.2):
        self.tick()
        msg = self.conn.recv_match(blocking=True, timeout=timeout)
        if msg is None or msg.get_type() == 'BAD_DATA' or msg.get_srcSystem() != self.sysid:
            return None
        kind = msg.get_type()
        self.latest[kind] = msg
        self.last_seen[kind] = time.monotonic()
        if kind in ('ATTITUDE','GLOBAL_POSITION_INT'):
            self.clock = msg.time_boot_ms / 1000
        if kind == 'MISSION_CURRENT':
            self.seq = msg.seq
        row = dict(msg.to_dict(), wall_time=time.time(), monotonic=time.monotonic(),
                   boot_s=self.clock, trial=self.trial, seq_current=self.seq,
                   phase=self.phase, source_system=msg.get_srcSystem())
        self.log.write(json.dumps(json_safe(row), allow_nan=False, default=str) + '\n')
        if self.trial is not None:
            self.records.append(row)
        if kind == 'STATUSTEXT':
            text = msg.text.decode(errors='replace') if isinstance(msg.text, bytes) else msg.text
            print('AP:', text, flush=True)
        return msg

    def wait(self, kind, predicate=lambda m: True, timeout=None):
        deadline = time.monotonic() + (self.timeout if timeout is None else timeout)
        while time.monotonic() < deadline:
            msg = self.recv(min(.2, max(0, deadline - time.monotonic())))
            if msg and msg.get_type() in ([kind] if isinstance(kind, str) else kind) and predicate(msg):
                return msg
        raise TimeoutError('Sem resposta %s via MAVProxy; confira --out, mavfwd e link' % str(kind))

    def drain(self, duration=.2):
        until = time.monotonic() + duration
        while time.monotonic() < until:
            self.recv(.01)

    def addressed(self, msg):
        return (getattr(msg, 'target_system', 0) in (0, self.source_system) and
                getattr(msg, 'target_component', 0) in (0, self.source_component))

    def command(self, command, *params, required=True):
        self.drain(.05)
        for retry in range(3):
            self.conn.mav.command_long_send(self.sysid, self.compid, command, retry,
                                            *(list(params) + [0] * (7-len(params))))
            try:
                ack = self.wait('COMMAND_ACK', lambda m: m.command == command and self.addressed(m))
            except TimeoutError:
                continue
            if ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                return True
            if ack.result == mavutil.mavlink.MAV_RESULT_IN_PROGRESS:
                ack = self.wait('COMMAND_ACK', lambda m: m.command == command and
                                self.addressed(m) and m.result != mavutil.mavlink.MAV_RESULT_IN_PROGRESS,
                                timeout=30)
                if ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                    return True
            if ack.result == mavutil.mavlink.MAV_RESULT_TEMPORARILY_REJECTED:
                time.sleep(.2)
                continue
            if required:
                raise RuntimeError('Comando %d recusado: resultado %d' % (command, ack.result))
            return False
        if required:
            raise TimeoutError('Comando %d sem ACK via MAVProxy' % command)
        return False

    def param(self, name, value=None):
        if value is not None and self.param_types.get(name) in (
                mavutil.mavlink.MAV_PARAM_TYPE_UINT8, mavutil.mavlink.MAV_PARAM_TYPE_INT8,
                mavutil.mavlink.MAV_PARAM_TYPE_UINT16, mavutil.mavlink.MAV_PARAM_TYPE_INT16,
                mavutil.mavlink.MAV_PARAM_TYPE_UINT32, mavutil.mavlink.MAV_PARAM_TYPE_INT32):
            value = round(value)
        for _ in range(3):
            if value is None:
                self.conn.mav.param_request_read_send(self.sysid, self.compid, name.encode(), -1)
            else:
                self.conn.mav.param_set_send(self.sysid, self.compid, name.encode(), float(value),
                                             mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
            try:
                msg = self.wait('PARAM_VALUE', lambda m: (m.param_id.decode() if isinstance(m.param_id, bytes)
                                else m.param_id).rstrip('\x00') == name, timeout=5)
                self.param_types[name] = msg.param_type
                if value is None or math.isclose(msg.param_value, value, rel_tol=1e-4, abs_tol=1e-5):
                    return msg.param_value
            except TimeoutError:
                pass
        raise RuntimeError('Parametro %s nao confirmado' % name)

    def set_params(self, values):
        for name, value in values.items():
            values[name] = self.param(name, value)
            print('PARAM %s = %.6g' % (name, values[name]), flush=True)

    def mode(self, name):
        mode_id = self.conn.mode_mapping()[name]
        self.command(mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                     mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED, mode_id)
        self.wait('HEARTBEAT', lambda m: m.custom_mode == mode_id)

    def disarm(self):
        self.command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0)
        self.wait('HEARTBEAT', lambda m: not m.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)

    def stop(self):
        self.mode('HOLD')
        self.disarm()

    def wait_stationary(self, timeout=90):
        deadline=time.monotonic()+timeout
        stable=None
        bit=mavutil.mavlink.MAV_SYS_STATUS_PREARM_CHECK
        while time.monotonic()<deadline:
            self.recv()
            pos=self.latest.get('GLOBAL_POSITION_INT')
            status=self.latest.get('SYS_STATUS')
            attitude=self.latest.get('ATTITUDE')
            fresh=time.monotonic()-self.last_seen.get('GLOBAL_POSITION_INT',0)<3
            speed=math.hypot(pos.vx,pos.vy)/100 if pos else math.inf
            rotation=(max(abs(attitude.rollspeed),abs(attitude.pitchspeed),abs(attitude.yawspeed))
                      if attitude else math.inf)
            health=(status is not None and (not status.onboard_control_sensors_present & bit or
                                             status.onboard_control_sensors_health & bit))
            if fresh and speed<.15 and rotation<.1 and health:
                if stable is None or self.clock<stable:stable=self.clock
                if self.clock-stable>=3:return
            else:
                stable=None
        raise RuntimeError('Rover nao estabilizou ou pre-armamento nao saudavel; verifique sensores e STATUSTEXT')

    def ready(self, real=False):
        hb = self.wait('HEARTBEAT', timeout=30)
        if hb.type != mavutil.mavlink.MAV_TYPE_GROUND_ROVER:
            raise RuntimeError('O alvo nao e um Rover')
        if hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
            raise RuntimeError('Comece com o rover desarmado e parado')
        # Request messages using the autopilot's normal command service.
        for msg_id in [33, 30, 36, 42, 46, 194, 24, 164, 193]:
            self.conn.mav.command_long_send(self.sysid, self.compid,
                mavutil.mavlink.MAV_CMD_GET_MESSAGE_INTERVAL, 0, msg_id, 0, 0, 0, 0, 0, 0)
            try:
                old = self.wait('MESSAGE_INTERVAL', lambda m: m.message_id == msg_id, timeout=2)
                self.interval_backup[msg_id] = old.interval_us
            except TimeoutError:
                self.interval_backup[msg_id] = 0  # default schedule if not supported
            self.conn.mav.command_long_send(self.sysid, self.compid,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0, msg_id,
                100000 if msg_id in (30, 33, 36, 194) else 1000000, 0, 0, 0, 0, 0)
        deadline = time.monotonic() + 180
        while time.monotonic() < deadline:
            self.recv()
            gps, pos, sim, ekf = [self.latest.get(k) for k in
                                 ['GPS_RAW_INT', 'GLOBAL_POSITION_INT', 'SIMSTATE', 'EKF_STATUS_REPORT']]
            # A GPS fix alone does not mean navigation has an EKF origin yet.
            if (gps and pos and ekf and gps.fix_type >= 3 and (pos.lat or pos.lon)
                    and ekf.flags & 16 and (real or sim)):
                self.origin = (pos.lat / 1e7, pos.lon / 1e7)
                print('Origem GPS:', self.origin, flush=True)
                return
        raise RuntimeError('GPS/EKF/SIMSTATE nao prontos. Rover fisico exige --real.')

    def download_mission(self):
        self.conn.mav.mission_request_list_send(self.sysid, self.compid)
        count = self.wait('MISSION_COUNT', self.addressed).count
        items = []
        for seq in range(count):
            for retry in range(3):
                self.conn.mav.mission_request_int_send(self.sysid, self.compid, seq)
                try:
                    msg = self.wait(['MISSION_ITEM_INT', 'MISSION_ITEM'],
                                    lambda m: m.seq == seq and self.addressed(m), timeout=4)
                    item = msg.to_dict()
                    if msg.get_type() == 'MISSION_ITEM_INT':
                        item['wire_x'], item['wire_y'] = msg.x, msg.y
                        item['x'], item['y'] = msg.x / 1e7, msg.y / 1e7
                    item['coordinate_units']='degrees'
                    items.append(item)
                    break
                except TimeoutError:
                    if retry == 2:
                        raise
        self.conn.mav.mission_ack_send(self.sysid, self.compid, mavutil.mavlink.MAV_MISSION_ACCEPTED)
        return items

    def upload_mission(self, items):
        self.drain(.2)
        self.conn.mav.mission_count_send(self.sysid, self.compid, len(items))
        last = None
        sent = set()
        for _ in range(len(items) * 4 + 10):
            try:
                msg = self.wait(['MISSION_REQUEST_INT', 'MISSION_REQUEST', 'MISSION_ACK'], self.addressed, timeout=5)
            except TimeoutError:
                if last is None:
                    self.conn.mav.mission_count_send(self.sysid, self.compid, len(items))
                else:
                    self.send_item(items[last], True)
                continue
            if msg.get_type() == 'MISSION_ACK':
                if msg.type != mavutil.mavlink.MAV_MISSION_ACCEPTED:
                    raise RuntimeError('Missao rejeitada: ACK %s' % msg.type)
                if len(sent) != len(items):
                    raise RuntimeError('ACK de missao antes de todos os itens')
                return
            if not 0 <= msg.seq < len(items):
                raise RuntimeError('MISSION_REQUEST com indice invalido')
            last = msg.seq
            # MAVLink requires INT items even in response to deprecated
            # MISSION_REQUEST. Float longitude here loses ~0.4 m resolution.
            self.send_item(items[last], True)
            sent.add(last)
        raise TimeoutError('Upload da missao nao confirmou todos os itens')

    def send_item(self, item, integer=True):
        frame={0:5,3:6,10:11}.get(item['frame'],item['frame'])
        self.conn.mav.mission_item_int_send(self.sysid, self.compid, item['seq'], frame, item['command'],
               int(item['seq'] == 0), 1, item.get('param1', 0), 0, 0, 0,
               int(round(item['lat'] * 1e7)),int(round(item['lon'] * 1e7)),item['altitude'])

    def verify_mission(self, items):
        received=self.download_mission()
        if len(received)!=len(items):
            raise RuntimeError('Numero de itens devolvidos difere da missao enviada')
        for expected,actual in zip(items[1:],received[1:]):
            if (expected['command']!=actual['command'] or
                    abs(expected['lat']-actual['x'])>1.5e-7 or
                    abs(expected['lon']-actual['y'])>1.5e-7):
                raise RuntimeError('Item %d difere no readback; sem armamento'%expected['seq'])
        return received

    def close(self):
        for msg_id, interval in self.interval_backup.items():
            self.conn.mav.command_long_send(self.sysid, self.compid,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0, msg_id, interval, 0, 0, 0, 0, 0)
        self.drain(.3)
        self.log.close()
        self.conn.close()
