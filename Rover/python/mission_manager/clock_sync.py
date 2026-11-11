"""Map autopilot boot timestamps to ROS time with matched MAVLink TIMESYNC replies."""
from collections import deque
import math


class BootClock:
    def __init__(self):
        self.pending={};self.samples=deque(maxlen=6)
        self.offset=None;self.last_sent=-float('inf');self.last_valid=None
        self.last_ros=None;self.error=None;self.sequence=0

    def request(self, ros_now, wall_now, send):
        if ros_now<=0:return
        if self.last_ros is not None and ros_now<self.last_ros-.05:
            self.error='Relogio ROS reiniciado; reinicie o gerenciador'
        self.last_ros=ros_now
        if self.error:return
        if wall_now-self.last_sent<(5. if self.offset is not None else .25):return
        self.pending={k:v for k,v in self.pending.items() if wall_now-v[1]<1.}
        self.sequence+=1
        token=int(wall_now*1e9)+self.sequence
        self.pending[token]=(ros_now,wall_now);self.last_sent=wall_now
        send(0,token)

    def response(self, tc1, ts1, ros_now, wall_now):
        start=self.pending.pop(ts1,None)
        if start is None or tc1<=0 or self.error:return False
        ros_sent,wall_sent=start;rtt=wall_now-wall_sent;elapsed=ros_now-ros_sent
        if not 0<=rtt<=.3 or not 0<=elapsed<=.3:return False
        offset=(ros_sent+ros_now)/2.-tc1/1e9
        if not math.isfinite(offset):return False
        if self.offset is not None and abs(offset-self.offset)>.2:
            self.error='Salto de relogio MAVLink/ROS; reinicie o gerenciador'
            return False
        self.samples.append((rtt,offset,wall_now))
        if len(self.samples)<3:return False
        recent=[x for x in self.samples if wall_now-x[2]<30.]
        self.offset=min(recent,key=lambda x:x[0])[1];self.last_valid=wall_now
        return True

    def age(self, boot_seconds, ros_now, wall_now):
        if self.error:raise ValueError(self.error)
        if self.offset is None or self.last_valid is None or wall_now-self.last_valid>30.:
            raise ValueError('Sincronizacao TIMESYNC ausente/desatualizada')
        return ros_now-(boot_seconds+self.offset)

    def status(self):
        return dict(offset_ros_minus_boot_s=self.offset,last_valid_wall=self.last_valid,error=self.error)
