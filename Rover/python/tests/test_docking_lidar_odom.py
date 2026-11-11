import pathlib,sys,time,types,unittest
from unittest.mock import patch
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parents[1]/'docking_lidar'))
from sensores import Sensores

def odom(x,cov):
 p=types.SimpleNamespace(x=x,y=0,z=0)
 q=types.SimpleNamespace(x=0,y=0,z=0,w=1)
 return types.SimpleNamespace(header=types.SimpleNamespace(stamp=types.SimpleNamespace(),frame_id='camera_init'),child_frame_id='body',
  pose=types.SimpleNamespace(pose=types.SimpleNamespace(position=p,orientation=q),covariance=cov))
class TestOdom(unittest.TestCase):
 def test_rejeita_posicao_explodida_e_covariancia(self):
  import threading
  s=Sensores.__new__(Sensores);s.lock=threading.Lock();s.odom_wall=time.monotonic()
  valid=[0.]*36;valid[0]=valid[7]=valid[14]=.01
  with patch('sensores.ros.age',return_value=0.):
   s.odom=odom(1000000,valid)
   with self.assertRaises(ValueError):s.lio()
   bad=valid.copy();bad[0]=800;s.odom=odom(0,bad)
   with self.assertRaises(ValueError):s.lio()
   s.odom=odom(2,valid);position,yaw=s.lio()
   self.assertEqual(position[0],2);self.assertEqual(yaw,0)
   s.odom.header.frame_id='map'
   with self.assertRaisesRegex(ValueError,'camera_init -> body'):s.lio()
if __name__=='__main__':unittest.main()
