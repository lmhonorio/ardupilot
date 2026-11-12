"""A lost UDP mission count must retry, never invent an empty mission."""
from pathlib import Path
import sys
from types import SimpleNamespace as NS
from unittest.mock import Mock
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from missions.mavproxy_link import Link

class MissionDownload(unittest.TestCase):
    def link(self):
        link=Link.__new__(Link);link.sysid=1;link.compid=1;link.timeout=12
        link.conn=NS(mav=Mock());link.source_system=244;link.source_component=191
        return link

    def test_count_reply_loss_retransmits_and_reads_actual_item(self):
        link=self.link();item=Mock(seq=0,x=1234567,y=2345678)
        item.get_type.return_value='MISSION_ITEM_INT';item.to_dict.return_value={'seq':0,'command':16}
        link.wait=Mock(side_effect=[TimeoutError(),NS(count=1),item])
        result=link.download_mission()
        self.assertEqual(link.conn.mav.mission_request_list_send.call_count,2)
        self.assertEqual(result[0]['x'],.1234567)
        link.conn.mav.mission_ack_send.assert_called_once()

    def test_total_loss_fails_without_ack_or_fabricated_empty_mission(self):
        link=self.link();link.wait=Mock(side_effect=TimeoutError())
        with self.assertRaises(TimeoutError):link.download_mission()
        self.assertEqual(link.conn.mav.mission_request_list_send.call_count,3)
        self.assertLessEqual(sum(x.kwargs['timeout'] for x in link.wait.call_args_list),link.timeout)
        link.conn.mav.mission_ack_send.assert_not_called()

    def test_legitimate_empty_mission_is_acked(self):
        link=self.link();link.wait=Mock(return_value=NS(count=0))
        self.assertEqual(link.download_mission(),[])
        self.assertEqual(link.conn.mav.mission_request_list_send.call_count,1)
        link.conn.mav.mission_ack_send.assert_called_once()

    def test_ekf_gate_rejects_boot_constant_and_predicted_only_position(self):
        link=self.link();link.command=Mock()
        def wait(kind,predicate,timeout):
            self.assertEqual(kind,'EKF_STATUS_REPORT')
            for flags in (167,1|2|512,831|1024,831|128):self.assertFalse(predicate(NS(flags=flags)))
            self.assertTrue(predicate(NS(flags=831)))
            return NS(flags=831)
        link.wait=wait
        self.assertEqual(link.wait_navigation_ready().flags,831)

if __name__=='__main__':unittest.main()
