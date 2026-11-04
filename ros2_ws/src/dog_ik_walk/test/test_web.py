import math
import threading
import unittest
from unittest.mock import patch
from dog_ik_walk.web_server import ControlAPI,MockBackend


class WebTests(unittest.TestCase):
    def setUp(self):
        self.backend=MockBackend();self.api=ControlAPI(self.backend);self.token=self.api.session()['token'];self.seq=0
    def send(self,active=True,gesture='a',**kw):
        self.seq+=1
        return self.api.request('/api/drive',dict(token=self.token,seq=self.seq,gesture=gesture,active=active,vx=.08,wz=0,**kw))
    def arm(self):self.backend.enabled=True;self.backend.ready=True
    def test_loading_page_does_not_enable(self):
        self.assertFalse(self.backend.enabled)
        with self.assertRaises(ValueError):self.send()
    def test_release_blocks_delayed_active_packets(self):
        self.arm();self.send();self.send(False)
        with self.assertRaises(ValueError):self.send()
        self.send(gesture='b');self.assertTrue(self.backend.last['active'])
    def test_reordered_packet_is_rejected(self):
        self.arm();self.send()
        with self.assertRaises(ValueError):self.api.request('/api/drive',dict(token=self.token,seq=1,gesture='a',active=True,vx=.08,wz=0))
    def test_expired_lease_requires_fresh_gesture(self):
        self.arm();self.send();self.api.owner_until=0
        with self.assertRaises(ValueError):self.send()
        self.send(gesture='new')
    def test_second_phone_cannot_take_over_but_can_disable(self):
        self.arm();self.send();other=self.api.session()['token']
        with self.assertRaises(ValueError):self.api.request('/api/drive',dict(token=other,seq=1,gesture='b',active=True,vx=.08,wz=0))
        self.api.request('/api/action',dict(token=other,seq=2,action='disable'))
        self.assertFalse(self.backend.enabled)
    def test_nan_and_outside_range_rejected(self):
        self.arm()
        for i,v in enumerate([float('nan'),float('inf'),.2]):
            with self.assertRaises(ValueError):self.api.request('/api/drive',dict(token=self.token,seq=i+1,gesture='a',active=True,vx=v,wz=0))
    def test_disable_not_blocked_by_slow_service(self):
        waiting=threading.Event();finish=threading.Event();disabled=threading.Event()
        def action(name):
            if name=='enable':waiting.set();finish.wait(1)
            else:disabled.set()
            return dict(ok=True)
        self.backend.action=action
        thread=threading.Thread(target=lambda:self.api.request('/api/action',dict(token=self.token,seq=1,action='enable')));thread.start();waiting.wait(1)
        self.api.request('/api/action',dict(token=self.token,seq=2,action='disable'))
        self.assertTrue(disabled.is_set());finish.set();thread.join()


if __name__=='__main__':unittest.main()
