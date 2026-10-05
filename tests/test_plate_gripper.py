import unittest
from types import SimpleNamespace
from unittest.mock import patch
from plate_gripper import restore_open_at_rest


class JawResetTests(unittest.TestCase):
    def exercise(self,start=-1.099,rest=True,stalled=False):
        clock=[0.]
        class Grip:
            def __init__(self):self.q=start;self.mode=1;self.timeout=12000;self.enabled=False;self.ever_enabled=False;self.commands=[]
            def get_register_f32(self,*args):return self.q
            def get_register_u32(self,r,*args):return self.mode if r==10 else self.timeout
            def ensure_mode(self,m,*args):self.mode=m
            def set_can_timeout_ms(self,ms):self.timeout=ms*20
            def write_register_u32(self,r,v):self.timeout=v
            def request_feedback(self):pass
            def get_state(self):return SimpleNamespace(status_code=int(self.enabled),vel=0.,torq=.05,t_mos=30.,t_rotor=30.)
            def send_force_pos(self,q,*args):
                self.commands.append(q)
                if self.enabled and not stalled:self.q=q
            def enable(self):self.enabled=True;self.ever_enabled=True
            def disable(self):self.enabled=False
        self.grip=grip=Grip()
        def read():clock[0]+=.05;return [0.,0. if rest else 10.,0.,0.,0.,0.]
        arm=SimpleNamespace(active=False,read=read,ctrl=SimpleNamespace(poll_feedback_once=lambda:None))
        geometry=SimpleNamespace(profile={'reference_raw_rad':[0.]*6})
        with patch('plate_gripper.time.monotonic',side_effect=lambda:clock[0]),patch('plate_gripper.time.sleep'):
            return restore_open_at_rest(arm,grip,geometry,lambda:True)

    def test_small_accumulated_drift_is_opened_at_rest(self):
        r=self.exercise()
        self.assertTrue(r['adjusted']);self.assertAlmostEqual(r['q_rad'],-1.17,delta=.004)
        self.assertTrue(all(a>=b for a,b in zip(self.grip.commands,self.grip.commands[1:])))
        self.assertFalse(self.grip.enabled);self.assertEqual((self.grip.mode,self.grip.timeout),(1,12000))

    def test_already_open_never_enables(self):
        self.assertFalse(self.exercise(start=-1.175)['adjusted'])
        self.assertFalse(self.grip.ever_enabled)

    def test_extended_arm_or_large_correction_never_enables(self):
        for kwargs in ({'rest':False},{'start':-.8}):
            with self.assertRaises(RuntimeError):self.exercise(**kwargs)
            self.assertFalse(self.grip.ever_enabled)

    def test_stalled_reset_disables_and_restores_settings(self):
        with self.assertRaisesRegex(RuntimeError,'did not settle'):self.exercise(stalled=True)
        self.assertFalse(self.grip.enabled);self.assertEqual((self.grip.mode,self.grip.timeout),(1,12000))
