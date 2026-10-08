import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import patch
import numpy as np

from printer_demo_replay import ReplayRunner, ReversedCurve
from blended_motion import BlendedPath


class ReplayInterlockTests(unittest.TestCase):
    def test_full_return_is_exact_curve_in_reverse_with_same_duration(self):
        p=BlendedPath([[0.]*6,[1.5,2.,-1.,.3,1.,.3],[3.,4.,-2.,1.,2.,1.]],3.,9.)
        reverse=ReversedCurve(p,p.duration)
        self.assertEqual(reverse.duration,p.duration)
        for t in np.linspace(0,p.duration,501):
            np.testing.assert_allclose(reverse.at(t),p.at(p.duration-t),atol=1e-12)

    def test_partial_return_stays_on_executed_curve_and_starts_at_rest(self):
        p=BlendedPath([[0.]*6,[1.5,2.,-1.,.3,1.,.3],[3.,4.,-2.,1.,2.,1.]],3.,9.)
        end=p.duration*.6;reverse=ReversedCurve(p,end)
        np.testing.assert_allclose(reverse.at(0),p.at(end))
        np.testing.assert_allclose(reverse.at(reverse.duration),p.at(0))
        t=np.linspace(0,reverse.duration,2001);q=np.array([reverse.at(x) for x in t])
        velocity=np.gradient(q,t,axis=0);acceleration=np.gradient(velocity,t,axis=0)
        self.assertLess(np.max(abs(velocity[[0,-1]])),.001)
        self.assertLess(np.max(abs(velocity)),3.)
        self.assertLess(np.max(abs(acceleration)),9.)
        for x in t[::100]:
            u=x/reverse.duration;s=u**3*(10-15*u+6*u*u)
            np.testing.assert_allclose(reverse.at(x),p.at(end*(1-s)))

    def test_half_degree_roll_deadband_does_not_deadlock_replay(self):
        r=self.runner();r.speed=3.;r.acceleration=9.;r.tracking_limit=1.5;r.pacing_lag=1.
        r.low=np.full(6,-10.);r.high=np.full(6,10.);r.targets=dict(enumerate([0.]*6))
        r.settle_tolerance=np.array([.2]*5+[.75]);r.phase='test';r.dynamic_reviews=300
        actual=np.zeros(6);r.rows=[{'raw_deg':actual.tolist()}];r.vision_ready=lambda:True
        clock=[100.]
        def now():clock[0]+=.05;return clock[0]
        def tick(**kwargs):
            target=np.array(list(r.targets.values()));error=target-actual
            actual[:5]=target[:5]
            if abs(error[5])>.55:actual[5]=target[5]-.5*np.sign(error[5])
            r.rows.append({'raw_deg':actual.tolist()});return actual.tolist()
        r.tick=tick
        with patch('printer_demo_replay.time.monotonic',side_effect=now), patch('printer_demo_replay.review') as review:
            visited=[[0.]*6];r.blend([[0.]*6,[0.]*5+[5.]],visited)
            review.assert_not_called()
        self.assertLess(abs(actual[5]-5.),.75)
        self.assertEqual(visited[-1],[0.]*5+[5.])

    def test_fixed_home_reuses_checked_entry_without_accumulating_measured_start_offset(self):
        home=BlendedPath([[0.]*6,[2.]*6],3.,9.)
        fresh=BlendedPath([[0.]*5+[-.6],[2.]*6],3.,9.)
        tail=BlendedPath([[2.]*6,[4.]*6],3.,9.)
        r=self.runner();r.executed_curves=[{'path':fresh,'elapsed':fresh.duration},{'path':tail,'elapsed':tail.duration}]
        calls=[]
        r.follow_curve=lambda path,visited,returning: calls.append(path)
        r.return_executed_curves(home)
        np.testing.assert_allclose(calls[0].at(calls[0].duration),calls[1].at(0))
        np.testing.assert_allclose(calls[-1].at(calls[-1].duration),home.at(0))
        # The recorded outgoing curve is untouched; interrupted returns retain it.
        self.assertIs(r.executed_curves[0]['path'],fresh)
        calls.clear();r.return_executed_curves()
        np.testing.assert_allclose(calls[-1].at(calls[-1].duration),fresh.at(0))
        r.executed_curves=[{'path':fresh,'elapsed':fresh.duration/2}]
        with self.assertRaisesRegex(RuntimeError,'does not join'):r.return_executed_curves(home)

    def runner(self):
        runner=object.__new__(ReplayRunner)
        runner.w=SimpleNamespace(lock=threading.RLock(),
            images={'wrist':b'jpeg','tripod':b'jpeg'},
            sources={r:{'time':time.monotonic(),'frame_age_s':.01} for r in ('wrist','tripod')})
        runner.rail=SimpleNamespace(connected=True,state='Idle',error=None,
            status_time=time.monotonic(),x_mm=0.)
        runner.pause_requested=False;runner.returning=False;runner.return_pauses=0
        runner.executed_curves=[]
        runner.output='unused'
        return runner

    def test_stale_camera_or_rail_blocks_motion(self):
        r=self.runner();self.assertTrue(r.vision_ready())
        for role in ('wrist','tripod'):
            before=r.w.sources[role]['time'];r.w.sources[role]['time']-=1
            self.assertFalse(r.vision_ready());r.w.sources[role]['time']=before
        r.rail.status_time-=1;self.assertFalse(r.vision_ready())

    def test_rail_motion_alarm_position_change_and_pause_block_motion(self):
        for key,value in [('state','Jog'),('state','Alarm'),('error','reset'),
                          ('connected',False),('x_mm',.2)]:
            r=self.runner();setattr(r.rail,key,value);self.assertFalse(r.vision_ready())
        r=self.runner();r.pause_requested=True;self.assertFalse(r.vision_ready())

    def test_outbound_pause_does_not_send_a_motor_command(self):
        r=self.runner();r.pause_requested=True
        with patch('printer_demo_replay.LiftRunner.tick') as tick:
            with self.assertRaisesRegex(ValueError,'Replay paused'):r.tick(moving=True)
            tick.assert_not_called()

    def test_return_also_stops_for_review_when_camera_is_lost(self):
        r=self.runner();r.returning=True;r.w.images['wrist']=b''
        with patch('printer_demo_replay.review',side_effect=RuntimeError('review pending')) as review:
            with patch('printer_demo_replay.LiftRunner.tick') as tick:
                with self.assertRaisesRegex(RuntimeError,'review pending'):r.tick(moving=True)
                tick.assert_not_called();review.assert_called_once()


if __name__=='__main__':unittest.main()
