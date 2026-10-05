import unittest
from unittest.mock import Mock,patch
from plate_observation import observe_or_hold


class ObservationPauseTests(unittest.TestCase):
    def test_fresh_observation_never_pauses(self):
        freeze=Mock();tick=Mock();save=Mock();fresh={'received':12.}
        self.assertIs(observe_or_hold(lambda:fresh,freeze,tick,save),fresh)
        freeze.assert_not_called();tick.assert_not_called();save.assert_not_called()

    def test_reacquisition_holds_before_using_new_timestamp(self):
        get=Mock(side_effect=[ValueError('stale'),ValueError('stale'),{'received':2.}])
        clock=[0.];events=[];saved=[]
        def tick():clock[0]+=.1;events.append('hold')
        with patch('plate_observation.time.monotonic',side_effect=lambda:clock[0]):
            data=observe_or_hold(get,lambda:events.append('freeze'),tick,saved.append)
        self.assertEqual(events,['freeze','hold','hold'])
        self.assertEqual(data['received'],2.)
        self.assertTrue(saved[0]['reacquired'])

    def test_persistent_loss_times_out_without_fresh_data(self):
        get=Mock(side_effect=ValueError('stale'));clock=[0.];saved=[]
        def tick():clock[0]+=.1
        with patch('plate_observation.time.monotonic',side_effect=lambda:clock[0]):
            with self.assertRaisesRegex(RuntimeError,'did not return'):
                observe_or_hold(get,lambda:None,tick,saved.append,.3)
        self.assertFalse(saved[0]['reacquired'])
        self.assertLessEqual(clock[0],.4)

    def test_motor_fault_propagates_without_reacquisition_retry(self):
        get=Mock(side_effect=ValueError('stale'));saved=[]
        with self.assertRaisesRegex(RuntimeError,'motor fault'):
            observe_or_hold(get,lambda:None,Mock(side_effect=RuntimeError('motor fault')),saved.append)
        self.assertEqual(get.call_count,1)
        self.assertFalse(saved[0]['reacquired'])
