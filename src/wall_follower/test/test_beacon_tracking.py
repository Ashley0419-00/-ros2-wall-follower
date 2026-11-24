"""Non-motion regressions: run python3 -m unittest discover -s src/wall_follower/test."""
import math
import unittest
from types import SimpleNamespace
import numpy as np
from wall_follower.beacon_tracking import Tracker, lidar_match


class TrackingTest(unittest.TestCase):
    def test_same_colour_two_entities_and_no_duplicate_hits(self):
        tracker = Tracker()
        observations = [('blue/pink', np.array([1., 0., 0.]), 'approx'),
                        ('blue/pink', np.array([2., 0., 0.]), 'approx')]
        for stamp in range(4):
            tracker.update(observations, stamp)
        self.assertEqual(len(tracker.tracks), 2)
        self.assertTrue(all(t.confirmed(4) for t in tracker.tracks))
        tracker.update(observations, 3)
        self.assertEqual([t.hits for t in tracker.tracks], [4, 4])

    def test_measured_range_not_shifted_by_approximate_observation(self):
        tracker = Tracker()
        for stamp in range(4):
            tracker.update([('green/pink', np.array([1., 0., 0.]), 'lidar')], stamp)
        tracker.update([('green/pink', np.array([1.15, 0., 0.]), 'approx')], 4)
        self.assertTrue(np.allclose(tracker.tracks[0].point, [1., 0., 0.]))

    def test_ambiguous_second_hit_not_a_new_entity(self):
        tracker = Tracker()
        for stamp in range(4):
            tracker.update([('green/pink', np.array([1., 0., 0.]), 'approx')], stamp)
        tracker.update([('green/pink', np.array([1., 0., 0.]), 'approx'),
                        ('green/pink', np.array([1.02, 0., 0.]), 'approx')], 4)
        self.assertEqual(len(tracker.tracks), 1)
        self.assertEqual(tracker.tracks[0].hits, 5)

    def test_known_circle_and_scan_wraparound(self):
        raw = []
        for i in range(360):
            a = math.radians(i); disc = .05**2-math.sin(a)**2
            raw.append(math.cos(a)-math.sqrt(disc) if disc >= 0 and math.cos(a)>0 else math.inf)
        scan = SimpleNamespace(ranges=raw, angle_min=0., angle_increment=math.pi/180,
                               range_min=.1, range_max=3.5)
        optical = np.array([[0.,-1.,0.,0.], [0.,0.,-1.,0.], [1.,0.,0.,0.], [0.,0.,0.,1.]])
        k = np.array([[300.,0.,80.], [0.,300.,60.], [0.,0.,1.]])
        result = lidar_match(scan, optical, k, np.zeros(5), (63,40,34,40), .1, 3.5)
        self.assertIsNotNone(result)
        self.assertLess(np.linalg.norm(result-np.array([1.,0.,0.])), .01)
        scan.ranges = [2./math.cos(math.radians(i)) if abs(math.cos(math.radians(i)))>.01
                       else math.inf for i in range(360)]
        self.assertIsNone(lidar_match(scan, optical, k, np.zeros(5), (63,40,34,40), .1, 3.5))


if __name__ == '__main__':
    unittest.main()
