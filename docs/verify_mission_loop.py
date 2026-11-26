"""Run the six isolated checks documented in the Stage 2 portfolio."""

import math
import time
import unittest
from pathlib import Path
from textwrap import indent
from types import SimpleNamespace


def load_mission_loop():
    """Run the published excerpt as a function with only its external names supplied."""
    portfolio = Path(__file__).with_name("stage-2-mission-manager.md")
    _, separator, fenced_code = portfolio.read_text(encoding="utf-8").partition(
        "\n```python\n"
    )
    excerpt, closing_fence, _ = fenced_code.partition("\n```")
    if not separator or not closing_fence:
        raise ValueError("Mission loop excerpt is missing from the Stage 2 portfolio")
    namespace = {
        "math": math,
        "time": time,
        "GoalStatus": SimpleNamespace(STATUS_SUCCEEDED=4),
        "NavigateToPose": SimpleNamespace(Goal=SimpleNamespace),
    }
    wrapper = "def run(self, markers, planner=None, grid=None, navigator=None):\n"
    exec(compile(wrapper + indent(excerpt, "    "), str(portfolio), "exec"), namespace)
    return namespace["run"]


class NavigationStub:
    def __init__(self, order):
        self.options = SimpleNamespace(
            route_numbers=order, goal_timeout=120, base_frame="base_footprint"
        )
        self.events = []
        self.dispatched = []
        self.status = 4
        self.action_error = None
        self.position_offset = 0
        self.transform_available = True

    def choose_goal(self, marker, planner, grid):
        return SimpleNamespace(pose=SimpleNamespace(position=marker), header=SimpleNamespace())

    def get_logger(self):
        return SimpleNamespace(info=lambda message: None)

    def get_clock(self):
        return SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: None))

    def action(self, navigator, request, timeout, label):
        self.dispatched.append(request.pose.pose.position.x)
        if self.action_error is not None:
            raise self.action_error
        return SimpleNamespace(status=self.status, result=SimpleNamespace())

    def lookup(self, target, source):
        if self.transform_available:
            return SimpleNamespace(
                translation=SimpleNamespace(
                    x=self.dispatched[-1] + self.position_offset, y=0
                )
            )
        return None


class MissionLoopTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.run_mission = staticmethod(load_mission_loop())

    def setUp(self):
        self.order = [3, 1, 2]
        self.node = NavigationStub(self.order)

    def run_route(self, order):
        markers = [SimpleNamespace(x=number, y=0, kind="test") for number in order]
        self.run_mission(self.node, markers)

    def assert_stops_after_first_goal(self, message):
        with self.assertRaisesRegex(RuntimeError, message):
            self.run_route(self.order)
        self.assertEqual(self.node.dispatched, self.order[:1])

    def test_supplied_order(self):
        order = [1, 2, 3]
        self.node = NavigationStub(order)
        self.run_route(order)
        self.assertEqual(self.node.dispatched, order)

    def test_shuffled_order(self):
        self.run_route(self.order)
        self.assertEqual(self.node.dispatched, self.order)
        self.assertEqual([event["marker"] for event in self.node.events], self.order)

    def test_aborted_action(self):
        self.node.status = 6
        self.assert_stops_after_first_goal("action status 6")
        self.assertEqual(self.node.events[-1]["state"], "failed")

    def test_missing_final_transform(self):
        self.node.transform_available = False
        self.assert_stops_after_first_goal("final-position check failed")

    def test_final_position_mismatch(self):
        self.node.position_offset = 0.50
        self.assert_stops_after_first_goal("final-position check failed")
        self.assertEqual(self.node.events[-1]["state"], "position_check_failed")

    def test_timeout_exception(self):
        self.node.action_error = RuntimeError("stub timeout")
        self.assert_stops_after_first_goal("stub timeout")


if __name__ == "__main__":
    unittest.main(verbosity=2)
