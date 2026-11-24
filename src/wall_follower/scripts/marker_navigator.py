#!/usr/bin/env python3
"""Stage 2: navigate with nav2 to the markers listed in a marker file, in file order.

Marker file (written by point_transformer at the end of a wall-follower run):
    start_x, start_y, start_heading
    marker1_x, marker1_y, marker1_type
    ...
The robot is placed at the wall follower's starting pose, which is the AMCL initial
pose. The start line is in odom, which equals map only if odom was zero when
Cartographer started; so the map-frame start that point_transformer writes to
<marker_file>.map_start is preferred, and the start line is the fallback.
nav2 (map_server, amcl, planner, controller) must already be launched with the saved
map; see launch/navigation.launch.py.

A marker is a solid cylinder, so it is lethal in the costmap. Each goal is therefore a
free stand-off point around the marker, chosen from the global costmap, facing it.
"""
import math
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

LETHAL = 253          # inscribed/lethal/unknown costs are >= this
UNKNOWN = 255
SAFE = 128            # prefer stand-off cells below this inflation cost


def read_marker_file(filename):
    """Return ((x, y, heading), [(x, y, type), ...]) from the Stage 2 file format."""
    lines = [line.strip() for line in Path(filename).expanduser().read_text().splitlines()]
    rows = [[field.strip() for field in line.split(',')] for line in lines if line]
    if not rows or len(rows[0]) != 3:
        raise ValueError('first line must be: start_x, start_y, start_heading')
    start = tuple(float(v) for v in rows[0])
    markers = []
    for number, row in enumerate(rows[1:], start=2):
        if len(row) != 3 or not row[2]:
            raise ValueError('line %d must be: marker_x, marker_y, marker_type' % number)
        markers.append((float(row[0]), float(row[1]), row[2]))
    if not all(math.isfinite(v) for v in start) or \
            not all(math.isfinite(m[0]) and math.isfinite(m[1]) for m in markers):
        raise ValueError('non-finite coordinate in marker file')
    return start, markers


def read_map_start(filename):
    """Return (x, y, heading) from a <marker_file>.map_start file, or None if absent."""
    path = Path(str(Path(filename).expanduser()) + '.map_start')
    if not path.exists():
        return None
    values = tuple(float(v) for v in path.read_text().strip().split(','))
    if len(values) != 3 or not all(math.isfinite(v) for v in values):
        raise ValueError('%s must be: x, y, heading' % path)
    return values


def make_pose(navigator, x, y, heading):
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x, pose.pose.position.y = float(x), float(y)
    pose.pose.orientation.z = math.sin(heading / 2.0)
    pose.pose.orientation.w = math.cos(heading / 2.0)
    return pose


def costmap_cost(costmap, x, y):
    """Cost at a map-frame point, or UNKNOWN outside the costmap."""
    info = costmap.metadata
    mx = int(math.floor((x - info.origin.position.x) / info.resolution))
    my = int(math.floor((y - info.origin.position.y) / info.resolution))
    if not (0 <= mx < info.size_x and 0 <= my < info.size_y):
        return UNKNOWN
    return costmap.data[my * info.size_x + mx]


def standoff_goals(costmap, marker_x, marker_y, from_x, from_y, radii, step_deg=10):
    """Free points around a marker, best first: comfortably free, then nearest the
    marker, then nearest the robot (so the robot stops on the side it arrives from)."""
    candidates = []
    for radius in radii:
        for k in range(0, 360, step_deg):
            a = math.radians(k)
            x, y = marker_x + radius * math.cos(a), marker_y + radius * math.sin(a)
            cost = costmap_cost(costmap, x, y)
            if cost >= LETHAL:
                continue
            facing = math.atan2(marker_y - y, marker_x - x)
            candidates.append((cost >= SAFE, round(radius, 3),
                               math.hypot(x - from_x, y - from_y), x, y, facing))
    candidates.sort()
    return [(x, y, facing) for _, _, _, x, y, facing in candidates]


def spread(candidates, minimum=0.25, count=3):
    """Pick up to `count` alternatives that are not almost the same spot."""
    chosen = []
    for c in candidates:
        if all(math.hypot(c[0] - o[0], c[1] - o[1]) >= minimum for o in chosen):
            chosen.append(c)
        if len(chosen) == count:
            break
    return chosen


def drive(navigator, goal, timeout_sec):
    navigator.goToPose(goal)
    started = navigator.get_clock().now()
    cancelled = False
    while not navigator.isTaskComplete():  # spins the node for ~0.1 s per call
        feedback = navigator.getFeedback()
        if feedback:
            navigator.get_logger().info('distance_remaining=%.2f' % feedback.distance_remaining,
                                        throttle_duration_sec=2.0)
        elapsed = (navigator.get_clock().now() - started).nanoseconds / 1e9
        if not cancelled and elapsed > timeout_sec:
            navigator.get_logger().warn('Goal timeout; cancelling')
            navigator.cancelTask()
            cancelled = True
    return navigator.getResult()


def main(args=None):
    rclpy.init(args=args)
    navigator = BasicNavigator()
    navigator.declare_parameter('marker_file', '~/wall_follower_markers.csv')
    navigator.declare_parameter('standoff_min_m', 0.30)
    navigator.declare_parameter('standoff_max_m', 0.60)
    navigator.declare_parameter('goal_timeout_sec', 120.0)
    p = lambda name: navigator.get_parameter(name).value
    log = navigator.get_logger()
    try:
        start, markers = read_marker_file(p('marker_file'))
        map_start = read_map_start(p('marker_file'))
    except (OSError, ValueError) as exc:
        log.error('Cannot read marker file %s: %s' % (p('marker_file'), exc))
        navigator.destroy_node()
        rclpy.shutdown()
        return 1
    log.info('Start (odom) %.3f, %.3f, %.3f; %d markers in order: %s' % (
        start + (len(markers), ', '.join(m[2] for m in markers))))
    if map_start is None:
        log.warn('No %s.map_start: using the odom start line as the map pose. This is '
                 'only correct if odom was zero when Cartographer started.' % p('marker_file'))
        map_start = start
    else:
        log.info('Start (map) %.3f, %.3f, %.3f' % map_start)

    # The robot is placed at the wall follower's start: use it as the AMCL initial pose.
    navigator.setInitialPose(make_pose(navigator, *map_start))
    navigator.waitUntilNav2Active()

    # Fail clearly here, rather than as a planning failure at the first marker.
    start_cost = costmap_cost(navigator.getGlobalCostmap(), map_start[0], map_start[1])
    if start_cost == UNKNOWN or start_cost >= 254:
        log.error('Initial pose (%.2f, %.2f) is %s in the saved map: wrong start frame, '
                  'wrong map, or the robot is not at the wall-follower start' % (
                      map_start[0], map_start[1],
                      'outside/unknown' if start_cost == UNKNOWN else 'inside an obstacle'))
        navigator.destroy_node()
        rclpy.shutdown()
        return 1

    low, high = float(p('standoff_min_m')), float(p('standoff_max_m'))
    radii = [low + i * 0.05 for i in range(int(round((high - low) / 0.05)) + 1)]
    here_x, here_y = map_start[0], map_start[1]
    results = []
    for index, (marker_x, marker_y, kind) in enumerate(markers, start=1):
        log.info('Marker %d/%d: %s at (%.2f, %.2f)' % (index, len(markers), kind,
                                                       marker_x, marker_y))
        goals = spread(standoff_goals(navigator.getGlobalCostmap(), marker_x, marker_y,
                                      here_x, here_y, radii))
        if not goals:
            log.warn('No free stand-off point in the costmap; trying the marker position')
            goals = [(marker_x, marker_y, math.atan2(marker_y - here_y, marker_x - here_x))]
        result = TaskResult.FAILED
        for attempt, (x, y, facing) in enumerate(goals, start=1):
            log.info('Goal attempt %d: (%.2f, %.2f) facing %.2f' % (attempt, x, y, facing))
            result = drive(navigator, make_pose(navigator, x, y, facing),
                           float(p('goal_timeout_sec')))
            if result == TaskResult.SUCCEEDED:
                here_x, here_y = x, y
                break
        log.info('Marker %s: %s' % (kind, 'reached' if result == TaskResult.SUCCEEDED
                                    else 'FAILED'))
        results.append((kind, result == TaskResult.SUCCEEDED))

    log.info('Done: ' + ', '.join('%s=%s' % (k, 'ok' if ok else 'failed') for k, ok in results))
    navigator.destroy_node()
    rclpy.shutdown()
    return 0 if all(ok for _, ok in results) else 2


if __name__ == '__main__':
    raise SystemExit(main())
