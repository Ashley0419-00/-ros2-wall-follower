# Stage 2 Mission Manager — retrospective personal contribution portfolio

This portfolio was assembled after submission from the final COMP3431 Stage 2 team source. It documents the personal Mission Manager contribution described in Report 2, Section III.B: mission input integration, resolving saved landmark records to targets, retaining the requested visit order, and dispatching one goal at a time with completion checks. The submitted source verifies the behaviour shown below. The report supplies the contribution attribution, and individual line authorship has no independent commit-level verification here.

## Design and team boundaries

The Mission Manager keeps route policy at the application level. It consumes ordered landmark records and their corresponding IDs. The submitted `navigation_data.py` provides the integrated CSV parsing, saved start pose and numbered-route resolution. The mission loop calls the integrated approach-pose selection and sends the resulting pose through Nav2. It advances only after the action succeeds and the map-to-base translation is available and within 0.25 m of the dispatched pose. That check measures proximity to the stopping pose.

The saved map, camera and landmark mapping pipeline, map export, localization and TF setup, map-health checks, launch/configuration and supporting navigation helpers belong to the integrated team project. Nav2 supplies planning and motion execution, and AMCL provides localization. The existing repository attribution applies to the wall-following and vision materials already present. See [ATTRIBUTION.md](../ATTRIBUTION.md).

## Verified excerpt

Source: final submission archive `src.zip`, member `src/src/wall_follower/scripts/waypoint_navigator.py`, `MarkerNavigation.navigate()`, lines 404–428. The excerpt removes the enclosing method indentation. The enclosing node, approach-pose search, ROS plumbing and shared modules are required for deployment. This excerpt is documentation.

- Archive SHA-256: `eb35ba9647faee6e73e6df1a26bdfa30f5b9f08111eba63222520c848118fe6c`
- Source member SHA-256: `f6c21207768e86c6711b4cd6b58f162a1df4d29d1d1b75179885c3bdfa5096a6`

```python
for number, marker in enumerate(markers, 1):
    ident=self.options.route_numbers[number-1]
    label = f'marker{ident} (goal {number}/{len(markers)}), type {marker.kind}'
    goal = self.choose_goal(marker, planner, grid)
    position = goal.pose.position
    gap = math.hypot(position.x - marker.x, position.y - marker.y)
    self.get_logger().info(f'{label}: goal ({position.x:.2f}, {position.y:.2f}), '
                           f'{gap:.2f} m from marker')
    goal.header.stamp = self.get_clock().now().to_msg()
    request = NavigateToPose.Goal()
    request.pose = goal
    started=time.monotonic()
    self.events.append(dict(marker=ident,goal=[position.x,position.y],state='started'))
    result = self.action(navigator, request, self.options.goal_timeout, label)
    self.events[-1].update(state='succeeded' if result.status==GoalStatus.STATUS_SUCCEEDED else 'failed',
                           status=result.status,elapsed_seconds=time.monotonic()-started)
    if result.status != GoalStatus.STATUS_SUCCEEDED:
        detail = getattr(result.result, 'error_msg', '')
        raise RuntimeError(f'{label} was not reached; action status {result.status}. {detail}')
    final=self.lookup('map',self.options.base_frame)
    if final is None or math.hypot(final.translation.x-position.x,final.translation.y-position.y)>.25:
        self.events[-1]['state']='position_check_failed'
        raise RuntimeError(f'{label} reported success, but the final-position check failed.')
    self.events[-1]['final_position']=[final.translation.x,final.translation.y]
    self.get_logger().info(f'{label} reached.')
```

## Isolated software verification

Six cases passed in a local Python harness that ran the submitted loop with synthetic markers and stubbed pose selection, action responses, TF, clock and logging.

| Case | Observed behaviour |
| --- | --- |
| Supplied route 1 → 2 → 3 | Dispatches all three goals in order |
| Supplied route 3 → 1 → 2 | Dispatches all three goals in order |
| Aborted action | Raises before dispatching the second goal |
| Missing final transform | Raises before dispatching the second goal |
| Final translation 0.50 m from the goal | Raises before dispatching the second goal |
| Timeout exception from the action stub | Propagates the exception before dispatching the second goal |

These checks cover loop control flow after route resolution. CSV parsing and target selection were outside the harness. Real ROS action timing, cancellation, TF accuracy, planning, simulated movement and physical robot performance remain outside the scope of this verification.
