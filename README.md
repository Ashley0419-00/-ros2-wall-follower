# ROS 2 Autonomous Wall Follower

A portfolio-ready ROS 2 robotics project for TurtleBot3 that combines LiDAR-based wall following, obstacle and wall-end manoeuvres, odometry-based return detection, and camera-based coloured landmark mapping.

![Wall following and mapped landmarks](docs/wall-following.png)

## What it demonstrates

- **Autonomous navigation:** fits local wall segments from `LaserScan` data and steers at a configurable stand-off distance.
- **State-based manoeuvres:** handles corridors, front obstacles, open left exits, wall endpoints, recovery, and wall reacquisition.
- **Motion safety:** applies scan freshness checks, adaptive braking, odometry-aware speed limiting, and bounded commands.
- **Return detection:** tracks travelled distance and stops after returning within a configurable radius of the start pose.
- **Visual landmarks:** segments two-colour markers with OpenCV, estimates range and bearing from camera intrinsics, and confirms detections across frames.
- **Map integration:** transforms marker observations into the `map` frame, rejects outliers, publishes RViz markers, and exports landmark coordinates.

## System architecture

```mermaid
flowchart TD
  A[LiDAR /scan] --> B[Wall follower C++]
  C[Odometry /odom] --> B
  B --> D[Velocity /cmd_vel]
  E[Camera image + intrinsics] --> F[Marker detector]
  F --> G[/marker_position]
  G --> H[TF point transformer]
  I[Cartographer /map + /tf] --> H
  H --> J[RViz MarkerArray + CSV]
```

## Repository layout

```text
src/                    Advanced C++ wall-following controller
include/                ROS 2 node interface
scripts/                OpenCV marker detection and TF transformation
wall_follower/           Python landmark model
launch/                  Integrated ROS 2 launch file
simulation_models/       Custom Gazebo maze and colour-marker models
docs/                    Validation notes and screenshots
```

## Prerequisites

- Ubuntu with ROS 2 and `colcon`
- TurtleBot3 Gazebo and Cartographer packages
- OpenCV, `cv_bridge`, `tf2_ros`, `tf2_geometry_msgs`, and `visualization_msgs`
- TurtleBot3 Waffle Pi simulation or compatible topics:
  - `/scan`
  - `/odom`
  - `/camera/image_raw`
  - `/camera/camera_info`
  - `/tf`

The package is ROS 2 API-compatible but was not containerized because Gazebo, ROS middleware, graphics, and robot drivers depend on the host environment.

## Build

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/Ashley0419-00/-ros2-wall-follower.git wall_follower
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select wall_follower
source install/setup.bash
```

## Run in simulation

Open three terminals and source the workspace in each one.

```bash
# Terminal 1: Gazebo maze
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch turtlebot3_gazebo turtlebot3_maze.launch.py

# Terminal 2: mapping
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True

# Terminal 3: navigation and landmark nodes
ros2 launch wall_follower wall_follower.launch.py
```

The custom models in `simulation_models/` can be added to Gazebo's model path when reproducing the landmark scene.

## Key parameters

The C++ node exposes these runtime parameters:

| Parameter | Default | Purpose |
| --- | ---: | --- |
| `max_linear_speed` | `0.40` m/s | Upper linear-speed bound |
| `brake_deceleration` | `0.35` m/s² | Conservative braking model |
| `return_radius` | `0.35` m | Stop zone around the start pose |
| `min_return_path` | `2.5` m | Minimum path before return detection arms |
| `min_return_time` | `8.0` s | Minimum movement time before arming |

Example:

```bash
ros2 run wall_follower wall_follower --ros-args \
  -p max_linear_speed:=0.30 \
  -p return_radius:=0.30
```

## Validation

One recorded simulation run covered an estimated **21.44 m** over **232 s** and triggered the return stop at an estimated **0.33 m** from the starting pose with a **0.35 m** threshold. See [the validation note](docs/validation.md) for scope and caveats.

## Portfolio scope

This is a team-based academic robotics project, curated here to demonstrate ROS 2 integration, perception-to-control workflows, state-machine design, debugging, and validation. Source-level attribution and license notices are preserved in [ATTRIBUTION.md](ATTRIBUTION.md); claims about individual contribution should remain aligned with the contributor's actual work.

## Safety

The controller is intended for education and simulation. Before using it on physical hardware, add an emergency-stop mechanism, validate sensor failure behaviour, tune limits for the platform, and test in a controlled area.

## License

Different files retain their original licenses. The modified wall-follower and vision sources are GPLv3; the ROBOTIS-derived interface header is Apache-2.0. See `LICENSES/` and the source headers.
