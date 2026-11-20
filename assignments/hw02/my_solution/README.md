# Your HW2 implementation

This directory contains only two setup examples. Complete the
[installation guide](../environment/README.md) before implementing your planner.

Phase 1 is verified; planner implementation has not started. See the
[implementation plan](PLAN.md) for the next phases and review checkpoints.

## Build and check the environment

Run these commands in Ubuntu/WSL Bash, not PowerShell, from the repository root.
Use Ubuntu Python without an active Conda/venv. Source only ROS Jazzy and HW02,
not the HW01 overlay.

```bash
source /opt/ros/jazzy/setup.bash
cd assignments/hw02/environment
colcon build --symlink-install --base-paths src --executor sequential
source install/setup.bash
cd ../../..
ROS_DOMAIN_ID=84 python3 -m pytest -q \
  assignments/hw02/tests/test_environment.py \
  assignments/hw02/tests/test_motion.py
```

Stop the simulation before running the helper tests: they publish fake robot
feedback. Domain 84 must be unused; the real simulation below uses domain 42.

## Run the examples

From the repository root, terminal A:

```bash
source /opt/ros/jazzy/setup.bash
source assignments/hw02/environment/install/setup.bash
export ROS_DOMAIN_ID=42
export GZ_PARTITION=hw02_phase1_20261120
ros2 launch hw02_sim simulation.launch.py layout:=wide gui:=false
```

Keep A running. In terminal B, also from the repository root:

```bash
source /opt/ros/jazzy/setup.bash
source assignments/hw02/environment/install/setup.bash
export ROS_DOMAIN_ID=42
export GZ_PARTITION=hw02_phase1_20261120
ros2 run hw02_sim readiness_demo
python3 assignments/hw02/my_solution/read_map.py
python3 assignments/hw02/my_solution/move_demo.py
ros2 run hw02_sim readiness_demo
```

`read_map.py` prints map metadata and saves the **raw**, uninflated map as NPZ and
PNG under ignored `build/hw02/demo_map/`. `move_demo.py` moves to `(1.5, 3)` and
returns to `(1, 3)`, checking measured positions. Both fail with a nonzero exit
code if prerequisites or movement fail. Neither example solves the assignment.
Use the same domain and partition in every HW02 terminal; choose different unused
values if these are already taken. `gui:=false` runs headless; omit it to request
the Gazebo GUI. Interactive GUI/RViz was not part of this verification.

## Use one-destination movement

```python
from hw02_sim.robot_client import RobotClient

robot = RobotClient(your_ros_node)
if robot.ready():
    future = robot.go_to(1.5, 3.0)
```

Keep spinning your node. When `future.done()` is true, inspect
`future.result().success` and `.message`. Call `robot.cancel()` on interruption.
`ready()` checks action-server discovery; run readiness to check the complete setup.

For your own planned route, request one waypoint at a time and wait for a successful
result before requesting the next. Stop on failure. The helper turns and drives
directly; it does not avoid obstacles or execute an entire path for you.

A manual equivalent, after sourcing, is:

```bash
ros2 run hw02_sim go_to --x 1.5 --y 3.0
```

Use clear destinations for manual tests. Restart the simulation before assignment
demonstrations to restore the initial robot pose and clear latched contacts.

See [instructions.md](../instructions.md) for planning, visualization and report
requirements. Commit source, report and required figures, not caches/build products.

## Phase 1 verification results

Verified on 2026-11-20 in WSL2 Ubuntu 24.04.4, Python 3.12.3, ROS Jazzy,
Gazebo Sim 8.15.0, NumPy 1.26.4, SciPy 1.11.4, and Matplotlib 3.6.3.
All documented HW02 dependencies were already installed; no installation or
changes to the supplied Python examples were needed.

| Check | Observed result |
| --- | --- |
| Sequential environment build | All three HW02 packages built successfully |
| Combined public environment/helper tests | 21 passed in 16.71 seconds, isolated domain 84 |
| ROS talker/listener in separate processes | Listener received `Hello World` messages |
| Fresh headless `wide` readiness | All seven checks passed, before and after motion |
| Raw map example | Saved 320 x 240 map, 0.025 m resolution, origin `(0, 0)`, frame `map` |
| Map inspection | Saved X/Y plot matches the supplied `wide` room figure |
| Movement example | Fresh feedback, outward movement, and return-to-start checks passed |

These are setup results, not PRM/A* or full-navigation results. Other layout/seed
execution, interactive GUI/RViz, and normal interactive shutdown are not yet
verified. Automated shutdown required targeted cleanup of a remaining Gazebo
process, consistent with the shared environment's documented shutdown limitation.
All verification processes were stopped afterward. Wait for old processes to exit
before restarting; do not run a second world in the same partition.

Local run logs are ignored under `build/hw02/phase1/`; the saved raw map is under
`build/hw02/demo_map/`. Restart the simulation before the next phase or demo.
