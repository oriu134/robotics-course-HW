# Your HW2 implementation

This directory contains two setup examples and our `hw02_solution` package. Complete the
[installation guide](../environment/README.md) before implementing your planner.

Phases 1-3 are verified. Our node validates inputs, builds configuration space,
and checks start/goal clearance. PRM, A*, and movement are not implemented yet. See the
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

## Build and run our geometry node (Phases 2-3)

In terminal B, from the repository root, build the student package:

```bash
source /opt/ros/jazzy/setup.bash
source assignments/hw02/environment/install/setup.bash
colcon --log-base assignments/hw02/my_solution/log build \
  --symlink-install --base-paths assignments/hw02/my_solution/src \
  --build-base assignments/hw02/my_solution/build \
  --install-base assignments/hw02/my_solution/install --executor sequential
source assignments/hw02/my_solution/install/setup.bash
```

Keep the simulation from terminal A running, then run in B:

```bash
export ROS_DOMAIN_ID=42 GZ_PARTITION=hw02_phase1_20261120
ros2 run hw02_solution planner_inputs
```

The node enables simulation time and waits for `/map`, `/hw02/start`, and
`/hw02/goal`, using reliable/transient-local subscriptions. Look for `Inputs ready:`
with map metadata, target coordinates, and `(row, column)` cells, ending with
`No motion.` The node first logs `Configuration space ready: radius=0.30 m;`
and the number of free cells. Press Ctrl+C to stop the node.

The small [grid module](src/hw02_solution/hw02_solution/grid.py) handles metadata
and world/cell-center conversions without ROS. The
[input node](src/hw02_solution/hw02_solution/planner_inputs.py) checks message
frames, finite planar coordinates, map shape, occupancy range, target bounds,
and start/goal footprint clearance.
It supports translated, axis-aligned maps and explicitly rejects rotated or
nonplanar maps. The supplied map has identity rotation.

Invalid updates are logged and clear readiness instead of reusing an older
accepted input. Raw unknown cells (`-1`) are preserved in the raw grid and treated
as occupied in configuration space. Cell indices use the resolution actually
received in the message: its float32 representation can place a point on a
nominal grid line in the preceding cell. No map coordinates are hardcoded.

### Configuration space and collisions (Phase 3)

The ROS-independent [collision module](src/hw02_solution/hw02_solution/cspace.py)
provides three functions:

- `inflate(grid)` returns a new grid with the same metadata and binary occupancy.
  Its fixed planning radius is 0.30 m. A cell is blocked if its closed area is
  within that radius of an occupied cell's closed area or the map exterior.
  The dilation kernel therefore uses square-to-square distances, not center
  distances. Unknown and all nonzero input occupancy values are obstacles.
- `edge_is_free(cspace, start, end)` checks the full segment analytically against
  closed occupied-cell rectangles in world coordinates. It does not sample at a
  fixed step, so thin obstacles, corner grazing, and grid-line contacts are checked.
  A one-ULP outward interval comparison treats numerical near-contact conservatively.
- `point_is_free(cspace, point)` uses the same logic with a zero-length segment.
  Nonfinite or out-of-map coordinates raise `ValueError`, never return free.

NumPy and SciPy perform numerical dilation only, not planning; both are declared
runtime dependencies and are already included in the HW02 environment.
Inflation is conservative at cell resolution: a free cell's whole area must clear
the footprint-expanded obstacles. No layout coordinates or names are used.

The node rebuilds C-space only when raw map contents/metadata change, clears it
when a map is rejected, and logs an error if start/goal touches an inflated obstacle.
It still has no movement interface. Publishing C-space/roadmap/path visualization
topics is planned for Phase 6; currently C-space is stored internally.

### Test the student package

After building and sourcing as above, run from the repository root:

```bash
ROS_DOMAIN_ID=85 colcon --log-base assignments/hw02/my_solution/log test \
  --base-paths assignments/hw02/my_solution/src \
  --build-base assignments/hw02/my_solution/build \
  --install-base assignments/hw02/my_solution/install \
  --packages-select hw02_solution --event-handlers console_direct+ \
  --return-code-on-test-failure
colcon --log-base assignments/hw02/my_solution/log test-result \
  --test-result-base assignments/hw02/my_solution/build --verbose
```

Domain 85 must be unused: these tests publish fake maps and targets, not real
simulation data. The grid and collision tests do not require ROS; the input tests use
isolated ROS nodes and do not require Gazebo.

## Use one-destination movement (for a later phase)

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

## Phase 2 verification results

Verified on 2026-11-20 using the same Ubuntu/ROS installation:

| Check | Observed result |
| --- | --- |
| Student package build | `hw02_solution` built; installed executable starts |
| Grid tests | 23 passed: row-major conversion, translated origin, cell centers, bounds/roundoff, invalid metadata, unknown values |
| ROS input tests | 8 passed: one-shot latched inputs published before subscription, invalid-update logging/clearing, recovery, new-map bounds, simulation time, no velocity publisher |
| Standard ROS lint tests | Flake8 and PEP 257 passed |
| Test result summary | 33 tests, 0 errors, 0 failures, 0 skipped |
| Pylance syntax checks | Both source modules and both behavior-test modules passed |
| Fresh headless `wide` world | Readiness passed; separately installed input node received all inputs after simulation startup |
| Independent physics monitoring | Three simulated seconds, zero displacement and velocity at nine decimal places, no obstacle contact, fresh pose/velocity/clock/contact data |
| ROS graph inspection | Input node has no velocity publisher or go-to service clients |

The generated Flake8 test emits Python 3.12 multi-threaded-fork warnings; lint
and behavior tests pass. Live validation used separate domain 86 and partition
`hw02_phase2_20261120`; all owned validation processes stopped afterward. Logs
are ignored under `build/hw02/phase2/`. This verifies input handling only, not
configuration space, planning, or route execution.

## Phase 3 verification results

Verified on 2026-11-20:

| Check | Observed result |
| --- | --- |
| Student test suite | 62 passed: 27 C-space/edge, 23 conversion, 10 ROS input, 2 lint; no failures or skips |
| Inflation oracle | Every cell in a small test map matches exhaustive closed-square/exterior distance calculations |
| Edge oracle | 6,036 segments plus reversals checked against exact rational world-rectangle intersections; no missed contacts, one conservative near-contact rejection |
| Regression cases | Thin obstacles, free endpoints with blocked interior, corner/grid-line contact, zero-length edges, and translated/float32-resolution boundary roundoff |
| ROS input updates | Colliding targets are rejected; changed obstacles rebuild C-space; invalid maps clear cached C-space |
| Fresh headless `wide` world | Installed node builds C-space once despite repeated map messages and accepts start/goal |
| Physics monitoring | Three simulated seconds, zero displacement/velocity to nine decimal places, fresh monitoring, no obstacle contact or movement interfaces |

An independent offline connectivity/physical-clearance audit used the supplied
raw maps with the received float32 resolution:

| Layout | Free cells after inflation | Start/goal connected | Direct start-goal edge |
| --- | --- | --- | --- |
| `wide` | 45,600 | Yes | Free |
| `mixed` | 41,796 | Yes | Blocked |
| `blocked` | 51,450 | No | Blocked |

All free-cell centers have at least 0.3125 m clearance from physical walls/pillars,
exceeding the required 0.30 m radius. Inflation took approximately 0.02-0.03 seconds
per supplied map on this machine. Connectivity labeling was a verification oracle
only, not a student planner: PRM and A* remain unimplemented.

Live checks used domain 86 and partition `hw02_phase3_20261120`; all owned processes
were stopped afterward. Logs are ignored under `build/hw02/phase3/`. The same
third-party Flake8 fork warnings noted in Phase 2 remain; all tests pass.
