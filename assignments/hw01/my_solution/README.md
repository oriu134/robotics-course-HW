# Your HW1 implementation

This folder contains a small installation example and the student package being
developed step by step. Read [the assignment](../instructions.md) and finish the
[environment setup](../environment/README.md) first.

## Steps 1A-1C: receive and synchronize the camera inputs

The [student package](src/hw01_solution/) currently contains only
[camera_listener](src/hw01_solution/hw01_solution/camera_listener.py).
Step 1A receives RGB; Step 1B adds depth and calibration; Step 1C matches RGB/depth
capture timestamps. All three subscriptions use sensor-data QoS:

| Topic | Message | What the listener reports |
| --- | --- | --- |
| `/camera/image_raw` | `sensor_msgs/msg/Image` | RGB dimensions, encoding, frame and timestamp |
| `/camera/depth/image_raw` | `sensor_msgs/msg/Image` | Depth dimensions, encoding, frame and timestamp |
| `/camera/camera_info` | `sensor_msgs/msg/CameraInfo` | Dimensions, focal lengths, principal point, frame and timestamp |

Each stream logs its first message and every 30th message afterward.
RGB and depth share a small metadata-logging helper. Camera info provides the
intrinsic matrix `k`; entries 0/4 are `fx`/`fy` and entries 2/5 are `cx`/`cy`.
Depth uses `32FC1`: one 32-bit floating-point depth value per pixel, in metres.
The listener only reports metadata; it does not inspect or validate depth pixels.
Individual-message callbacks still run independently. RGB/depth subscriptions
now use `message_filters.Subscriber`, a wrapper around a ROS subscription.
`ApproximateTimeSynchronizer` buffers up to ten messages per image stream and
calls `on_synchronized_images(rgb, depth)` when their header timestamps are close.
It uses capture time, not callback arrival time or the independent message counts.
Camera calibration remains a separate subscription.

The synchronizer has `slop=0.05` seconds. The standard ROS helper uses a strict
comparison: timestamp differences **less than 0.05 simulated seconds** are paired;
exactly 0.05 seconds or greater are not paired. This stays within the assignment's
50 ms limit without changing the ROS helper. Unmatched messages wait for a suitable
counterpart and the oldest timestamps are discarded when a buffer fills.
The new callback only counts pairs and reports timestamps and their difference.
Depth-pixel validation, stale-data handling, marker detection, reconstruction,
transforms and motion are not implemented yet.

Build in an Ubuntu/WSL terminal. The first command selects the current clone;
change that path if you move the repository.

```bash
cd "/mnt/c/Users/Or/Desktop/Robotics Assignments/robotics-course-HW"
source /opt/ros/jazzy/setup.bash
cd assignments/hw01/my_solution
colcon build --symlink-install --base-paths src --executor sequential
cd ../../..
```

In terminal A (skip launching another simulation if one is already running):

```bash
cd "/mnt/c/Users/Or/Desktop/Robotics Assignments/robotics-course-HW"
source /opt/ros/jazzy/setup.bash
source assignments/hw01/environment/install/setup.bash
ros2 launch hw01_sim simulation.launch.py preset:=1
```

In terminal B, stop the old listener with Ctrl+C before running the updated one:

```bash
cd "/mnt/c/Users/Or/Desktop/Robotics Assignments/robotics-course-HW"
source /opt/ros/jazzy/setup.bash
source assignments/hw01/my_solution/install/setup.bash
ros2 run hw01_solution camera_listener --ros-args -p use_sim_time:=true
```

Expect individual-message logs and a synchronized-pair log (order can vary):

```text
Received image #1: 320x240, encoding=rgb8, ...
Received depth image #1: 320x240, encoding=32FC1, ...
Received camera info #1: 320x240, fx=..., fy=..., cx=..., cy=..., ...
Synchronized pair #1: RGB stamp=..., depth stamp=..., delta=0.000000000s
```

All should identify `camera_optical_frame` and include capture timestamps in
simulation time. Additional lines appear as messages arrive. The node initially
reports that it is waiting if the simulation is not running.
`use_sim_time:=true` makes the node's clock follow simulation time; the printed
timestamps are taken directly from the incoming message headers.
Both individual streams and synchronized pairs log their first item and every
30th item afterward. Their counters need not agree. A zero `delta` means the
two images have identical capture timestamps; a small nonzero difference is
accepted only below the synchronization tolerance.
Stop the listener with Ctrl+C.

## Test the timestamp matching

After building, run from the repository root:

```bash
source /opt/ros/jazzy/setup.bash
cd assignments/hw01/my_solution
ROS_DOMAIN_ID=93 colcon test --packages-select hw01_solution
colcon test-result --verbose
cd ../../..
```

Use an unused ROS domain for tests; 93 is the example above.
The synchronization tests inject synthetic messages without spinning a camera
subscriber or launching Gazebo. They check both arrival orders, timestamps just
inside/on/outside the tolerance, independent counters, bounded buffering and
consumption of matched messages. They do not test perception or arm motion.

## Run the example

Start the simulation in terminal A from the repository root after completing
[the environment setup](../environment/README.md#5-start-the-simulation):

```bash
source /opt/ros/jazzy/setup.bash
source assignments/hw01/environment/install/setup.bash
ros2 launch hw01_sim simulation.launch.py preset:=1
```

Keep terminal A running. In terminal B, from the repository root:

```bash
source /opt/ros/jazzy/setup.bash
source assignments/hw01/environment/install/setup.bash
python3 assignments/hw01/my_solution/check_setup.py
```

The example prints the RGB image dimensions and encoding, then moves the arm to
`(0.20, -0.20, 0.14)` metres. Expect two PASS lines: fresh camera reception and
fixed-point arm movement. Failure returns a nonzero exit code. This does not check
or solve marker detection, depth reconstruction, or coordinate transforms.

## Use the helper in your own node

```python
from geometry_msgs.msg import PointStamped
from hw01_sim.arm_client import ArmClient

arm = ArmClient(your_ros_node)
point = PointStamped()
point.header.frame_id = 'base_link'
point.point.x, point.point.y, point.point.z = 0.20, -0.20, 0.14
if arm.ready():
    future = arm.move_to_point(point)
```

Keep spinning your ROS node while the motion runs. When `future.done()` is true,
`future.result().success` and `.message` describe the outcome. Do not block your
subscription callback waiting for the result. Call `arm.cancel()` to cancel a
pending motion. Only one motion may be active; coordinates use metres, and this
arm's fixed tip height is 0.14 m. The helper handles kinematics for you.
