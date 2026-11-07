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
Step 3A below now decodes and validates depth samples inside the selected marker.
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
The callback counts pairs and reports timestamps and their difference.
Step 2A, described below, now also converts the matched RGB image into pixels.
Depth-pixel validation, stale-data handling, marker detection, reconstruction,
transforms and motion are not implemented yet.

Build in an Ubuntu/WSL terminal. The first command moves to your clone of the
repository; replace the path with wherever you cloned it.

```bash
cd ~/robotics-course-HW
source /opt/ros/jazzy/setup.bash
cd assignments/hw01/my_solution
colcon build --symlink-install --base-paths src --executor sequential
cd ../../..
```

In terminal A (skip launching another simulation if one is already running):

```bash
cd ~/robotics-course-HW
source /opt/ros/jazzy/setup.bash
source assignments/hw01/environment/install/setup.bash
ros2 launch hw01_sim simulation.launch.py preset:=1
```

In terminal B, stop the old listener with Ctrl+C before running the updated one:

```bash
cd ~/robotics-course-HW
source /opt/ros/jazzy/setup.bash
source assignments/hw01/my_solution/install/setup.bash
ros2 run hw01_solution camera_listener --ros-args -p use_sim_time:=true
```

Expect individual-message logs (order can vary):

```text
Received image #1: 320x240, encoding=rgb8, ...
Received depth image #1: 320x240, encoding=32FC1, ...
Camera info #1: 320x240, fx=..., fy=..., cx=..., cy=..., frame=camera_optical_frame
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

## Step 2A: inspect RGB pixels

The synchronized-image callback now calls `update_rgb_pixels(rgb)`.
`CvBridge.imgmsg_to_cv2(image, desired_encoding='rgb8')` converts the ROS message
into a NumPy array with channel order red, green, blue, including when the input
encoding is `bgr8`. The latest successful result is stored in `latest_rgb_pixels`.

Decoding failures are logged as `Cannot decode RGB image (...)`; successful
decoding is silent.

The three entries of `shape` are height, width and color channels.
`uint8` holds whole-number channel values from 0 to 255.
Indexing uses `pixels[row, column]`, so `pixels[120, 160]` returns one RGB triplet.
The displayed center pixel is only an inspection example, not a detected marker
or a commanded target. Its values vary with the scene.

The converter checks dimensions, row stride and data length before decoding.
This installed CvBridge version mishandles non-pixel-aligned row padding, so padded
rows are packed on a copy before conversion; incoming messages are not modified.
Decoding errors are logged and clear the cached pixels rather than retaining an
older image. RGB pixel information is printed for the first pair and every 30th
pair. Restart the listener using the commands above after rebuilding.

Step 2A makes pixel values available for the red-color mask and marker-region
selection described below. There is still no depth reconstruction, TF
transformation or arm movement.

## Step 2B: highlight candidate red pixels

The synchronized-image callback passes the decoded RGB array to
`make_red_mask(pixels)`. This pure NumPy helper checks every pixel using the
starting rule `R >= 150`, `G <= 100`, and `B <= 100`.
The result is stored in `latest_red_mask`, a two-dimensional `uint8` array:

- `255` (white) means the pixel passes the red-color rule.
- `0` (black) means it does not.

This is a mask, not a new ROS topic or another RGB image. It has shape
`(height, width)`, with one flag per pixel instead of three color channels.
The mask itself is not logged. An image with no red pixels produces the warning
`Marker not detected: No red region.`; it does not reuse an earlier
mask. RGB decoding failures clear the mask too.

These thresholds are a simple, testable starting heuristic for the course camera,
not a complete object detector. Red noise or other red objects can also pass.
Step 2C below selects the marker region and checks its shape before accepting a
center. We do not yet publish a target or command movement.

## Step 2C: select a marker region and find its pixel center

`detect_marker(mask)` uses OpenCV connected components to group neighboring white
pixels. Regions smaller than 20 pixels are ignored as noise. Exactly one sizeable
region must remain; multiple candidates are rejected instead of choosing the
largest or guessing.

The detailed docstring in
[detect_marker](src/hw01_solution/hw01_solution/camera_listener.py) explains the
input, outputs and failure cases. In particular, `regions` is a list of integer
component IDs, not a list of images or pixel coordinates. OpenCV label 0 is
background and is never a candidate.

| Foreground component areas | IDs left after size filtering | Next decision |
| --- | --- | --- |
| Label 1: 91 pixels; label 2: 3 pixels | `[1]` | Check label 1's shape before accepting it |
| Label 1: 91 pixels; label 2: 80 pixels | `[1, 2]` | Reject as ambiguous; do not choose the larger one |

This uniqueness check happens before the shape tests below. The helper does not
check all sizeable shapes and then pick the most circular candidate.

The main detector delegates geometry validation to two small helpers:

- `_bounding_box_rejection` checks image-edge contact, box proportions and fill.
- `_outline_rejection` checks the selected region's contour and circularity.

Each returns a descriptive rejection string, or `None` if its checks pass.
The main detector turns a rejection string into `(None, reason)`; otherwise it
continues to calculate the center. These helpers do not select candidates or
compute pixel coordinates.

The remaining region must not touch the image border. Its shorter/longer bounding
box side ratio must be at least 0.75, its pixel area must occupy between 0.55 and
0.90 of that box, and its contour circularity must be at least 0.70. These
conservative heuristics suit the round marker viewed by the fixed overhead camera.
They reject obvious clipping, elongated/partly occluded regions, squares and thin
or hollow regions, but are not a guarantee against every possible occlusion or
red distractor.

An accepted `MarkerDetection` contains:

- `pixel`: `(u, v)`, the average column and row of the selected component's pixels.
  The values are floating point; they are not rounded or converted to metres.
- `mask`: only the selected region, excluding the small noise regions. Later
  depth sampling will use this mask rather than all red pixels.

The function also returns a reason explaining acceptance or rejection.
The node stores an accepted result in `latest_marker`. For example:

```text
Marker pixel: u=..., v=..., pixels=...
```

`u` increases to the right and `v` increases downward from the image's top-left.
Failures produce `Marker not detected: ...` warnings. A new missing, ambiguous
or malformed observation clears the previous marker; reason changes are reported
immediately, with periodic reports every 30th pair too.

This is still only an image-space observation. No 3D point, TF transformation,
target publication or arm movement is implemented. Timestamp/wall-clock freshness
guards for a stopped stream also remain necessary before commanding motion.

## Step 3A: measure depth inside the selected marker

`update_depth_pixels(depth, rgb)` checks the `32FC1` encoding, aligned dimensions
and matching nonempty optical-frame headers. CvBridge then decodes the message
with `desired_encoding='passthrough'` into a `(height, width)` float32 array.
Both RGB and depth use `_packed_image` for shared storage validation and safe
row-padding removal on a copy.

`measure_marker_depth(depth_pixels, detection.mask)` selects only the accepted
marker component, excluding background and small red-noise regions. Samples must
be finite and between 0.05 and 3.0 metres, matching the camera's clipping range.
These limits only validate sensor measurements; they do not specify or assume
the marker's position.

At least half of the selected pixels must have usable depth. The method then
returns their median, reducing the influence of isolated outliers. The median and
50-percent requirement are our depth-selection choices, not prescribed helper
functions or thresholds from the handout. The report should explain them.

Expected additional output:

```text
Marker depth: 0.940000m
```

The numerical depth can vary. It is distance along the camera optical Z axis,
not a Euclidean camera-to-marker range and not the marker's robot-frame height.
The latest current estimate is stored in `latest_marker_depth`.
Missing/unusable samples report a reason; malformed depth images report an error.
A failed new observation clears the previous depth rather than reusing it.

`camera_listener` itself still never moves the arm; Steps 3B-6 below build on it.

## Steps 3B-6: reconstruct, transform and reach (`marker_reacher`)

[marker_reacher](src/hw01_solution/hw01_solution/marker_reacher.py) subclasses
`CameraListener`, so it reuses all the detection and depth logic above, and adds:

1. **3D reconstruction (3B).** The intrinsic matrix `k` is read from the latest
   `/camera/camera_info`; nothing is hardcoded. The calibration frame and size must
   match the image. With pixel center `(u, v)` and optical-Z depth `Z`:
   `Y = (v - cy) * Z / fy` and `X = (u - cx - skew * Y / Z) * Z / fx`, giving a point
   in `camera_optical_frame`. It is published on `/hw01/target_camera`.
2. **TF2 (4).** A `PointStamped` with the RGB header (frame and capture stamp) is
   transformed into `base_link` using `tf2_ros.Buffer.lookup_transform` at the image
   time and `tf2_geometry_msgs.do_transform_point`. A failed lookup means no target.
   The marker estimate (before the offset) is published on `/hw01/target_base`.
3. **Hover goal (5).** 0.08 m is added to the base-frame Z (marker top ≈ 0.06 m,
   so goal Z ≈ 0.14 m) and the goal is sent with the provided
   `hw01_sim.arm_client.ArmClient.move_to_point`. The helper does the IK.
4. **Reporting / one motion (6).** A goal is sent only when no motion is active
   (`arm.busy`) and three consecutive fresh observations agree within 1 cm. The
   goal uses the latest observation, not an average. The future's `success` and
   `message` are logged. A reported success is also compared with the
   physics-measured `/hw01/measured_tip`; if it is more than 0.02 m away, it counts
   as a failure. Failures are retried from a fresh observation after 2 s, at most
   three times. After a success, the same target is not commanded again.

Rejecting stale or missing data:

- Simulated stamp older than 0.5 s relative to `/clock` → rejected. No `/clock`
  yet → rejected. `use_sim_time` is forced on inside the node.
- No synchronized pair for 2 wall-clock seconds (system-clock timer) → stored
  observations are cleared.
- Any rejected pair (no/ambiguous/occluded marker, invalid depth, no calibration,
  TF failure) clears the stability history, so an earlier target is never reused.
- Ctrl+C / SIGTERM cancel a pending motion through `ArmClient.cancel()` before exit.

Run (simulation and `check_reach` already started, see below):

```bash
source /opt/ros/jazzy/setup.bash
source assignments/hw01/environment/install/setup.bash
source assignments/hw01/my_solution/install/setup.bash
ros2 run hw01_solution marker_reacher
```

Expected output:

```text
Marker camera=(0.148, -0.052, 0.940) base=(0.352, -0.148, 0.060)
Sending hover goal (0.352, -0.148, 0.140) in base_link.
Motion succeeded: Trajectory completed (tip 0.0001 m from goal).
```

Once the tip hovers above the marker, the arm partly hides it from the overhead
camera, so "not round enough (possible occlusion)" warnings after success are expected.

## Test camera input handling

After building, run from the repository root. The reacher tests import the motion
helper, so source the shared environment too:

```bash
source /opt/ros/jazzy/setup.bash
source assignments/hw01/environment/install/setup.bash
cd assignments/hw01/my_solution
ROS_DOMAIN_ID=93 colcon test --packages-select hw01_solution
colcon test-result --verbose
cd ../../..
```

Use an unused ROS domain for tests; 93 is the example above.
The tests inject synthetic messages without spinning a camera
subscriber or launching Gazebo. They check both arrival orders, timestamps just
inside/on/outside the tolerance, independent counters, bounded buffering and
consumption of matched messages. RGB tests verify array shape/type, RGB/BGR channel
order, padded rows and explicit malformed-message errors with no stale fallback.
Red-mask tests cover known colors, exact threshold boundaries, binary shape/type,
input preservation and invalid arrays. They do not validate reliable marker
selection on arbitrary real scenes or arm motion. Marker-region tests cover
column/row order, subpixel centers, noise exclusion, the area boundary, ambiguity,
clipping, obvious occlusion, non-marker shapes and clearing old detections.
Depth tests cover selected-region sampling, median/outliers, invalid values,
clipping-range and validity-fraction boundaries, alignment, byte order, padded
rows and clearing old measurements.

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
