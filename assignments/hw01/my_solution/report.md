# HW1 report: camera-guided reaching with a two-joint arm

Package: [`hw01_solution`](src/hw01_solution/) · main node:
[`marker_reacher`](src/hw01_solution/hw01_solution/marker_reacher.py), built on
[`camera_listener`](src/hw01_solution/hw01_solution/camera_listener.py).

## 1. Message flow

```text
/camera/image_raw ──┐ ApproximateTimeSynchronizer        /camera/camera_info
/camera/depth/...  ─┴─ (slop 0.05 s, sensor-data QoS) ──┐       │ (cached, validated)
                                                        ▼       ▼
        red mask → one round region → pixel (u, v) → median depth Z → (X, Y, Z) optical
                                                                │  publish /hw01/target_camera
                                     TF2 lookup at image stamp  ▼
                                    base_link ← camera_optical_frame
                                                                │  publish /hw01/target_base
                         3 agreeing fresh observations, arm idle ▼
                       goal = target + (0, 0, 0.08) → ArmClient.move_to_point
                                                                │
                      result.success/message + /hw01/measured_tip check → log
```

All three image subscriptions use `qos_profile_sensor_data` (best effort). The
node forces `use_sim_time=True`, so all freshness checks use `/clock`.

## 2. Marker detection

1. **Decoding.** Images go through `cv_bridge` using their declared encoding
   (`rgb8` requested), after validating `step`/data length and removing row
   padding, so no raw byte layout is assumed.
2. **Color mask.** A pixel is a candidate if `R ≥ 150, G ≤ 100, B ≤ 100`. The
   marker is saturated red, and the grey/white scene and arm fail this rule.
3. **Region selection.** I use `cv2.connectedComponentsWithStats` with
   8-connectivity. Components under 20 px are treated as noise. Exactly one
   remaining component is required; two sizeable red regions are rejected as
   ambiguous instead of guessing by size.
4. **Reliability and occlusion checks.** The single region is rejected if:
   - it touches the image border (it may be cut off, which would bias its center);
   - its bounding box aspect ratio is below 0.75 (a partly covered disk becomes elongated);
   - its fill ratio is outside 0.55–0.90 (a disk fills about π/4 ≈ 0.79 of its box);
   - its contour circularity `4πA/P²` is below 0.70.

   These thresholds are conservative heuristics for this overhead camera, not
   guarantees.
5. **Pixel center.** The subpixel centroid `(u, v)` (column, row) of the accepted
   component. Each failure clears the previous detection, with a logged reason
   (for example "Red region is not round enough (possible occlusion)").

In the runs, the marker region was 87–91 px.

## 3. Depth selection and 3D reconstruction

**Depth selection.** Depth is sampled only inside the accepted marker mask, so
platform and background pixels never contribute. The depth image must be
`32FC1`, in the same frame and size as the RGB image (it is aligned). A sample is
valid if it is finite and within the camera clip range [0.05, 3.0] m. This rejects
NaN/±inf, zero, negative and out-of-range values. At least half of the mask
pixels must be valid, otherwise the observation is rejected. The estimate is the
**median** of the valid samples. The marker top is flat and faces the camera, so
its pixels share almost the same Z. The median ignores a few edge pixels that hit
the platform or return invalid values, where a mean or minimum would be pulled
off. The value is distance along the optical Z axis, not a Euclidean range. All
runs measured Z = 0.940 m, which matches the camera at 1.0 m above a marker top
at 0.06 m.

**Reconstruction.** The intrinsics come from the latest `CameraInfo`. Its frame
and size must match the image, and `fx`, `fy` must be finite and positive.
Nothing is hardcoded; the simulator reports fx = fy = 277.13, cx = 160, cy = 120.
With `k = [fx, s, cx; 0, fy, cy; 0, 0, 1]`:

```text
Y = (v − cy) · Z / fy
X = (u − cx − s · Y / Z) · Z / fx        (s = 0 here)
Z = median marker depth
```

This gives a point in `camera_optical_frame` (+X right, +Y down, +Z forward). It
is published as a `PointStamped` on `/hw01/target_camera` with the RGB header,
meaning that frame and the capture timestamp. Pixel and depth always come from
the same synchronized pair.

## 4. TF direction and timing

- **Direction:** `tf_buffer.lookup_transform(target='base_link',
  source='camera_optical_frame', time=image stamp)` returns T(base_link ←
  optical). `tf2_geometry_msgs.do_transform_point` applies it to the optical-frame
  point. The chain runs through the static `world → camera_link →
  camera_optical_frame` transforms published by the launch file; I didn't write
  any transform by hand.
- **Timing:** the lookup uses the image capture stamp, not "latest" and not wall
  time. The timeout is zero, so the camera callback never blocks the
  single-threaded executor. If the transform isn't available yet, the
  observation is rejected and the next frame tries again. The output point keeps
  the image stamp.
- **Sanity check:** in preset 1, optical (0.148, −0.052, 0.940) became base
  (0.352, −0.148, 0.060), and the base Z matches the platform's known 0.06 m top.
  The point is published on `/hw01/target_base` before any offset is added.

## 5. Motion

`goal = target_base + (0, 0, 0.08)` in `base_link` (Z ≈ 0.14 m). It is sent with
the provided `ArmClient.move_to_point` only when all of these hold:

- the arm is idle (`arm.busy` is false), so only one motion is ever active;
- the last 3 observations are fresh, consecutive and agree within 1 cm. The goal
  uses the latest observation, not an average of old ones;
- this target hasn't already been reached, and the retry delay has passed.

The returned future is handled with a done-callback, so the executor keeps
spinning and the camera callback never blocks. On completion the node logs
`success` and `message`. Since action success isn't proof of the physical
result, it also compares the goal with the physics-measured `/hw01/measured_tip`.
A "success" more than 0.02 m away counts as a failure. Failures are retried from
a new fresh observation after 2 s, at most 3 times.

## 6. Failure handling

| Situation | Behaviour |
|---|---|
| No red / tiny specks / several red regions | rejected; previous marker cleared |
| Border-touching, elongated, unfilled or non-round region (occlusion) | rejected |
| Depth invalid (NaN, inf, ≤0, out of range) or < 50 % valid in the mask | rejected |
| Undecodable image, wrong encoding, RGB/depth size or frame mismatch | logged error; rejected |
| Missing or invalid calibration, or calibration frame/size mismatch | rejected; nothing hardcoded is used instead |
| Image stamp > 0.5 simulated s old, or no `/clock` yet | rejected as stale |
| No synchronized pair for 2 wall-clock s (system-clock timer, works if sim pauses) | stored observations cleared; warning |
| TF lookup fails (lookup, connectivity, extrapolation) | rejected; no target published |
| Any rejected frame | stability history cleared, so an earlier target is never reused |
| Helper rejects the goal or motion fails | `message` logged; retry after 2 s, max 3 |
| Helper says success but measured tip is > 0.02 m off | counted as failure |
| Ctrl+C / SIGTERM | `ArmClient.cancel()`, spin up to 2 s for the cancel, then clean shutdown |

ROS's default signal handler would shut down the context before the cancel could
be sent. The node therefore installs its own SIGINT/SIGTERM handler, and the
pending goal is actually canceled. In my test the arm controller logged
"Canceling active action goal".

Once the tip hovers above the marker, the arm partly hides it from the overhead
camera. The node then logs occlusion rejections, which is expected; no new goal is
sent.

## 7. Results

Each preset was a fresh simulation start with the same code. `check_reach` was
started before the node. These runs used Ubuntu 24.04 / ROS 2 Jazzy / Gazebo
Harmonic in WSL2 with `gui:=false`.

| Preset | Pixel (u, v) | Z (m) | Camera point (m) | Base estimate (m) | Goal sent (m) | Helper | `check_reach` |
|---|---|---|---|---|---|---|---|
| 1 | (203.55, 104.73) | 0.940 | (0.148, −0.052, 0.940) | (0.352, −0.148, 0.060) | (0.352, −0.148, 0.140) | success, tip 0.0001 m from goal | **PASS**, tip error 0.0029 m, held 2 s, no contact |
| 2 | (159.46, 90.05) | 0.940 | (−0.002, −0.102, 0.940) | (0.402, 0.002, 0.060) | (0.402, 0.002, 0.140) | success, 0.0001 m | **PASS**, 0.0024 m, held 2 s, no contact |
| 3 | (115.36, 104.71) | 0.940 | (−0.151, −0.052, 0.940) | (0.352, 0.151, 0.060) | (0.352, 0.151, 0.140) | success, 0.0002 m | **PASS**, 0.0023 m, held 2 s, no contact |

The remaining 2–3 mm error comes from estimating the centroid from roughly
90 pixels at about 3.4 mm per pixel at this depth. It is well within the 0.02 m
tolerance.

Unit tests: `colcon test` gives 107 passed, 1 skipped (the generated copyright
check). The tests run without Gazebo and cover:

- synchronization boundaries and image decoding;
- the red mask, region selection and depth validity;
- back-projection math and the TF2 direction, using a known transform;
- stale, missing-TF, missing-calibration and missing-marker frames;
- camera silence, the one-motion limit, retry and the measured-tip check;
- cancel on interrupt.

## 8. Build and run commands

All commands start from the repository root, in an Ubuntu terminal without an
active Conda/venv.

Build (the shared environment is built once, per the installation guide):

```bash
source /opt/ros/jazzy/setup.bash
source assignments/hw01/environment/install/setup.bash
cd assignments/hw01/my_solution
colcon build --symlink-install --base-paths src --executor sequential
ROS_DOMAIN_ID=93 colcon test --packages-select hw01_solution
colcon test-result --verbose
cd ../../..
```

Terminal A, simulation (use `preset:=2` / `preset:=3` for the other runs; add
`gui:=false` for headless):

```bash
source /opt/ros/jazzy/setup.bash
source assignments/hw01/environment/install/setup.bash
ros2 launch hw01_sim simulation.launch.py preset:=1
```

Terminal B, checker, started **before** the solution:

```bash
source /opt/ros/jazzy/setup.bash
source assignments/hw01/environment/install/setup.bash
ros2 run hw01_sim check_reach --preset 1 --timeout 90
```

Terminal C, solution:

```bash
source /opt/ros/jazzy/setup.bash
source assignments/hw01/environment/install/setup.bash
source assignments/hw01/my_solution/install/setup.bash
ros2 run hw01_solution marker_reacher
```

Stop terminals C and A with Ctrl+C and restart all three for each preset.

## 9. Limitations

- Lens distortion is not corrected. The simulated camera has none; a real camera
  would need `cv2.undistortPoints` with `CameraInfo.d`.
- Color and shape thresholds are tuned for one saturated red disk under even
  lighting. A different marker or lighting would need retuning (an HSV mask
  would be more robust).
- After a success the node doesn't move again unless the estimate changes by more
  than 1 cm. That fits the stationary marker in this assignment, but it isn't a
  continuous tracker.
