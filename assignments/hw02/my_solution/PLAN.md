# HW02 implementation plan

Planning only. No planner implementation or runtime validation has been performed
for this branch yet. Review this plan before starting Phase 1; implement and
explain one phase at a time, then pause for review.

## Scope and isolation

- Branch: `hw02`, created directly from current course/fork `main` at
  `bc5c1a4`, not from any HW01 solution branch.
- Keep this work independent of HW01: no HW01 solution commits, imports, overlays,
  or changes to its files. Reusing the installed ROS distribution is fine.
- Put our package, tests, report, and required figures under this directory.
  Leave the supplied examples, shared environment, and public tests unchanged.
- This initial draft PR is for development review in the fork, not submission
  of a completed assignment to the course.

## Rules checked

Sources:

- [Assignment requirements](../instructions.md)
- [Environment setup](../environment/README.md)
- [Verification record and limitations](../environment/VERIFICATION.md)
- [Student examples and movement API](README.md)
- [Public checks](../tests/README.md)
- [Repository submission process](../../../README.md#submitting-an-assignment)

Use the current handout on `main`. It no longer labels HW02 as draft or requires
recorded videos. The live demonstration and report are still required.
Instructor verification results are not evidence that our solution passes.

Non-negotiable requirements:

- Plan in two dimensions with a 0.30 m radius: 0.25 m footprint plus 0.05 m margin.
- Read the actual map, start, and goal from ROS. No hardcoded obstacle positions,
  routes, or layout-dependent planning branches.
- Treat unknown cells and the exterior as occupied. Inflation must account for
  occupied-cell area, not just distances between cell centers.
- Implement our own PRM and A* on its graph. Numerical utilities are allowed;
  reference planners and library replacements for these algorithms are not.
- Validate the entire length of every edge, including contact with occupied
  cells at corners or boundaries.
- Use reproducible seeds and finite sampling, candidate-draw, and retry budgets.
  Roadmap failure is not a general proof that a physical path is impossible.
- Execute only a complete validated route, one asynchronous `go_to` at a time.
  Stop on failure and cancel pending movement on interruption.
- Keep generated builds, caches, and temporary outputs out of the PR.

## Implementation approach

Create one ROS 2 Python package, `hw02_solution`, under `src/`. Keep grid geometry,
roadmap construction, and graph search independent of ROS so they can be tested
without Gazebo. Use a thin ROS node for inputs, outputs, and waypoint execution.
Do not add perception, SLAM, Nav2, a controller, or a full-path action.

Sampling and neighbor parameters will be selected through measured tests, not
copied from instructor results. Use the same configuration across layouts unless
a documented, layout-independent bounded retry policy changes the budget.

## Phases and review checkpoints

### Phase 1 - Verify the HW02 environment

- Verify Ubuntu 24.04, ROS Jazzy, Gazebo Harmonic, and the HW02 dependencies.
- Build and source only the HW02 environment and ROS installation.
- Use consistent, isolated ROS domain and Gazebo partition settings; never mix
  real simulation with the fake publishers used by helper tests.
- Run the public environment/helper checks, readiness, and both unchanged
  examples. Check map orientation against the room.

**Review checkpoint:** exact commands and observed results, with any environment
blocker clearly separated from student-code problems. Restart after example motion.

### Phase 2 - Package and map inputs, with no movement

- Scaffold the package using ROS tools and declare dependencies.
- Subscribe to `/map`, `/hw02/start`, and `/hw02/goal` with reliable,
  transient-local QoS. Enable simulation time.
- Validate metadata, frames, dimensions, finite inputs, and world/grid conversion
  using map resolution and origin; preserve row-major indexing.
- Log invalid input explicitly. Wait for valid inputs without requesting motion.

**Review checkpoint:** the node receives late-published/static inputs correctly;
unit tests cover conversion, map extents, and invalid inputs.

### Phase 3 - Configuration space and edge collision checks

- Inflate occupied/unknown cells and account for room boundaries using the full
  planning radius and cell area.
- Implement one shared point/segment validity mechanism for PRM edges and
  start/goal connections.
- Reject segments that touch occupied configuration-space cells or leave the map,
  including diagonal corner grazing and grid-line cases.
- Avoid excessive inflation that unnecessarily removes passable gaps.

**Review checkpoint:** focused geometry tests demonstrate footprint clearance,
unknown/boundary handling, narrow-gap behavior, and collision-free endpoints whose
connecting edge is nevertheless blocked.

### Phase 4 - Build the PRM

- Sample only free configurations using an explicit seed.
- Connect neighbors with Euclidean costs only after full-edge validation.
- Insert the received start and goal using the same collision rules.
- Bound sample attempts and retries even when little or no free space exists.
- Record settings, retries, planning time, and node/edge counts.

**Review checkpoint:** repeated seeds reproduce the graph; every accepted edge
passes validation; disconnected start/goal cases terminate without movement.

### Phase 5 - A* and complete route validation

- Implement A* over the roadmap using Euclidean edge cost and heuristic.
- Reconstruct an ordered start-to-goal route and compute its length.
- Handle identical endpoints, unreachable goals, and invalid start/goal inputs
  explicitly. Never return a partial path as success.
- Test against small graphs with known shortest paths, then the received room
  maps. Keep route execution disabled.

**Review checkpoint:** graph-search tests pass; routes are complete and all
segments are valid; bounded failure is distinguishable from invalid input.

### Phase 6 - ROS outputs and saved figures

- Publish `/hw02/cspace` as `OccupancyGrid` with unchanged map metadata,
  `/hw02/roadmap` as `MarkerArray`, `/hw02/planned_path` as `Path`, and
  `/hw02/planner_status` as `String`, all reliable/transient-local.
- Use the `map` frame and the required statuses: `planning`, `path_found`,
  `executing`, `completed`, `no_path_found`, and `failed`.
- Save physical-map/configuration-space comparisons with samples, accepted edges,
  start/goal, and the selected route. Use correct world axes and lower-row origin.
- A no-path figure must not contain a fabricated or stale route.

**Review checkpoint:** RViz/late subscribers receive correct outputs, saved
figures match Gazebo coordinates, and planning alone still sends no movement.

### Phase 7 - Execute waypoints safely

- Use the provided `RobotClient`; keep spinning while awaiting each result.
- Start only after a complete route is validated and the movement helper is ready.
- Request the next waypoint only after success; stop the sequence on any failure.
- Cancel on interruption and do not publish competing velocity commands.
- For no-path runs, request no movement, publish `no_path_found`, and keep the node
  alive for at least three simulated seconds afterward.
- Check cancellation, action failure, and unavailable/stale simulation behavior.
  Restart the simulation after contact; never bypass the helper's safeguards.

**Review checkpoint:** one-at-a-time execution and cancellation tests pass;
a fresh `wide` run passes the independent observer started before our node.

### Phase 8 - Full experiments, report, and submission review

- Run fresh worlds for `wide` and `mixed` with seeds 0, 1, and 2, plus a bounded
  no-path run for `blocked`. Start the public observer before every run.
- Each solvable run must arrive within 0.10 m, remain stopped for two simulated
  seconds, and have fresh physics/contact data with no wall/pillar contact.
- The blocked run must report no path, stay within 0.03 m of its start, and remain
  stopped. Do not equate missing monitoring with a successful check.
- Account for observer timeouts in wall seconds, including planning/execution,
  separately from simulated-time stopping requirements.
- Record settings, retries, planning time, node/edge counts, path length,
  execution outcome, and observer result for each run.
- Write `report.md` with exact build/run commands, dependencies, readiness
  evidence, algorithm explanations, figures for all three layouts, and sampling
  limitations. Prepare explanations for the live demonstration.
- Check the final diff contains only HW02 student work. Confirm student/group
  names and the course destination/title before marking the submission ready.

**Review checkpoint:** reproducible results and complete deliverables, not just
successful action responses. No claim of completion before runtime checks pass.
