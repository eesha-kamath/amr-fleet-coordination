# ARCNET Handoff: read this first

## 1. What this is
BEL problem statement 26123: decentralized coordination and collision avoidance for 3+ warehouse AMRs, running per-robot on edge hardware (Pi/Jetson-class). Success criteria: **zero inter-robot collisions** and **at least 20% lower task completion time than stop-and-wait** on overlapping paths. Deliverables: P2P comms, multi-agent path planning, conflict/deadlock resolution, task allocation and rerouting, fleet dashboard, and proof that robots keep working with the central computer disconnected.

## 2. Honesty rules (non-negotiable)
1. **No 20% figure has been measured.** Never state a number as a result until it comes from `metrics_log.csv`, smart vs baseline, same scenario.
2. `ALGO_REFERENCE.md` is a **design spec written as-if-implemented**. The Built/Roadmap table in section 8 is the source of truth for status.
3. Any metric on the pitch site is a labeled target, not a result.

## 3. Repo map (verify against repo)
- `fleet_dev/` (pure-Python brain, no ROS except `brain_node.py`): `brain_node.py`, `path_planner.py`, `conflict_resolver.py`, `lamport_clock.py`, `motion_controller.py`, `intent_codec.py`, `warehouse_grid.py`, `metrics.py`, `get_shelf_bbox.py`, plus standalone tests.
- `src/`: ROS2 workspace packages: modified `bcr_bot` (xacros parameterized with `robot_name`, `bcr_bot_multi_spawn.launch.py`, fleet bringup launch) and the AWS small-warehouse world.

## 4. Environment
Windows, WSL2 Ubuntu 24.04 (imported to D:), ROS2 Jazzy, Gazebo Harmonic, workspace `~/amr_ws`. Gotchas already solved:
- `gz` binary lives in `/opt/ros/jazzy/opt/gz_tools_vendor/bin`; add to PATH.
- Software rendering only (WSLg). **Render sensors (camera/lidar) are disabled fleet-wide**: three robots' sensors segfaulted Gazebo. Spawns are staggered.
- Bridge topics are namespaced: `/robotN/odom`, `/robotN/cmd_vel`. Teleop must remap to the namespaced topic.
- Large terminal pastes truncate in legacy PowerShell; use Windows Terminal or chunk them.

## 5. Run procedure
1. Start world + 3 robots (fleet bringup launch).
2. One terminal per robot: `python3 brain_node.py <robot> <goal_x> <goal_y> [smart|baseline]`
3. Results append to `~/amr_ws/fleet_dev/metrics_log.csv` (time, collisions, near-misses, e-stops, replans, deadlocks).
4. To reset, reset the sim in Gazebo before relaunching brain nodes.

## 6. How the brain works (per robot)
Loops: control 10 Hz, intent broadcast 2 Hz, replan every 3 s, deadlock check every 1 s. Peers share JSON on `/fleet/intent` (position, goal, planned cells/times, Lamport time, waiting_for, wait_started_at). All planned times are **absolute `time.time()`**, never per-robot elapsed (a past bug).
Key constants: grid 0.5 m cells, 100x100, origin (-20,-20); collision 0.45 m, hard stop 0.5, caution 0.7, near-miss 1.0, baseline stop 1.2; peer staleness 2.0 s planning / 1.5 s safety; congestion window 2.0 s, penalty 15; dynamic block radius 1 cell, horizon 2.5 s; yield 4 s; aging step 3 s, max 20.
Stop logic: under 0.5 m everyone stops (priority-blind); 0.5 to 0.7 m only the priority loser stops; baseline mode just stops within 1.2 m.

## 7. Bug history (do not reintroduce)
Guessed shelf geometry (robots drove into real shelves); soft-only planner allowed a real collision; relative timestamps made cross-robot times incomparable and broke aging; both robots stopping regardless of priority caused a 140 s freeze.

## 8. Status
**Built:** P2P broadcast, staleness gating, predictive A*, Lamport + aging, wait-for-graph deadlock recovery, three-tier safety, dual-mode metrics.
**Roadmap:** blocked-cell channel, D* Lite, heartbeat failure detection and task requeue, uncertainty radius on silence, ORCA smoothing (differential-drive: treat output as a steering target), QoS tiering, CBBA-style bidding (reuse Lamport tiebreak), proximity filtering (needs path-distance, not Euclidean), dead-end reverse maneuver, a killable central-assigner process.

## 9. Locked decisions
Headless Gazebo backend + custom 2D web dashboard over WebSocket; real Gazebo 3D only for demos and recordings; Vercel frontend + Render backend. **Open:** framework (custom frontend vs NiceGUI shell), and whether Render can run ROS2 + headless Gazebo within its limits (no GPU there). No kinematic-integrator replacement for now.

## 10. Work queue, in order
1. Run smart vs baseline on a forced choke-point scenario and record real numbers. Everything else is gated on this.
2. Blocked-cell channel + D* Lite.
3. Heartbeat detection, dead-robot obstacle and requeue.
4. Uncertainty radius.
5. ORCA as a standalone pure-Python module with a synthetic test, then wire in.
6. QoS tiering.
7. Task bidding.
8. Backend API/WebSocket, central-server process, dashboard tabs (setup grid, Live Monitor, Validation, Scenario Library, Architecture).
9. Containerize headless stack, test on Render.
10. Proximity filtering (last).

## 11. Conventions
Pure-Python modules with a standalone test before ROS wiring; small patches, not rewrites; no emojis or em dashes in UI/code output; simple comments.

## 12. Onboarding prompt for a new Claude session
"Read HANDOFF.md, ALGO_REFERENCE.md and DASHBOARD_PLAN.md in this repo. Follow the honesty rules. Start at work-queue item 1 unless told otherwise."
