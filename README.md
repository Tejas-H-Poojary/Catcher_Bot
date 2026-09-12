# turtlebot_pursuit_evasion

Catcher and Runner ROS2 nodes for the **TurtleBot Pursuit & Evasion
Challenge**, plus a standalone (no ROS2/Gazebo required) 2D simulator used
to develop and tune the underlying algorithms before they ever touch a
real robot or simulator install.

```
turtlebot_pursuit_evasion/
├── turtlebot_pursuit_evasion/       # the ROS2 ament_python package
│   ├── core/                        # pure-Python decision logic -- NO ROS imports
│   │   ├── geometry.py              # angle/frame math helpers
│   │   ├── potential_fields.py      # obstacle repulsion + VFH-style heading selection
│   │   ├── arena.py                 # arena/LIDAR model used ONLY by the standalone sim
│   │   ├── catcher_brain.py         # CatcherBrain: predictive pursuit
│   │   └── runner_brain.py          # RunnerBrain: VFH evasion + juking
│   ├── perception.py                # opponent localisation (ground-truth + LIDAR fallback)
│   ├── catcher_node.py              # ROS2 I/O wrapper around CatcherBrain
│   ├── runner_node.py               # ROS2 I/O wrapper around RunnerBrain
│   └── referee_node.py              # optional local match judge for testing
├── launch/match.launch.py           # brings up Catcher + Runner (+ referee)
├── config/params.yaml               # all tunable parameters, one place
├── simulate.py                      # standalone match simulator (`python3 simulate.py`)
├── package.xml / setup.py / setup.cfg / resource/   # ROS2 ament_python boilerplate
└── README.md                        # you are here
```

The most important design decision: **the actual decision-making logic
lives in `core/`, which has zero ROS dependencies.** `catcher_node.py` and
`runner_node.py` are thin I/O layers — they read sensors, call
`brain.compute_cmd(...)`, and publish the result. `simulate.py` calls the
exact same `compute_cmd(...)` against a lightweight Python-only arena
model. That means you can develop, test, and tune the algorithms with
nothing but `python3` (no ROS2 or Gazebo install needed), and know the
exact same code will run on the real stack.

---

## Assumptions we had to make

The rulebook you shared says a lot of the operational detail — "exact
Gazebo version, TurtleBot configuration, sensor setup, ROS/ROS2 version,
software requirements, and submission procedure" — **will be communicated
through the official technical rulebook**, which hasn't been released
yet. To make something you can actually build and run today, this package
assumes:

| Item | Assumption | Where to change it |
|---|---|---|
| ROS2 distro | Humble (TurtleBot 4's default) | Nothing distro-specific used; should also work on Iron/Jazzy |
| Gazebo | Classic + `gazebo_ros` bridge (`gazebo_msgs/ModelStates` on `/gazebo/model_states`) | `perception.py` — see the Fortress/Harmonic note there if the real sim uses `ros_gz_bridge` instead |
| Robot topics | Per-robot namespace (`catcher/`, `runner/`) with default TurtleBot 4 relative names: `odom`, `scan`, `cmd_vel` | `catcher_node.py`, `runner_node.py`, `launch/match.launch.py` |
| **How the Catcher/Runner know where the opponent is** | Ground-truth via Gazebo's model-state topic (`perception_mode: "ground_truth"`) | `config/params.yaml` — switch to `perception_mode: "lidar"` for a perception-only fallback (see below) |
| Robot footprint | 0.18 m radius, ~0.31 m/s max linear speed, ~1.9 rad/s max angular speed (TurtleBot 4 Lite spec-sheet ballpark) | `config/params.yaml` |

**The perception assumption is the one to revisit first** once the real
rulebook drops. Two modes are implemented in `perception.py`:

- **`ground_truth`** (default): reads the opponent's simulated pose
  straight from Gazebo. Simplest way to get pursuit/evasion, obstacle
  avoidance, and the capture logic all working and testable right away,
  and it's how a lot of educational pursuit-evasion competitions are
  actually judged even when the robots themselves don't get it.
- **`lidar`**: a "map-differencing" detector — it ray-casts a known
  static obstacle layout to predict what the LIDAR *should* see, then
  flags contiguous beams that come back shorter than expected as "the
  opponent is there." Closer to a real perception-only setup, at the
  cost of needing the static obstacle layout ahead of time and being
  noisier. Needs `static_obstacles` set in `config/params.yaml`.

If the real competition publishes a different way to sense the opponent
(a shared pose topic, a vision pipeline, etc.), swap it in — `catcher_node.py`
and `runner_node.py` only ever call `tracker.get_opponent_relative(own_pose)`
and don't care how that's implemented underneath.

---

## The algorithms

### Catcher — predictive pursuit + obstacle avoidance (`core/catcher_brain.py`)

1. While the opponent isn't detected, rotate slowly in place to sweep the
   LIDAR (mainly matters after the Runner breaks line of sight behind an
   obstacle).
2. Once detected, keep a short history of the opponent's estimated world
   position and finite-difference it into a velocity estimate.
3. Aim at a **predicted intercept point** a short lead time ahead of the
   opponent's current position, not its current position directly — this
   is what turns a tail-chase into a real pursuit, and it visibly cuts
   corners when the Runner has to divert around obstacles (see the
   trajectory plots below).
4. Blend that attraction vector with an obstacle-repulsion vector
   (classic artificial potential field from the LIDAR scan) and drive a
   heading-proportional controller towards the sum.
5. **Obstacle-influence radius shrinks as the Catcher closes in on the
   target.** Without this, a Runner backed up against a wall creates a
   textbook "goal near obstacle" potential-fields trap: the wall directly
   behind the target repels the Catcher exactly as hard as the target
   attracts it, and the Catcher stalls at a fixed standoff distance
   forever instead of finishing the capture. We hit this bug during
   tuning (see below) — once you're closer than the base influence
   radius, only obstacles nearer than the target itself are treated as
   "in the way."

### Runner — VFH-style evasion + juking (`core/runner_brain.py`)

1. Compute the direction directly away from the Catcher.
2. **Don't just drive that raw direction.** A pure "flee vector" has no
   way to know that "directly away from the Catcher" might be a dead end
   (a wall or a corner) until the Runner is already trapped there.
   Instead, every candidate heading the LIDAR can see gets scored on two
   things — how well it matches "away from the Catcher," and how much
   open space is actually that way — and the Runner drives the
   best-scoring one (`potential_fields.select_best_heading`, a small
   Vector Field Histogram). This is what lets it peel off along a wall or
   cut back through open space instead of running itself into a corner.
3. **Weave ("juke")** the preferred escape bearing side to side over time
   instead of holding one constant heading. A straight, unwavering flight
   is exactly what a predictive pursuer (see above) is built to cut off;
   periodically biasing the preferred angle confuses the Catcher's
   velocity estimate.
4. When the Catcher is far away or not detected, patrol with a slow
   random-walk turn biased toward open space, so the Runner keeps moving
   and doesn't telegraph its position by camping in one spot.

### Bugs we found and fixed while tuning against each other

Building `simulate.py` early paid off — three real bugs showed up
immediately when the two brains were run against each other repeatedly,
none of which would have been obvious from reading the code alone:

1. **Straight-line flight into corners.** The Runner's first version just
   fled directly away from the Catcher. Fixed by the VFH heading
   selection described above.
2. **The Catcher stalling just outside capture range.** The classic APF
   "goal near obstacle" trap described above — fixed by shrinking the
   obstacle-influence radius as the Catcher closes in.
3. **The Runner freezing in place next to an obstacle**, spinning on the
   spot while the Catcher walked in for an easy capture. Root cause: the
   simulated LIDAR ray-cast against obstacles' *true* radius (treating
   the robot as a point), while collision-checking used
   `obstacle_radius + robot_radius`. A direction could read as "clear"
   on the LIDAR while the robot's own body would still clip it. Fixed by
   having the simulated LIDAR reason in **configuration space** (inflate
   obstacles/walls by the robot's radius before ray-casting) — the same
   trick real ROS2 Nav2-style costmaps use, and worth keeping in mind if
   you ever build a costmap-based planner on the real TurtleBot instead
   of this reactive controller.

After these fixes, 20 varied test matches produced a believable ~35%
Catcher / ~65% Runner win split with capture times and final distances
that vary sensibly — not the suspicious identical numbers you get from a
stuck controller. Given both robots share the same top speed, the Runner
having a real edge if it evades well is expected, not a red flag.

---

## Running the standalone simulator (no ROS2/Gazebo needed)

```bash
# One match, prints the result
python3 simulate.py

# ...and save a trajectory plot
python3 simulate.py --seed 3 --plot match.png

# Batch-run to check win-rate balance while tuning
python3 simulate.py --matches 20 --seed 0
```

This is the fastest way to iterate on `core/catcher_brain.py` and
`core/runner_brain.py` — every parameter in `config/params.yaml` has a
matching constructor argument on `CatcherBrain`/`RunnerBrain`, so you can
try new values in `simulate.py` in seconds instead of rebuilding and
relaunching Gazebo.

---

## Building and running the ROS2 package

Requires ROS2 (Humble assumed) and Gazebo Classic + `gazebo_ros`, plus
whatever spawns the arena world and two TurtleBot 4 models per the
official rulebook once it's out (not included here — see "Assumptions"
above).

```bash
# From your colcon workspace's src/ directory:
colcon build --packages-select turtlebot_pursuit_evasion
source install/setup.bash

# Bring up both robots (assumes /catcher and /runner namespaced
# TurtleBot 4s are already spawned in Gazebo):
ros2 launch turtlebot_pursuit_evasion match.launch.py

# Disable the local referee (e.g. if the organizers' own judge is running):
ros2 launch turtlebot_pursuit_evasion match.launch.py use_referee:=false

# Run a single node standalone, e.g. to test just the Catcher:
ros2 run turtlebot_pursuit_evasion catcher_node --ros-args -r __ns:=/catcher
```

Watch the match live:
```bash
ros2 topic echo /match/distance
ros2 topic echo /match/status
```

---

## Tuning guide

Everything is exposed in `config/params.yaml`. The levers most worth
touching first:

- **Catcher too timid / bounces off obstacles too far away:** lower
  `obstacle_influence_radius` or `obstacle_gain`.
- **Catcher loses the Runner around corners:** raise `lead_time` (more
  predictive lead) or `heading_kp` (snappier turning).
- **Runner gets cornered too often:** raise `safe_distance` (VFH weighs
  open space more heavily) or lower `min_clearance` cautiously (accepts
  tighter gaps as viable escape routes).
- **Runner's evasion looks too jittery / wastes speed turning:** raise
  `heading_smoothing` (more inertia on the escape direction) or
  `min_turn_factor` (keeps more speed through turns).
- **Want a less predictable Runner:** shorten `juke_period` or widen
  `juke_angle` — but note this trades off top speed, since more time
  turning is less time running.

Re-run `python3 simulate.py --matches 20` after any change to sanity-check
you haven't traded a "too easy" Catcher for a "never catches anything"
one, or vice versa.

---

## Known limitations / next steps

- The `lidar` perception mode's opponent detector is a simple
  map-differencing heuristic — it will confuse "opponent" with "a static
  obstacle that moved into a beam it didn't expect" if the static
  obstacle layout you gave it doesn't match the real arena exactly, and
  it has no persistence/tracking across brief occlusions beyond
  `detection_max_age`. Good enough to develop against; worth hardening
  (e.g. a simple Kalman filter) once the real sensing setup is known.
- No unit tests are included yet (`package.xml` declares the usual
  `ament_copyright`/`ament_flake8`/`ament_pep257` test dependencies for
  when you add them).
- `RefereeNode` is for local testing only — for the real Round 2 knockout
  format (two 3-minute innings with roles reversed), you'll want to
  script switching each robot's `catcher_node`/`runner_node` role and
  parameters between innings, or just relaunch with roles swapped.
