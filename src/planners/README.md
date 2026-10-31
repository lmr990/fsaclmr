# planners

Path-planning nodes for the QUTMS / EUFS stack.

| Node | Purpose |
|---|---|
| `ft_planner_node` | FaSTTUBe `fsd_path_planning`: blue/yellow boundary sorting + matching + spline |
| `cone_id_mapper_node` | `/mapping/cone_map` -> `/urinay/cone_map` (stable IDs, geometry only) |
| `urinay_path_bridge_node` | urinay `PathLimits` -> `/planning/{midline_path,path_with_speeds}` |
| `planner_arbiter_node` | Shadow-planner comparison; never publishes control topics |
| `skidpad_planner_node` | Figure-eight template fit + analytic path + windowed `PathWithSpeeds` |

Parameters live in [`config/`](config); tests in [`test/`](test).

## Skidpad (figure-eight) mode

The regular planners cannot drive a skidpad: the centreline must self-intersect
(urinay forbids it), and ft's left/right matching fails where the two circles
cross. `planner:=skidpad` uses a dedicated mode instead:

1. `skidpad_fitting.py` fits two co-centric inner/outer cone rows per circle
   (shared centre, two radii) from `/mapping/cone_map`, re-fits after removing
   residuals > 0.6 m (at most 20%), and gates the result on counts, RMS/p90,
   centre distance, radius, ring width and alignment. Orange start-line cones
   are excluded by colour; the fit geometry itself is colour-blind.
2. After five consecutive good fits with < 0.15 m centre drift the template is
   frozen. No path is published before that.
3. `skidpad_geometry.py` generates the analytic path
   `ENTRY -> RIGHT_1 -> RIGHT_2 -> LEFT_1 -> LEFT_2 -> EXIT` at `ds = 0.25 m`
   and validates tangent continuity, arc lengths and curvature signs.
4. `skidpad_progress.py` maintains a monotonic arc-length progress. The node
   publishes only an 18 m window ahead of it, so the existing tracker's global
   nearest-point lookup is unambiguous at the intersection.
5. `planning/skidpad_state` (JSON) reports segment/lap/s_prog/state/fit
   metrics; `planning/skidpad_full_path` is a latched full path for RViz.

### Launch

Simulation (EUFS `skidpad` track, GT cone map for perception isolation):

```bash
ros2 launch qutms_eufs_bridge yolo_autonomous_loop.launch.py \
  track:=skidpad planner:=skidpad \
  skidpad_cone_map_topic:=/ground_truth/global_map \
  gazebo_gui:=false rviz:=false use_sim_time:=true \
  path_tracker_ff_gain:=0.0
```

Same without `skidpad_cone_map_topic` uses the live YOLO `/mapping/cone_map`.
From the EUFS start line the 10 m / 110 deg simulated camera sees only
13-26 blue/yellow boundary cones (most of both circles stays beside/behind
the forward FOV even while driving the entry), below the 40/16 fit gate, so
the production planner waits forever and the car never moves. Two ways out:

1. **GT cone map** (perception isolated, round 1):
   `skidpad_cone_map_topic:=/ground_truth/global_map`.
2. **YOLO warm-up, one command** (round 2): while the sim runs, run

   ```bash
   ros2 run planners skidpad_yolo_warmup
   ```

   It drives one GT lap to build the YOLO map, resets the car, and hands
   over to the production planner on `/mapping/cone_map`. Repeated warm-ups
   in the same session fragment the YOLO map (the script warns above 120
   boundary cones); restart the simulation for a fresh `cone_mapper`.
3. **One click in the test GUI**: select YOLO Autonomous mode and the
   skidpad track, then press Start Simulator — the launch guard forces
   `planner:=skidpad` and the GUI starts `skidpad_yolo_warmup`
   automatically once the pipeline is RUNNING. Stop Simulator sweeps the
   helper and its planner children.

The launch-level guard in `yolo_autonomous_loop.launch.py` forces
`planner:=skidpad` (and `path_tracker_ff_gain:=0.0` when left at the default)
whenever `track:=skidpad`, so a forgotten planner argument can no longer
produce the "drives straight through the crossing" failure mode.

Real car:

```bash
ros2 launch vehicle_bringup skidpad.launch.py
# or via the supervisor: set skidpad_mission:=true and select TRACKDRIVE
```

### Parameter notes

All parameters are in [`config/skidpad.yaml`](config/skidpad.yaml).

- `speed_cap: 1.2` is a **first-test value**, not a vehicle limit. It was
  lowered from 2.0 m/s because the unchanged tracker ran ~0.7 m wide at 2.0;
  at 1.2 m/s the circle RMS is ~0.22 m.
- `path_tracker_ff_gain:=0.0` is passed by the skidpad launch/GUI. Pure pursuit
  already contains the circle curvature; adding the tracker's curvature
  feedforward again double-counted it. `control.yaml` is unchanged and the
  default remains 1.0.
- `finish_margin: 3.0` (not 0.5) because the tracker's minimum lookahead is
  2.0 m: its target-speed point reaches zero ~2 m before the geometric end, so
  the car stops there. The state still reports `FINISHED` at the EXIT tail.
- The fit gate is intentionally not relaxed. If it does not pass, the node
  publishes no path and logs the failing reasons at 1 Hz.

### Acceptance

- Unit tests: `cd src/planners && python3 -m pytest test/ -q`
- Two-round EUFS acceptance and the RViz screenshot:
  [`docs/skidpad_acceptance.md`](docs/skidpad_acceptance.md),
  [`docs/skidpad_rviz.png`](docs/skidpad_rviz.png)

### Known limitations (v1)

- No ICP fine alignment; the template pose comes from the circle fit.
- YOLO cold start from the EUFS start line cannot pass the 40-cone gate; the
  recorded YOLO acceptance used a warm-up mapping lap (map from
  `/mapping/cone_map`, final planner still consumes the live YOLO map).
- Real-car handler/mission launch has not been exercised on hardware; only the
  simulation acceptance is claimed.
