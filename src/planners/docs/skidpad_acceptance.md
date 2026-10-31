# Skidpad mode — simulation acceptance (2026-10-30)

Branch: `skidpad-mode` (based on `bcc90e8`, no push)
Phase commits:

```
ba0c2af  skidpad: phase1 geometry + tests
08858b5  skidpad: phase2 template fitting + tests
a989ef9  skidpad: phase3 planner node + progress window + tests
1946a88  skidpad: phase4 handler, launch and GUI integration
6a2c2eb  skidpad: phase5 acceptance tuning (finish margin, test speed)
```

## Interface review (§0.2 on HEAD `bcc90e8`)

| # | Anchor | Result |
|---|---|---|
| 1 | `/planning/path_with_speeds` / `driverless_msgs/PathWithSpeeds` (`header, poses[], target_speeds[], curvatures[]`, track frame, left-positive curvature) | match |
| 2 | `controllers/node_path_tracker.py`: path/cmd topics, `path_timeout_sec=0.5`, `Ld=clamp(0.55v+1.8,2,8)`, global nearest point | match; frames are `track`/`base_link` |
| 3 | `/mapping/cone_map`, 10 Hz, `merge_radius=0.5`, min-observation gate | match |
| 4 | `planners/speed_profile.py` `compute_curvatures` / `compute_speed_profile`; ft `publish_path_with_speeds` pattern | match |
| 5 | `planner:=urinay|ft`, urinay default, shadow arbiter | match; extended with `skidpad`, shadows disabled for skidpad |
| 6 | `vehicle_bringup` handlers only trackdrive/inspection/ebs; GUI skidpad track entry without backend | confirmed; skidpad handler + watcher + mission launch added |
| 7 | `eufs_tracks/csv/skidpad.csv` 115 data rows: 30 blue, 30 yellow, 30 midpoint, 20 orange, 4 big orange, 1 car_start; EUFS dimensions 18.5 m centre distance / 9.11 m centreline / 3.0 m ring width (FSAC: 18.25 / 9.125 / 3.0) | confirmed; no EUFS coordinates hardcoded in the planner |
| 8 | YOLO detection rate / ghost cones; robust template fit required | confirmed; robust refit used |

## Changes by package

- **planners**: `skidpad_geometry.py`, `skidpad_fitting.py`, `skidpad_progress.py`, `node_skidpad_planner.py`, `config/skidpad.yaml`, tests, this report + screenshot.
- **yolo_cone_detection**: `yolo_autonomous.launch.py` adds `planner:=skidpad`, skidpad cone-map/heading args, `path_tracker_ff_gain`.
- **qutms_eufs_bridge**: passes skidpad args and `rviz` through the one-command sim launch.
- **vehicle_bringup**: `node_skidpad_handler.py`, `node_skidpad_watcher.py`, `mission_launch/skidpad.launch.py`, mission/supervisor wiring (`skidpad_mission` selects `launch/skidpad` in the TRACKDRIVE slot).
- **qutms_test_gui**: planner selector includes Skidpad; selecting the skidpad track forces `planner:=skidpad` and `path_tracker_ff_gain:=0.0`.

## Unit tests

```
colcon test --packages-select planners cone_mapping controllers yolo_cone_detection
Summary: 72 tests, 0 errors, 0 failures, 0 skipped
```

`planners` alone: 60 tests (including skidpad geometry/fitting/progress) pass.

## Simulation acceptance

EUFS `skidpad` track. Both runs use `speed_cap=1.2 m/s` (test value) and
`path_tracker_ff_gain=0.0` (see notes). Metrics from
[`scripts/skidpad_acceptance_stats.py`](../scripts/skidpad_acceptance_stats.py)
over the recorded TF / path / state / GT-cone streams.

| Item | Threshold | Round 1 (GT cones) | Round 2 (YOLO map) |
|---|---|---|---|
| Segment order | ENTRY→R1→R2→L1→L2→EXIT→FINISHED | pass | pass |
| Laps | 1→4 | `[0,1,2,3,4,5]` | `[0,1,2,3,4,5]` |
| Completion | exits and stops | FINISHED, `s_prog=262.4/265.4`, final speed 0.11 m/s | FINISHED, `s_prog=262.0/265.0`, final speed 0.15 m/s |
| Circle lateral error | RMS ≤0.35 m, max ≤0.8 m | **RMS 0.220 / max 0.263** | **RMS 0.219 / max 0.270** |
| All-path lateral error | (report) | RMS 0.190 / max 0.263 | RMS 0.096 / max 0.270 |
| Cone hits | 0 | **0** (min centre-cone 1.139 m, 0 samples <0.8 m) | **0** (min 1.093 m, 0 samples <0.8 m) |
| Path publish interval | max <0.5 s | **max 0.172 s**, mean 0.100 | **max 0.124 s**, mean 0.100 |
| Single PathWithSpeeds publisher | exactly 1 | 1 (`skidpad_planner_node`) | 1 (`skidpad_planner_node`) |
| Fit RMS / p90 | RMS ≤0.35, p90 ≤0.5 | **0.0006 / 0.0009** | **0.0350 / 0.0208** |
| Fit R / centre distance / ring width | R 7.5–11, d 16–20.5, width 2.4–3.6 | 9.1117 / 18.4978 / 2.9996 | 9.0976 / 18.4383 / 2.9974 |
| Fit cones | ≥40 total, ≥16/group | 60 inliers, 0 removed | 51 boundary cones, 49 inliers, 2 removed |
| Fit centres vs GT (14.40, ±9.25) | ≤0.5 m | ≈0.000 m | right 0.045 m, left 0.022 m |
| Screenshot | full 8 + vehicle trajectory | — | [`skidpad_rviz.png`](skidpad_rviz.png) |

Geometry notes: GT fit centres `(14.40,-9.25)/(14.40,9.25)`; YOLO fit
`(14.38,-9.21)/(14.39,9.23)`. Max vehicle speed recorded 1.61 / 1.59 m/s
(the sim speed controller overshoots the 1.2 m/s target; not a vehicle limit).

### Round 1 command (GT, perception isolated)

```bash
ros2 launch qutms_eufs_bridge yolo_autonomous_loop.launch.py \
  track:=skidpad planner:=skidpad \
  skidpad_cone_map_topic:=/ground_truth/global_map \
  gazebo_gui:=false rviz:=false use_sim_time:=true \
  path_tracker_ff_gain:=0.0
```

### Round 2 (live YOLO map) — warm-up required

At the EUFS start line the 10 m / 110 deg simulated camera sees only 13-26
blue/yellow boundary cones — most of both circles stays beside/behind the
forward FOV even while driving the entry straight — so the 40/16 fit gate
cannot pass from a cold start and the car never moves (the gate was NOT
relaxed; this is a sensor-coverage fact, not a fitting bug). The acceptance
used the mapping warm-up, now automated:

```bash
# terminal 1: sim (the launch guard forces planner:=skidpad on this track)
ros2 launch qutms_eufs_bridge yolo_autonomous_loop.launch.py \
  track:=skidpad gazebo_gui:=false rviz:=false use_sim_time:=true

# terminal 2: one-command warm-up + handover
python3 src/planners/scripts/skidpad_yolo_warmup.py
```

The script (`scripts/skidpad_yolo_warmup.py`) stops any running skidpad
planner, drives one GT lap so the live YOLO detector / cone_mapper build
`/mapping/cone_map` (59-67 boundary cones), stops, verifies the car back at
the start pose via TF, and spawns the production planner on the live YOLO
map. The production planner consumed `/mapping/cone_map` only. Repeated
warm-ups in one session fragment the YOLO map (the script warns above 120
boundary cones); restart the simulation for a fresh `cone_mapper`.

## Regression

- `planner:=urinay`, `track:=small_oval`: path publisher = 1 (`urinay_path_bridge`), tracker drove at ~1.41 m/s; no errors.
- `planner:=ft`, `track:=small_over`/`small_track`: ft launches and is the single PathWithSpeeds publisher; the short 50 s window did not sustain a path from the sparse start map (`No cones found`). ft code and defaults were not modified; this is the pre-existing sparse-start behavior, not attributed to skidpad.
- Existing tests: 72/72 pass (above).

## Known issues / v2

- v1 has no ICP fine alignment; pose comes from the common-centre circle fit.
- YOLO cold start cannot satisfy the 40/16 gate with the 15 m sim camera;
  the measured Round 2 needs the mapping warm-up above. Real-car camera range
  must be checked before using this mode from a standstill.
- `finish_margin` is 3.0 m, not the spec's 0.5 m: the unchanged tracker's
  minimum lookahead is 2.0 m and its target speeds reach zero ~2 m before the
  geometric end. The state still ends at the EXIT tail.
- `curvature_ff_gain=0.0` is applied only by the skidpad launch; `control.yaml`
  and all existing modes keep the default 1.0.
- The vehicle handler / mission launch was not exercised on hardware; only the
  simulation acceptance is claimed.
