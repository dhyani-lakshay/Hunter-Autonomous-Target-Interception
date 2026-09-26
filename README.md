# HUNTER – Autonomous Target Interception

Our solution for the **HUNTER** challenge (Intra IIT Tech Meet 1.0, Mobile Robotics & Autonomous Systems track).

A Hunter robot has to reach a Runner robot in a simulated arena, across three missions:

| Mission | Task | Runner | Obstacles |
|---|---|---|---|
| 1 – FIND | Reach a target | Stationary | Yes |
| 2 – PREDICT | Intercept a target | Moving | No |
| 3 – HUNT | Intercept a target | Moving | Yes |

## Files

```
controller_mission1.py   # Mission 1 – FIND
controller_mission2.py   # Mission 2 – PREDICT
controller_mission3.py   # Mission 3 – HUNT
```

Each file defines a `HunterController` class with `reset()`, `update()` and `get_debug()`, as described in `CONTROLLER_API.md`. Only plain Python (`math`, `heapq`) is used.

## How to run

1. Put the three controller files in the HUNTER starter-kit folder, next to `HUNTER_ARENA.py`.
2. Install the dependencies once:
   - Windows: `setup_windows.bat`
   - Ubuntu: `./setup_ubuntu.sh`
3. Run a mission:

| Mission | Windows | Ubuntu |
|---|---|---|
| 1 | `run_mission1_A.bat` (also `_B`, `_C`) | `./run_mission1_A.sh` (also `_B`, `_C`) |
| 2 | `run_mission2.bat` | `./run_mission2.sh` |
| 3 | `run_mission3.bat` | `./run_mission3.sh` |

Results are saved in the `results/` folder.

## How it works

### Mission 1 – FIND

1. **Map clearance:** we build a fine grid over the arena and compute each point's distance to the nearest wall.
2. **Path search:** A\* finds a route that keeps the robot a safe distance from walls.
3. **Shortcutting:** the route is shortened by skipping waypoints wherever a straight line is safe.
4. **Driving:** the robot drives the straight parts at full speed and takes corners as smooth circular arcs, choosing the largest arc that stays clear of walls.
5. **Safety check:** before each move, the robot checks that the next step is not too close to a wall.

### Mission 2 – PREDICT

1. **Tracking:** a simple **Kalman filter** (constant velocity, one per axis) smooths the noisy measurements and estimates the Runner's velocity.
2. **Delay compensation:** measurements arrive late, so we move the estimate forward by the delay (`current time − measurement timestamp`).
3. **Interception:** we look ahead in time for the earliest point where the Hunter can arrive before the Runner does, and drive there.
4. **Walls:** the Hunter keeps a safe distance from the arena walls.

### Mission 3 – HUNT

This combines Missions 1 and 2:

1. **Tracking:** the Runner is tracked with the same Kalman filter as Mission 2.
2. **Travel distances:** every 0.25 s, one Dijkstra search from the Hunter gives the **real travel distance around obstacles** to every reachable point.
3. **Interception:** we use these distances to find the earliest reachable intercept point, predicting up to 5 s ahead.
4. **Walls in predictions:** a prediction that runs into a wall stops there, because the Runner must turn.
5. **Unreachable Runner:** if the Runner can't be reached, the Hunter waits at the closest point it can reach.
6. **Driving:** the path comes straight from the same Dijkstra search and is driven like in Mission 1.

## Assumptions

- **Hunter shape:** the Hunter is a circle of radius 0.20 m. This is the simulator's default.
- **Runner speed:** the Runner is slower than the Hunter, as stated in the API.
- **Runner trajectory:** we **do not** use the Runner's trajectory, even though it is present in the scenario config. The Hunter only uses the noisy measurements.
- **Sensor noise:** the noise level is not known in advance, so the filter uses a fixed estimate (0.25 m).

## Results (local simulator)

| Scenario | Status | Collision | Capture time (s) | Path distance (m) | Estimate RMSE (m) | Prediction RMSE (m) |
|---|---|---|---|---|---|---|
| Mission 1 – Maze A | CAPTURED | No | 9.95 | 21.61 | – | – |
| Mission 1 – Maze B | CAPTURED | No | 8.77 | 19.16 | – | – |
| Mission 1 – Maze C | CAPTURED | No | 11.98 | 24.83 | – | – |
| Mission 2 | CAPTURED | No | 10.85 | 14.09 | 0.36 | 6.32 |
| Mission 3 | CAPTURED | No | 17.62 | 20.12 | 0.26 | 2.51 |

**Prediction RMSE:** `target_prediction` is the planned **intercept point**, which is ahead of the Runner on purpose. The local scorer compares it with the Runner's *current* position, so this error is naturally larger than the estimate error.


## Tuning

All important settings are constants at the top of each controller file, such as safety margins, filter noise and prediction horizon. They can be changed without touching the logic.
