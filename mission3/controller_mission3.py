"""HUNTER Mission 3 - HUNT."""

import heapq
import math


ROBOT_RADIUS = 0.20
CAPTURE_RADIUS = 0.5

# Planning (same idea as Mission 1).
HARD_MARGIN = 0.04
LINE_MARGIN = 0.10
PREFERRED_CLEARANCE = 0.55
WALL_PENALTY = 1.0

# Kalman filter tuning (same as Mission 2).
MEASUREMENT_NOISE = 0.25
ACCELERATION_NOISE = 0.4

# Interception search.
MAX_HORIZON = 5.0
HORIZON_STEP = 0.1
EFFECTIVE_SPEED = 0.85   # fraction of max speed we really average (turns cost time)

REPLAN_INTERVAL = 0.25

# Path following.
WAYPOINT_TOLERANCE = 0.12
DRIVE_ANGLE = math.radians(15.0)
LOOKAHEAD = 0.4

# Corners are driven as circular arcs (same as Mission 1).
CORNER_RADII = (1.0, 0.8, 0.6, 0.45, 0.3, 0.2)
CORNER_MARGIN = 0.06
MAX_ARC_TURN = math.radians(135.0)
ARC_START_ANGLE = math.radians(5.0)


def wrap_angle(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def distance(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def point_rectangle_distance(point, rect):
    """Distance from a point to an axis-aligned rectangle."""
    x, y = point
    xmin, xmax, ymin, ymax = rect

    dx = max(xmin - x, 0.0, x - xmax)
    dy = max(ymin - y, 0.0, y - ymax)

    return math.hypot(dx, dy)


class AxisFilter:
    """Constant-velocity Kalman filter for one axis (position and velocity)."""

    def __init__(self, position):
        self.position = position
        self.velocity = 0.0

        # Covariance matrix [[p11, p12], [p12, p22]].
        self.p11 = MEASUREMENT_NOISE ** 2
        self.p12 = 0.0
        self.p22 = 4.0

    def predict(self, dt):
        q = ACCELERATION_NOISE ** 2

        self.position += self.velocity * dt

        self.p11 += 2.0 * dt * self.p12 + dt * dt * self.p22 + q * dt ** 3 / 3.0
        self.p12 += dt * self.p22 + q * dt ** 2 / 2.0
        self.p22 += q * dt

    def correct(self, measured):
        s = self.p11 + MEASUREMENT_NOISE ** 2

        k1 = self.p11 / s
        k2 = self.p12 / s

        innovation = measured - self.position

        self.position += k1 * innovation
        self.velocity += k2 * innovation

        p11 = self.p11
        p12 = self.p12

        self.p11 = (1.0 - k1) * p11
        self.p12 = (1.0 - k1) * p12
        self.p22 = self.p22 - k2 * p12


class HunterController:
    def __init__(self):
        self.reset(None)

    def reset(self, mission_config):
        self.map_ready = False
        self.robot_radius = ROBOT_RADIUS
        self.capture_radius = CAPTURE_RADIUS

        self.filter_x = None
        self.filter_y = None
        self.last_timestamp = None

        self.estimate = None
        self.intercept = None

        self.path = []
        self.corners = []
        self.waypoint_index = 0
        self.turning = False
        self.last_plan_time = None
        self.clock = 0.0

        if mission_config is None:
            return

        arena = mission_config["arena"]

        self.load_map(
            float(arena["width"]),
            float(arena["height"]),
            float(arena["cell_size"]),
            mission_config.get("grid", []),
        )

        self.robot_radius = float(
            mission_config.get("hunter_radius", ROBOT_RADIUS)
        )

        self.capture_radius = float(
            mission_config.get("capture_radius", CAPTURE_RADIUS)
        )

    # ------------------------------------------------------------------
    # Map (same as Mission 1)
    # ------------------------------------------------------------------

    def load_map(self, width, height, cell, grid):
        self.width = width
        self.height = height
        self.cell = cell

        self.blocked = {
            (r, c)
            for r, row in enumerate(grid)
            for c, value in enumerate(row)
            if value == "#"
        }

        # Fine planning grid with the clearance of every point.
        self.resolution = min(0.15, cell / 5.0)
        self.rows = int(height / self.resolution)
        self.cols = int(width / self.resolution)

        self.node_clearance = [
            [
                self.clearance(self.node_point((i, j)))
                for j in range(self.cols)
            ]
            for i in range(self.rows)
        ]

        self.map_ready = True

    def clearance(self, point):
        """Distance to the nearest obstacle block or arena wall."""
        x, y = point

        best = min(
            x,
            y,
            self.width - x,
            self.height - y,
        )

        if best <= 0.0:
            return 0.0

        # Only blocks within two cells can matter.
        col = int(x // self.cell)
        row = int(y // self.cell)

        for r in range(row - 2, row + 3):
            for c in range(col - 2, col + 3):

                if (r, c) not in self.blocked:
                    continue

                rect = (
                    c * self.cell,
                    (c + 1) * self.cell,
                    r * self.cell,
                    (r + 1) * self.cell,
                )

                best = min(
                    best,
                    point_rectangle_distance(point, rect)
                )

        return best

    def segment_is_safe(self, a, b, min_clearance):
        """Check points every 4 cm along the segment."""
        steps = max(1, int(distance(a, b) / 0.04))

        for k in range(steps + 1):
            t = k / steps

            point = (
                a[0] + t * (b[0] - a[0]),
                a[1] + t * (b[1] - a[1]),
            )

            if self.clearance(point) < min_clearance:
                return False

        return True

    def node_point(self, node):
        return (
            (node[1] + 0.5) * self.resolution,
            (node[0] + 0.5) * self.resolution,
        )

    def point_node(self, point):
        j = max(
            0,
            min(self.cols - 1, int(point[0] / self.resolution))
        )

        i = max(
            0,
            min(self.rows - 1, int(point[1] / self.resolution))
        )

        return i, j

    # ------------------------------------------------------------------
    # Target estimation (same as Mission 2)
    # ------------------------------------------------------------------

    def add_measurement(self, measurement):
        timestamp = measurement["timestamp"]

        # Ignore old or repeated packets.
        if self.last_timestamp is not None and timestamp <= self.last_timestamp:
            return

        if self.filter_x is None:
            self.filter_x = AxisFilter(measurement["x"])
            self.filter_y = AxisFilter(measurement["y"])
            self.last_timestamp = timestamp
            return

        # The filter runs in measurement time, not arrival time.
        dt = timestamp - self.last_timestamp

        self.filter_x.predict(dt)
        self.filter_y.predict(dt)

        self.filter_x.correct(measurement["x"])
        self.filter_y.correct(measurement["y"])

        self.last_timestamp = timestamp

    def predict_position(self, time_ahead):
        """Runner position time_ahead seconds after the last measurement (constant velocity)."""
        return (
            self.filter_x.position + self.filter_x.velocity * time_ahead,
            self.filter_y.position + self.filter_y.velocity * time_ahead,
        )

    # ------------------------------------------------------------------
    # Travel distances from the Hunter
    # ------------------------------------------------------------------

    def build_travel_field(self, start):
        """Dijkstra from the Hunter to every reachable grid point.

        self.travel[node] is the (wall-penalised) path length from the Hunter,
        self.previous[node] lets us walk the path back to the Hunter.
        """
        min_clearance = self.robot_radius + HARD_MARGIN
        start_node = self.point_node(start)

        moves = (
            (-1, 0),
            (1, 0),
            (0, -1),
            (0, 1),
            (-1, -1),
            (-1, 1),
            (1, -1),
            (1, 1),
        )

        travel = {
            start_node: 0.0
        }

        previous = {
            start_node: None
        }

        queue = [
            (
                0.0,
                start_node,
            )
        ]

        while queue:

            current_cost, current = heapq.heappop(queue)

            if current_cost > travel[current] + 1e-9:
                continue

            for dr, dc in moves:

                nxt = (
                    current[0] + dr,
                    current[1] + dc,
                )

                if not (
                    0 <= nxt[0] < self.rows
                    and
                    0 <= nxt[1] < self.cols
                ):
                    continue

                clear = self.node_clearance[nxt[0]][nxt[1]]

                if clear < min_clearance:
                    continue

                # Do not cut diagonally past a forbidden node.
                if dr and dc:

                    if (
                        self.node_clearance[current[0] + dr][current[1]] < min_clearance
                        or
                        self.node_clearance[current[0]][current[1] + dc] < min_clearance
                    ):
                        continue

                # Small extra cost close to walls.
                closeness = max(0.0, PREFERRED_CLEARANCE - clear)
                penalty = WALL_PENALTY * closeness / PREFERRED_CLEARANCE

                step = math.hypot(dr, dc) * self.resolution
                new_cost = current_cost + step * (1.0 + penalty)

                if new_cost < travel.get(
                    nxt,
                    float("inf")
                ):

                    travel[nxt] = new_cost
                    previous[nxt] = current

                    heapq.heappush(
                        queue,
                        (
                            new_cost,
                            nxt,
                        )
                    )

        self.travel = travel
        self.previous = previous

    def best_reachable_node(self, point):
        """Reachable grid node close enough to capture a Runner at point."""
        reach = 0.5 * self.capture_radius
        window = int(reach / self.resolution) + 1
        center = self.point_node(point)

        best = None
        best_cost = float("inf")

        for i in range(center[0] - window, center[0] + window + 1):
            for j in range(center[1] - window, center[1] + window + 1):

                node = (i, j)

                if node not in self.travel:
                    continue

                if distance(self.node_point(node), point) > reach:
                    continue

                if self.travel[node] < best_cost:
                    best = node
                    best_cost = self.travel[node]

        return best

    # ------------------------------------------------------------------
    # Interception
    # ------------------------------------------------------------------

    def runner_can_be_at(self, point):
        """The Runner cannot be inside an obstacle or outside the arena."""
        return self.clearance(point) > 0.05

    def find_intercept(self, hunter, now):
        """Earliest predicted Runner position the Hunter can reach in time.

        Returns (runner point, grid node the Hunter drives to).
        """
        speed = EFFECTIVE_SPEED * hunter["max_speed"]
        age = now - self.last_timestamp

        # The prediction stops at the first wall: the Runner must turn there.
        runner_point = self.estimate
        stopped = not self.runner_can_be_at(runner_point)

        t = 0.0

        while t <= MAX_HORIZON:

            if not stopped:
                point = self.predict_position(age + t)

                if self.runner_can_be_at(point):
                    runner_point = point
                else:
                    stopped = True

            node = self.best_reachable_node(runner_point)

            if node is not None and self.travel[node] / speed <= t:
                return runner_point, node

            t += HORIZON_STEP

        # No intercept within the horizon: head for the furthest prediction
        # we trust. If that is not reachable, get as close as we can.
        node = self.best_reachable_node(runner_point)

        if node is not None:
            return runner_point, node

        return self.estimate, self.closest_reachable_node(self.estimate)

    def closest_reachable_node(self, point):
        """Reachable node nearest to a point we cannot reach (wait there)."""
        best = None
        best_distance = float("inf")

        for node in self.travel:

            d = distance(self.node_point(node), point)

            if d < best_distance:
                best = node
                best_distance = d

        return best

    def plan(self, hunter, now):
        position = (hunter["x"], hunter["y"])

        self.build_travel_field(position)

        self.intercept, goal = self.find_intercept(hunter, now)

        if goal is None:
            # Runner not reachable right now: stay on the current path.
            return

        # Walk back from the goal to the Hunter.
        points = []
        current = goal

        while current is not None:
            points.append(self.node_point(current))
            current = self.previous[current]

        points.reverse()
        points[0] = position

        if self.segment_is_safe(
            points[-1],
            self.intercept,
            self.robot_radius + 0.02
        ):
            points.append(self.intercept)

        if len(points) < 2:
            points.append(self.intercept)

        self.path = self.simplify_path(points)
        self.corners = self.plan_corners(self.path)
        self.waypoint_index = 1

    def farthest_visible(self, points, i, min_clearance):
        j = len(points) - 1

        while j > i + 1:

            if self.segment_is_safe(points[i], points[j], min_clearance):
                break

            j -= 1

        return j

    def simplify_path(self, points):
        """Remove unnecessary points, with a tighter margin in narrow passages."""
        comfortable = self.robot_radius + LINE_MARGIN
        tight = self.robot_radius + HARD_MARGIN

        result = [points[0]]
        i = 0

        while i < len(points) - 1:

            j = self.farthest_visible(points, i, comfortable)

            if j == i + 1:
                j = self.farthest_visible(points, i, tight)

            result.append(points[j])
            i = j

        return result

    def arc_is_safe(self, center, radius, start_angle, turn):
        """Check points every 4 cm along an arc."""
        steps = max(1, int(abs(turn) * radius / 0.04))

        for k in range(steps + 1):
            angle = start_angle + turn * k / steps

            point = (
                center[0] + radius * math.cos(angle),
                center[1] + radius * math.sin(angle),
            )

            if self.clearance(point) < self.robot_radius + CORNER_MARGIN:
                return False

        return True

    def plan_corners(self, path):
        """Choose an arc for every corner of the path (same as Mission 1).

        corners[k] is None (stop and turn in place at waypoint k) or
        (radius, start distance before the waypoint, turn sign, outgoing heading).
        """
        corners = [None] * len(path)
        used_before = 0.0

        for k in range(1, len(path) - 1):
            a, b, c = path[k - 1], path[k], path[k + 1]

            heading_in = math.atan2(b[1] - a[1], b[0] - a[0])
            heading_out = math.atan2(c[1] - b[1], c[0] - b[0])
            turn = wrap_angle(heading_out - heading_in)

            # Arc length available on each side of the corner.
            room_in = distance(a, b) - used_before
            room_out = 0.5 * distance(b, c)
            used_before = 0.0

            if abs(turn) < 1e-3 or abs(turn) > MAX_ARC_TURN:
                continue

            sign = 1.0 if turn > 0.0 else -1.0

            for radius in CORNER_RADII:

                # Distance before (and after) the corner where the arc touches the lines.
                start_distance = radius * math.tan(abs(turn) / 2.0)

                if start_distance > room_in or start_distance > room_out:
                    continue

                arc_start = (
                    b[0] - start_distance * math.cos(heading_in),
                    b[1] - start_distance * math.sin(heading_in),
                )

                # Arc center is to the left (sign +1) or right (sign -1) of travel.
                center = (
                    arc_start[0] - sign * radius * math.sin(heading_in),
                    arc_start[1] + sign * radius * math.cos(heading_in),
                )

                start_angle = math.atan2(
                    arc_start[1] - center[1],
                    arc_start[0] - center[0],
                )

                if self.arc_is_safe(center, radius, start_angle, turn):
                    corners[k] = (
                        radius,
                        start_distance,
                        sign,
                        heading_out,
                    )
                    used_before = start_distance
                    break

        return corners

    # ------------------------------------------------------------------
    # Path following (same as Mission 1)
    # ------------------------------------------------------------------

    def project_on_segment(self, point):
        """Return (distance along, segment length) for the current segment."""
        a = self.path[self.waypoint_index - 1]
        b = self.path[self.waypoint_index]

        length = distance(a, b)

        if length < 1e-9:
            return 0.0, 0.0

        along = (
            (point[0] - a[0]) * (b[0] - a[0])
            + (point[1] - a[1]) * (b[1] - a[1])
        ) / length

        return max(0.0, min(length, along)), length

    def steering_point(self, point):
        """Point a little ahead on the segment, pulls the robot back onto it."""
        a = self.path[self.waypoint_index - 1]
        b = self.path[self.waypoint_index]

        along, length = self.project_on_segment(point)

        if length < 1e-9:
            return b

        ahead = min(length, along + LOOKAHEAD)

        return (
            a[0] + (b[0] - a[0]) * ahead / length,
            a[1] + (b[1] - a[1]) * ahead / length,
        )

    def update(self, sensor_data, dt):
        hunter = sensor_data["hunter"]
        position = (hunter["x"], hunter["y"])

        # Simulation time (own clock as a fallback).
        self.clock += dt
        now = sensor_data.get("time", self.clock)

        # Fallback if reset() did not receive the map.
        if not self.map_ready:
            arena_map = sensor_data["map"]

            self.load_map(
                float(arena_map["width"]),
                float(arena_map["height"]),
                float(arena_map["cell_size"]),
                arena_map["grid"],
            )

        measurement = sensor_data.get("target_measurement")

        if measurement and measurement.get("available"):
            self.add_measurement(measurement)

        # Nothing measured yet: wait.
        if self.filter_x is None:
            return 0.0, 0.0

        # Best guess of where the Runner is right now (delay compensated).
        self.estimate = self.predict_position(now - self.last_timestamp)

        # Re-plan the intercept and the route regularly
        # (but never in the middle of a corner arc).
        if (
            self.last_plan_time is None
            or (now - self.last_plan_time >= REPLAN_INTERVAL and not self.turning)
        ):
            self.plan(hunter, now)
            self.last_plan_time = now
            self.turning = False

        if len(self.path) < 2:
            return 0.0, 0.0

        # Driving around a corner arc.
        if self.turning:
            command = self.follow_arc(hunter, position, dt)

            if command is not None:
                return command

        # Move to the next waypoint once the current one is reached.
        # Corners with an arc are handled by the arc instead.
        while self.waypoint_index < len(self.path) - 1:

            if self.corners[self.waypoint_index] is not None:
                break

            if distance(
                position,
                self.path[self.waypoint_index]
            ) > WAYPOINT_TOLERANCE:
                break

            self.waypoint_index += 1

        waypoint = self.path[self.waypoint_index]
        last_waypoint = self.waypoint_index == len(self.path) - 1

        steer = self.steering_point(position)

        # Near the end of the path, aim straight at the moving target point.
        if last_waypoint and distance(position, waypoint) < LOOKAHEAD:
            steer = waypoint

        desired = math.atan2(
            steer[1] - position[1],
            steer[0] - position[0],
        )

        error = wrap_angle(
            desired - hunter["theta"]
        )

        # Turn as fast as allowed, and exactly reach the heading in the last step.
        omega = max(
            -hunter["max_omega"],
            min(
                hunter["max_omega"],
                error / dt
            )
        )

        # Turn first, then drive. Speed fades to zero near DRIVE_ANGLE.
        if abs(error) < DRIVE_ANGLE:

            ratio = abs(error) / DRIVE_ANGLE
            speed = hunter["max_speed"] * (1.0 - ratio * ratio)

            # Stop at corners of the path (or at the start of their arc),
            # but not at the final point: the Runner keeps moving.
            if not last_waypoint:
                corner = self.corners[self.waypoint_index]
                stop_distance = distance(position, waypoint)

                if corner is not None:
                    stop_distance -= corner[1]

                speed = min(
                    speed,
                    max(0.0, stop_distance) / dt
                )

                # Reached the arc start while facing along the line: begin the arc.
                if (
                    corner is not None
                    and stop_distance < 1e-3
                    and abs(error) < ARC_START_ANGLE
                ):
                    self.turning = True
                    self.waypoint_index += 1

                    command = self.follow_arc(hunter, position, dt)

                    if command is not None:
                        return command

        else:
            speed = 0.0

        # Safety check: never step closer to a wall when already too close.
        next_position = (
            position[0] + speed * math.cos(hunter["theta"]) * dt,
            position[1] + speed * math.sin(hunter["theta"]) * dt,
        )

        next_clearance = self.clearance(next_position)

        if (
            speed > 0.0
            and next_clearance < self.robot_radius + 0.01
            and next_clearance < self.clearance(position)
        ):
            speed = 0.0

        return speed, omega

    def follow_arc(self, hunter, position, dt):
        """Drive along the current corner arc. Returns None when the arc is done."""
        radius, _, sign, heading_out = self.corners[self.waypoint_index - 1]

        remaining = sign * wrap_angle(heading_out - hunter["theta"])

        if remaining < 1e-4:
            self.turning = False
            return None

        # Fastest turn rate for this radius, without passing the final heading.
        turn_rate = min(
            hunter["max_omega"],
            hunter["max_speed"] / radius,
            remaining / dt
        )

        speed = turn_rate * radius
        omega = sign * turn_rate

        next_position = (
            position[0] + speed * math.cos(hunter["theta"]) * dt,
            position[1] + speed * math.sin(hunter["theta"]) * dt,
        )

        # Safety check, same as on straight lines.
        if self.clearance(next_position) < self.robot_radius + 0.01:
            self.turning = False
            return None

        return speed, omega

    def get_debug(self):
        if self.estimate is None:
            return {
                "target_estimate": None,
                "target_prediction": None,
                "planned_path": [],
            }

        return {
            "target_estimate": list(self.estimate),
            "target_prediction": list(self.intercept) if self.intercept else None,
            "planned_path": [
                list(point)
                for point in self.path
            ],
        }