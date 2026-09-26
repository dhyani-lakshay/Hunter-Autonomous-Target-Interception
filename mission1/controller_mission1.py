"""HUNTER Mission 1 - FIND."""

import heapq
import math


ROBOT_RADIUS = 0.20
CAPTURE_RADIUS = 0.5

HARD_MARGIN = 0.04
LINE_MARGIN = 0.10
PREFERRED_CLEARANCE = 0.55
WALL_PENALTY = 1.0

WAYPOINT_TOLERANCE = 0.12
DRIVE_ANGLE = math.radians(15.0)
LOOKAHEAD = 0.4

# Corners are driven as circular arcs. The largest safe radius is used.
CORNER_RADII = (1.0, 0.8, 0.6, 0.45, 0.3, 0.2)
CORNER_MARGIN = 0.06
MAX_ARC_TURN = math.radians(135.0)
ARC_START_ANGLE = math.radians(5.0)

REPLAN_DISTANCE = 0.4
BLOCKED_REPLAN_TIME = 0.5


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


class HunterController:
    def __init__(self):
        self.map_ready = False
        self.path = []
        self.corners = []
        self.waypoint_index = 0
        self.blocked_time = 0.0
        self.turning = False

    def reset(self, mission_config):
        self.map_ready = False
        self.path = []
        self.corners = []
        self.waypoint_index = 0
        self.blocked_time = 0.0
        self.turning = False

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

        start = tuple(map(float, mission_config["hunter"]["start"]))
        self.target = tuple(map(float, mission_config["target"]["position"]))

        self.plan_path(start)

    def load_map(self, width, height, cell, grid):
        self.width = width
        self.height = height
        self.cell = cell
        self.robot_radius = ROBOT_RADIUS
        self.capture_radius = CAPTURE_RADIUS

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

    def astar(self, start, min_clearance):
        """A* to any node near the target, or None if unreachable."""
        goal_tolerance = 0.6 * self.capture_radius
        start_node = self.point_node(start)

        def heuristic(node):
            return max(
                0.0,
                distance(self.node_point(node), self.target) - goal_tolerance
            )

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

        cost = {
            start_node: 0.0
        }

        previous = {
            start_node: None
        }

        queue = [
            (
                heuristic(start_node),
                start_node,
            )
        ]

        goal = None

        while queue:

            _, current = heapq.heappop(queue)

            if distance(self.node_point(current), self.target) <= goal_tolerance:
                goal = current
                break

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
                new_cost = cost[current] + step * (1.0 + penalty)

                if new_cost < cost.get(
                    nxt,
                    float("inf")
                ):

                    cost[nxt] = new_cost
                    previous[nxt] = current

                    heapq.heappush(
                        queue,
                        (
                            new_cost + heuristic(nxt),
                            nxt,
                        )
                    )

        if goal is None:
            return None

        points = []
        current = goal

        while current is not None:
            points.append(self.node_point(current))
            current = previous[current]

        points.reverse()

        return points

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

    def plan_path(self, start):
        points = self.astar(start, self.robot_radius + HARD_MARGIN)

        if points is None:
            points = self.astar(start, self.robot_radius + 0.005)

        if points is None:
            # No route found. The safety check still prevents collisions.
            self.path = [start, self.target]
            self.corners = [None, None]
            self.waypoint_index = 1
            self.turning = False
            return

        points[0] = start

        if self.segment_is_safe(
            points[-1],
            self.target,
            self.robot_radius + 0.02
        ):
            points.append(self.target)

        self.path = self.simplify_path(points)
        self.corners = self.plan_corners(self.path)
        self.waypoint_index = 1
        self.blocked_time = 0.0
        self.turning = False

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
        """Choose an arc for every corner of the path.

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

    def project_on_segment(self, point):
        """Return (distance along, segment length, distance off) for the current segment."""
        a = self.path[self.waypoint_index - 1]
        b = self.path[self.waypoint_index]

        length = distance(a, b)

        if length < 1e-9:
            return 0.0, 0.0, distance(point, a)

        ux = (b[0] - a[0]) / length
        uy = (b[1] - a[1]) / length

        along = (point[0] - a[0]) * ux + (point[1] - a[1]) * uy
        along = max(0.0, min(length, along))

        closest = (
            a[0] + along * ux,
            a[1] + along * uy,
        )

        return along, length, distance(point, closest)

    def steering_point(self, point):
        """Point a little ahead on the segment, pulls the robot back onto it."""
        a = self.path[self.waypoint_index - 1]
        b = self.path[self.waypoint_index]

        along, length, _ = self.project_on_segment(point)

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

        # Fallback if reset() did not receive the map.
        if not self.map_ready:
            arena_map = sensor_data["map"]

            self.load_map(
                float(arena_map["width"]),
                float(arena_map["height"]),
                float(arena_map["cell_size"]),
                arena_map["grid"],
            )

            self.target = (
                sensor_data["target"]["x"],
                sensor_data["target"]["y"],
            )

            self.plan_path(position)

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

        if self.project_on_segment(position)[2] > REPLAN_DISTANCE:
            self.plan_path(position)

        waypoint = self.path[self.waypoint_index]
        steer = self.steering_point(position)

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

            # Stop exactly at the waypoint, or at the start of its arc.
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
            self.blocked_time += dt

        else:
            self.blocked_time = 0.0

        if self.blocked_time > BLOCKED_REPLAN_TIME:
            self.plan_path(position)

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
        return {
            "planned_path": [
                list(point)
                for point in self.path
            ]
        }