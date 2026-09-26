"""HUNTER Mission 2 - PREDICT."""

import math


ROBOT_RADIUS = 0.20
WALL_MARGIN = 0.40

# Kalman filter tuning (the true noise level is not given to the Hunter).
MEASUREMENT_NOISE = 0.25
ACCELERATION_NOISE = 0.4

# Interception search.
MAX_HORIZON = 15.0
HORIZON_STEP = 0.05

TURN_GAIN = 3.0


def wrap_angle(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def distance(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


class AxisFilter:
    """Constant-velocity Kalman filter for one axis (position and velocity).

    x and y are independent, so one small filter per axis is enough.
    """

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
        self.filter_x = None
        self.filter_y = None
        self.last_timestamp = None

        self.estimate = None
        self.intercept = None
        self.clock = 0.0

        # Arena size is only needed to stay away from the walls.
        self.width = None
        self.height = None
        self.robot_radius = ROBOT_RADIUS
        self.capture_radius = 0.5

        if mission_config is None:
            return

        arena = mission_config["arena"]

        self.width = float(arena["width"])
        self.height = float(arena["height"])

        self.robot_radius = float(
            mission_config.get("hunter_radius", ROBOT_RADIUS)
        )

        self.capture_radius = float(
            mission_config.get("capture_radius", 0.5)
        )

    # ------------------------------------------------------------------
    # Estimation
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
        """Runner position time_ahead seconds after the last measurement.

        Constant velocity: over a few seconds this is more reliable than
        trying to extrapolate the Runner's curves.
        """
        return (
            self.filter_x.position + self.filter_x.velocity * time_ahead,
            self.filter_y.position + self.filter_y.velocity * time_ahead,
        )

    def inside_arena(self, point):
        """Move a point inside the arena, away from the walls."""
        if self.width is None:
            return point

        return (
            max(WALL_MARGIN, min(self.width - WALL_MARGIN, point[0])),
            max(WALL_MARGIN, min(self.height - WALL_MARGIN, point[1])),
        )

    # ------------------------------------------------------------------
    # Interception
    # ------------------------------------------------------------------

    def find_intercept(self, hunter, now):
        """Earliest future point the Hunter can reach before the Runner gets there."""
        position = (hunter["x"], hunter["y"])

        # Time from the last measurement to now (this includes the sensor delay).
        age = now - self.last_timestamp

        t = 0.0
        point = self.estimate

        while t <= MAX_HORIZON:

            point = self.inside_arena(
                self.predict_position(age + t)
            )

            direction = math.atan2(
                point[1] - position[1],
                point[0] - position[0],
            )

            turn_time = abs(wrap_angle(direction - hunter["theta"])) / hunter["max_omega"]

            # We only need to get within the capture radius.
            gap = max(0.0, distance(position, point) - 0.5 * self.capture_radius)

            if gap / hunter["max_speed"] + turn_time <= t:
                return point

            t += HORIZON_STEP

        return point

    # ------------------------------------------------------------------
    # Control
    # ------------------------------------------------------------------

    def wall_clearance(self, point):
        if self.width is None:
            return float("inf")

        return min(
            point[0],
            point[1],
            self.width - point[0],
            self.height - point[1],
        )

    def update(self, sensor_data, dt):
        hunter = sensor_data["hunter"]
        position = (hunter["x"], hunter["y"])

        # Simulation time (own clock as a fallback).
        self.clock += dt
        now = sensor_data.get("time", self.clock)

        measurement = sensor_data.get("target_measurement")

        if measurement and measurement.get("available"):
            self.add_measurement(measurement)

        # Nothing measured yet: wait.
        if self.filter_x is None:
            return 0.0, 0.0

        # Best guess of where the Runner is right now (delay compensated).
        self.estimate = self.predict_position(now - self.last_timestamp)

        self.intercept = self.find_intercept(hunter, now)

        desired = math.atan2(
            self.intercept[1] - position[1],
            self.intercept[0] - position[0],
        )

        error = wrap_angle(
            desired - hunter["theta"]
        )

        omega = max(
            -hunter["max_omega"],
            min(
                hunter["max_omega"],
                TURN_GAIN * error
            )
        )

        # Full speed when facing the intercept point, slower while turning.
        speed = hunter["max_speed"] * max(0.0, min(1.0, 1.5 * math.cos(error)))

        # Safety check: never step closer to a wall when already too close.
        next_position = (
            position[0] + speed * math.cos(hunter["theta"]) * dt,
            position[1] + speed * math.sin(hunter["theta"]) * dt,
        )

        next_clearance = self.wall_clearance(next_position)

        if (
            next_clearance < self.robot_radius + 0.02
            and next_clearance < self.wall_clearance(position)
        ):
            speed = 0.0

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
            "target_prediction": list(self.intercept),
            "planned_path": [],
        }