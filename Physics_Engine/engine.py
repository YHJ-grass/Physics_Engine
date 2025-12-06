# engine.py

import math
import random

class Vec2:
    def __init__(self, x: float = 0.0, y: float = 0.0):
        self.x = float(x)
        self.y = float(y)

    def __add__(self, other: "Vec2") -> "Vec2":
        return Vec2(self.x + other.x, self.y + other.y)

    def __sub__(self, other: "Vec2") -> "Vec2":
        return Vec2(self.x - other.x, self.y - other.y)

    def __mul__(self, scalar: float) -> "Vec2":
        return Vec2(self.x * scalar, self.y * scalar)

    __rmul__ = __mul__

    def __truediv__(self, scalar: float) -> "Vec2":
        return Vec2(self.x / scalar, self.y / scalar)

    def dot(self, other: "Vec2") -> float:
        return self.x * other.x + self.y * other.y

    def length(self) -> float:
        return math.sqrt(self.x * self.x + self.y * self.y)

    def normalized(self) -> "Vec2":
        l = self.length()
        if l == 0.0:
            return Vec2(0.0, 0.0)
        return Vec2(self.x / l, self.y / l)

    def perp(self) -> "Vec2":
        return Vec2(-self.y, self.x)

    def __repr__(self) -> str:
        return f"Vec2({self.x}, {self.y})"

class SoftVertex:
    def __init__(self, pos: Vec2, mass: float):
        # Current position
        self.pos: Vec2 = Vec2(pos.x, pos.y)
        # Rest (original) position
        self.original_pos: Vec2 = Vec2(pos.x, pos.y)
        # Velocity and force
        self.vel: Vec2 = Vec2(0.0, 0.0)
        self.force: Vec2 = Vec2(0.0, 0.0)
        # Local pressure scalar
        self.pressure: float = 0.0
        # Mass
        self.mass: float = mass

    def distance_to(self, other: "SoftVertex") -> float:
        return (self.pos - other.pos).length()

def polygon_area(vertices: list[SoftVertex]) -> float:
    n = len(vertices)
    if n < 3:
        return 0.0

    area = 0.0
    for i in range(n):
        x1, y1 = vertices[i].pos.x, vertices[i].pos.y
        x2, y2 = vertices[(i + 1) % n].pos.x, vertices[(i + 1) % n].pos.y
        area += (x1 * y2) - (x2 * y1)
    return abs(area) * 0.5


def polygon_center(vertices: list[SoftVertex]) -> Vec2:
    n = len(vertices)
    if n == 0:
        return Vec2(0.0, 0.0)
    sx = 0.0
    sy = 0.0
    for v in vertices:
        sx += v.pos.x
        sy += v.pos.y
    inv = 1.0 / n
    return Vec2(sx * inv, sy * inv)


def point_at_distance_from_center(center: Vec2, target: Vec2, distance: float) -> Vec2:
    delta = target - center
    angle = math.atan2(delta.y, delta.x)
    x = center.x + distance * math.cos(angle)
    y = center.y + distance * math.sin(angle)
    return Vec2(x, y)


DAMPING: float = 0.98
GRAVITY: float = 9.81

RESTITUTION: float = 1.0
SCALE: float = 100.0 

EXPLOSION_SPEED_THRESHOLD: float = 80.0
EXPLOSION_CHANCE: float = 0.1
MIN_EXPLOSION_RADIUS: float = 40.0

MERGE_OVERLAP_RATIO: float = 0.4
MERGE_CHANCE: float = 0.1
MERGE_GROWTH_FACTOR: float = 1.05 

GROUND_FRICTION: float = 0.85
SLEEP_THRESHOLD: float = 0.5

class RigidBodyCircle:
    def __init__(self,
                 position: Vec2,
                 radius: float,
                 mass: float = 1.0,
                 restitution: float = 0.8,
                 shape: str = "circle",
                 resolution: int = 10):

        self.position: Vec2 = Vec2(position.x, position.y)
        self.velocity: Vec2 = Vec2(0.0, 0.0)

        self.radius: float = radius
        self.mass: float = float(mass)
        self.inv_mass: float = 0.0 if mass == 0.0 else 1.0 / mass
        self.restitution: float = restitution

        self.shape: str = shape

        self.locked: bool = False

        self.force_acc: Vec2 = Vec2(0.0, 0.0)

        self.vertices: list[SoftVertex] = []
        self.original_center: Vec2 = Vec2(self.position.x, self.position.y)
        self.area: float = 0.0
        self.pressure: float = 0.0
        self.impulse: Vec2 = Vec2(0.0, 0.0)

        self.max_wall_impact_speed: float = 0.0
        self.exploded: bool = False
        self.merged: bool = False

        self._init_vertices(resolution)

    def _init_vertices(self, resolution: int) -> None:
        if resolution < 3:
            resolution = 3

        vertex_mass = self.mass / float(resolution) if self.mass > 0.0 else 1.0
        self.vertices.clear()

        cx, cy = self.position.x, self.position.y
        r = self.radius

        for i in range(resolution):
            ang = i * 2.0 * math.pi / resolution - math.pi / 2.0
            x = cx + r * math.cos(ang)
            y = cy - r * math.sin(ang)
            v = SoftVertex(Vec2(x, y), vertex_mass)
            self.vertices.append(v)

        self.area = polygon_area(self.vertices)
        self.position = polygon_center(self.vertices)
        self.original_center = Vec2(self.position.x, self.position.y)

def apply_impulse(body: RigidBodyCircle, impulse: Vec2) -> None:
    if body.inv_mass == 0.0:
        return

    dv = impulse * body.inv_mass
    body.velocity = body.velocity + dv

    for v in body.vertices:
        v.vel = v.vel + dv

def _simulate_soft_circle(body: RigidBodyCircle,
                          dt: float,
                          width: int,
                          height: int) -> None:

    if body.locked or body.inv_mass == 0.0 or body.exploded or body.merged:
        return

    left_wall = 0.0
    right_wall = float(width)
    ceiling = 0.0
    ground = float(height) - 1.0

    body.force_acc = Vec2(0.0, 0.0)
    body.impulse = Vec2(0.0, 0.0)
    body.max_wall_impact_speed = 0.0

    bounce_dir = Vec2(0.0, 0.0)
    bouncy_count = 0

    for v in body.vertices:
        v.force = Vec2(0.0, 0.0)

        distance = (body.position - v.pos).length()
        rest_distance = (body.original_center - v.original_pos).length()
        if rest_distance == 0.0:
            rest_distance = 1e-6

        p = 1.0 - (distance / rest_distance)
        v.pressure = p

        offset_from_center = v.original_pos - body.original_center
        ideal_pos = body.position + offset_from_center

        pressure_dir_vec = point_at_distance_from_center(
            body.position, ideal_pos, rest_distance
        ) - body.position

        direction = ideal_pos - v.pos

        direction = direction + (
            (pressure_dir_vec * v.pressure) + (pressure_dir_vec * body.pressure)
        ) * 0.5

        v.force = v.force + direction * dt

        inv_mass = 1.0 / v.mass if v.mass > 0.0 else 0.0
        acc = v.force * inv_mass
        v.vel = v.vel + acc * dt

        v.vel = v.vel * DAMPING

        v.vel = v.vel + (direction - v.vel) * 0.2

        v.pos = v.pos + (v.vel * SCALE) * dt

        if v.pos.y > ground:
            v.pos.y = ground

            v.vel.x *= GROUND_FRICTION
            if abs(v.vel.x) < SLEEP_THRESHOLD:
                v.vel.x = 0.0

            if ideal_pos.y > ground and v.vel.y > 0.0:
                bounce_offset = ideal_pos - v.pos

                bounce_offset.x = 0.0

                bounce_dir = bounce_dir + bounce_offset

                impact_speed = abs(v.vel.y)
                if impact_speed > body.max_wall_impact_speed:
                    body.max_wall_impact_speed = impact_speed

                v.vel.y = 0.0
                bouncy_count += 1
            else:
                if v.vel.y > 0.0:
                    v.vel.y = 0.0

        if v.pos.y < ceiling:
            v.pos.y = ceiling

            if ideal_pos.y < ceiling and v.vel.y < 0.0:
                bounce_offset = ideal_pos - v.pos

                bounce_offset.x = 0.0

                bounce_dir = bounce_dir + bounce_offset

                impact_speed = abs(v.vel.y)
                if impact_speed > body.max_wall_impact_speed:
                    body.max_wall_impact_speed = impact_speed

                v.vel.y = 0.0
                bouncy_count += 1
            else:
                if v.vel.y < 0.0:
                    v.vel.y = 0.0

        if v.pos.x < left_wall:
            v.pos.x = left_wall

            if ideal_pos.x < left_wall and v.vel.x < 0.0:
                bounce_offset = ideal_pos - v.pos

                bounce_offset.y = 0.0

                bounce_dir = bounce_dir + bounce_offset

                impact_speed = abs(v.vel.x)
                if impact_speed > body.max_wall_impact_speed:
                    body.max_wall_impact_speed = impact_speed

                v.vel.x = 0.0
                bouncy_count += 1
            else:
                if v.vel.x < 0.0:
                    v.vel.x = 0.0

        if v.pos.x > right_wall:
            v.pos.x = right_wall

            if ideal_pos.x > right_wall and v.vel.x > 0.0:
                bounce_offset = ideal_pos - v.pos

                bounce_offset.y = 0.0

                bounce_dir = bounce_dir + bounce_offset

                impact_speed = abs(v.vel.x)
                if impact_speed > body.max_wall_impact_speed:
                    body.max_wall_impact_speed = impact_speed

                v.vel.x = 0.0
                bouncy_count += 1
            else:
                if v.vel.x > 0.0:
                    v.vel.x = 0.0

    if body.radius >= MIN_EXPLOSION_RADIUS:
        if body.max_wall_impact_speed > EXPLOSION_SPEED_THRESHOLD:
            if random.random() < EXPLOSION_CHANCE:
                body.exploded = True
                return 

    if bouncy_count > 0:
        avg_bounce = bounce_dir * (1.0 / bouncy_count)
        body.impulse = body.impulse + avg_bounce * RESTITUTION

    impulse_dt = body.impulse * dt
    if body.mass > 0.0:
        change_v = impulse_dt * (1.0 / body.mass)
        body.velocity = body.velocity - change_v

    body.velocity.y += GRAVITY * dt

    body.velocity = body.velocity * DAMPING
    body.position = body.position + (body.velocity * SCALE) * dt

    n_vertices = len(body.vertices)
    if n_vertices > 0:
        per_vertex_impulse = body.impulse * (1.0 / n_vertices) * dt
        for v in body.vertices:
            v.vel = v.vel + per_vertex_impulse
            v.vel.y += GRAVITY * dt

    current_area = polygon_area(body.vertices)
    if body.area > 0.0:
        body.pressure = max(0.0, 1.0 - current_area / body.area)
    else:
        body.pressure = 0.0

def _resolve_soft_collision(b1: RigidBodyCircle,
                            b2: RigidBodyCircle) -> None:

    r1 = b1.radius
    r2 = b2.radius

    def handle_vertices(a: RigidBodyCircle, b: RigidBodyCircle, b_radius: float) -> None:
        for v in a.vertices:
            delta = v.pos - b.position
            dist = delta.length()

            if dist == 0.0:
                n = Vec2(1.0, 0.0)
                dist_eps = 1e-6
            else:
                n = delta * (1.0 / dist)
                dist_eps = dist

            if dist_eps >= b_radius:
                continue

            penetration = b_radius - dist_eps
            v.pos = v.pos + n * penetration

            rel_vel = v.vel - b.velocity
            vel_along_normal = rel_vel.dot(n)
            if vel_along_normal > 0.0:
                continue

            inv_mass_v = 0.0 if v.mass == 0.0 else 1.0 / v.mass
            inv_mass_b = b.inv_mass
            denom = inv_mass_v + inv_mass_b
            if denom == 0.0:
                continue

            e = RESTITUTION
            j = -(1.0 + e) * vel_along_normal / denom
            impulse = n * j

            v.vel = v.vel + impulse * inv_mass_v
            b.velocity = b.velocity - impulse * inv_mass_b

    handle_vertices(b1, b2, r2)
    handle_vertices(b2, b1, r1)

def _maybe_merge_bodies(b1: RigidBodyCircle,
                        b2: RigidBodyCircle) -> RigidBodyCircle | None:

    if b1.exploded or b2.exploded or b1.merged or b2.merged:
        return None

    delta = b2.position - b1.position
    dist = delta.length()
    if dist == 0.0:
        dist = 1e-6

    if dist > (b1.radius + b2.radius) * MERGE_OVERLAP_RATIO:
        return None

    if random.random() >= MERGE_CHANCE:
        return None

    total_mass = b1.mass + b2.mass
    if total_mass <= 0.0:
        total_mass = 1.0

    w1 = b1.mass / total_mass
    w2 = b2.mass / total_mass

    new_pos = (b1.position * w1) + (b2.position * w2)
    new_vel = (b1.velocity * w1) + (b2.velocity * w2)

    base_radius = math.sqrt(b1.radius * b1.radius + b2.radius * b2.radius)
    new_radius = base_radius * MERGE_GROWTH_FACTOR

    new_resolution = max(len(b1.vertices), len(b2.vertices))

    merged = RigidBodyCircle(
        position=new_pos,
        radius=new_radius,
        mass=total_mass,
        restitution=(b1.restitution + b2.restitution) * 0.5,
        shape="circle",
        resolution=new_resolution,
    )
    merged.velocity = new_vel

    b1.merged = True
    b2.merged = True

    return merged

def step(bodies: list[RigidBodyCircle],
         dt: float,
         width: int,
         height: int) -> None:

    for body in bodies:
        _simulate_soft_circle(body, dt, width, height)

    n = len(bodies)
    merged_bodies: list[RigidBodyCircle] = []

    for i in range(n):
        for j in range(i + 1, n):
            if i >= len(bodies) or j >= len(bodies):
                continue

            b1 = bodies[i]
            b2 = bodies[j]

            if b1.merged or b2.merged or b1.exploded or b2.exploded:
                continue

            _resolve_soft_collision(b1, b2)

            merged = _maybe_merge_bodies(b1, b2)
            if merged is not None:
                merged_bodies.append(merged)

    if merged_bodies or any(b.merged for b in bodies):
        remaining = [b for b in bodies if not b.merged]
        remaining.extend(merged_bodies)
        bodies[:] = remaining
