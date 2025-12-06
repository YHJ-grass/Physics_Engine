# engine.py
# Soft-body (jelly-like) physics implemented in your rigid engine style.
# All comments are written in English.

import math
import random


# ==========================
# Basic 2D vector
# ==========================

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
        """Return a perpendicular vector (rotated 90 degrees)."""
        return Vec2(-self.y, self.x)

    def __repr__(self) -> str:
        return f"Vec2({self.x}, {self.y})"


# ==========================
# Soft-body vertex
# ==========================

class SoftVertex:
    """Single mass point for a jelly body."""
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
        """Return distance to another vertex."""
        return (self.pos - other.pos).length()


# ==========================
# Helper geometry functions
# ==========================

def polygon_area(vertices: list[SoftVertex]) -> float:
    """Compute polygon area using the shoelace formula."""
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
    """Compute simple geometric center (average of vertex positions)."""
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
    """Return point from 'center' toward 'target' at a fixed distance."""
    delta = target - center
    angle = math.atan2(delta.y, delta.x)
    x = center.x + distance * math.cos(angle)
    y = center.y + distance * math.sin(angle)
    return Vec2(x, y)


# ==========================
# Global physics parameters
# ==========================

DAMPING: float = 0.98       # Velocity damping for both center and vertices
GRAVITY: float = 9.81       # Gravity used on center and vertices (downwards)

RESTITUTION: float = 1.0    # Used for wall and ball-ball collisions
SCALE: float = 100.0        # Scale factor (velocities ¡æ position step)

# Explosion parameters
EXPLOSION_SPEED_THRESHOLD: float = 80.0   # Threshold of wall impact speed to consider explosion
EXPLOSION_CHANCE: float = 0.1             # Probability to explode when above threshold
MIN_EXPLOSION_RADIUS: float = 40.0        # Do not explode very small balls (fragments)

# Merging parameters
MERGE_OVERLAP_RATIO: float = 0.4          # How deep two balls must overlap to consider merging
MERGE_CHANCE: float = 0.1                 # Probability to merge when overlap is deep enough
MERGE_GROWTH_FACTOR: float = 1.05         # Radius growth factor when two balls merge

# Ground friction parameters
GROUND_FRICTION: float = 0.85             # Horizontal friction when touching the ground
SLEEP_THRESHOLD: float = 0.5              # Velocities below this are snapped to zero


# ==========================
# Soft-body circle, but in your "RigidBodyCircle" interface
# ==========================

class RigidBodyCircle:
    """
    Soft-body (jelly) circle implemented with multiple vertices,
    but exposed under the same class name as your rigid engine.
    """

    def __init__(self,
                 position: Vec2,
                 radius: float,
                 mass: float = 1.0,
                 restitution: float = 0.8,
                 shape: str = "circle",
                 resolution: int = 10):
        # Center of mass (used for picking / dragging)
        self.position: Vec2 = Vec2(position.x, position.y)
        self.velocity: Vec2 = Vec2(0.0, 0.0)

        # Physical properties
        self.radius: float = radius
        self.mass: float = float(mass)
        self.inv_mass: float = 0.0 if mass == 0.0 else 1.0 / mass
        self.restitution: float = restitution

        # Shape string is kept for compatibility (only "circle" is supported here)
        self.shape: str = shape

        # Locked flag (used while dragging)
        self.locked: bool = False

        # Force accumulator for rigid-style API (not heavily used here)
        self.force_acc: Vec2 = Vec2(0.0, 0.0)

        # Soft-body data
        self.vertices: list[SoftVertex] = []
        self.original_center: Vec2 = Vec2(self.position.x, self.position.y)
        self.area: float = 0.0
        self.pressure: float = 0.0
        self.impulse: Vec2 = Vec2(0.0, 0.0)

        # Explosion / merging helpers
        self.max_wall_impact_speed: float = 0.0  # max speed when hitting any wall this frame
        self.exploded: bool = False             # true when this body should be visually exploded
        self.merged: bool = False               # true when this body has been merged into another

        # Create vertices on a circle around the center
        self._init_vertices(resolution)

    def _init_vertices(self, resolution: int) -> None:
        """Initialize soft vertices on a circle around the center."""
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

        # Compute rest area and center based on initial configuration
        self.area = polygon_area(self.vertices)
        self.position = polygon_center(self.vertices)
        self.original_center = Vec2(self.position.x, self.position.y)


# ==========================
# External API functions (same names as your rigid engine)
# ==========================

def apply_impulse(body: RigidBodyCircle, impulse: Vec2) -> None:
    """
    Apply an instantaneous impulse to the body.
    For soft-body we change both center velocity and each vertex velocity.
    """
    if body.inv_mass == 0.0:
        return

    dv = impulse * body.inv_mass
    body.velocity = body.velocity + dv

    # Push the same velocity change to all vertices so the shape moves together.
    for v in body.vertices:
        v.vel = v.vel + dv


# ==========================
# Soft-body simulation for a single circle
# ==========================

def _simulate_soft_circle(body: RigidBodyCircle,
                          dt: float,
                          width: int,
                          height: int) -> None:
    """Core soft-body step based on the example engine, adapted to Vec2."""
    # Do not simulate locked, massless, exploded, or merged bodies
    if body.locked or body.inv_mass == 0.0 or body.exploded or body.merged:
        return

    # Walls (ceiling, ground, left, right)
    left_wall = 0.0
    right_wall = float(width)
    ceiling = 0.0
    ground = float(height) - 1.0

    # Reset body-level force, impulse and impact speed
    body.force_acc = Vec2(0.0, 0.0)
    body.impulse = Vec2(0.0, 0.0)
    body.max_wall_impact_speed = 0.0

    bounce_dir = Vec2(0.0, 0.0)
    bouncy_count = 0

    # --- Per-vertex integration ---
    for v in body.vertices:
        # Reset per-vertex force
        v.force = Vec2(0.0, 0.0)

        # Distance from current center and reference distance from original center
        distance = (body.position - v.pos).length()
        rest_distance = (body.original_center - v.original_pos).length()
        if rest_distance == 0.0:
            rest_distance = 1e-6

        # Local pressure based on stretch/compression
        p = 1.0 - (distance / rest_distance)
        v.pressure = p

        # Ideal position if the body kept its rest shape
        offset_from_center = v.original_pos - body.original_center
        ideal_pos = body.position + offset_from_center

        # Pressure direction from center toward ideal position at rest distance
        pressure_dir_vec = point_at_distance_from_center(
            body.position, ideal_pos, rest_distance
        ) - body.position

        # Basic direction from current to ideal position
        direction = ideal_pos - v.pos

        # Combine local and global pressure components
        direction = direction + (
            (pressure_dir_vec * v.pressure) + (pressure_dir_vec * body.pressure)
        ) * 0.5

        # Simple spring-like force
        v.force = v.force + direction * dt

        # Integrate acceleration and velocity
        inv_mass = 1.0 / v.mass if v.mass > 0.0 else 0.0
        acc = v.force * inv_mass
        v.vel = v.vel + acc * dt

        # Damping
        v.vel = v.vel * DAMPING

        # Extra smoothing pulling velocity toward "direction"
        v.vel = v.vel + (direction - v.vel) * 0.2

        # Integrate position
        v.pos = v.pos + (v.vel * SCALE) * dt

        # -------------------------
        # Wall collision handling
        # Track impact speed for explosion logic
        # -------------------------

        # Ground (normal: (0, -1))
        if v.pos.y > ground:
            v.pos.y = ground

            # Apply horizontal ground friction
            v.vel.x *= GROUND_FRICTION
            if abs(v.vel.x) < SLEEP_THRESHOLD:
                v.vel.x = 0.0

            # Only treat this as a bouncing impact if the vertex is actually moving downward
            if ideal_pos.y > ground and v.vel.y > 0.0:
                bounce_offset = ideal_pos - v.pos

                # Use only vertical component for bounce (no horizontal impulse from ground)
                bounce_offset.x = 0.0

                bounce_dir = bounce_dir + bounce_offset

                impact_speed = abs(v.vel.y)
                if impact_speed > body.max_wall_impact_speed:
                    body.max_wall_impact_speed = impact_speed

                v.vel.y = 0.0
                bouncy_count += 1
            else:
                # If the vertex is already resting or moving upward, stop vertical motion
                if v.vel.y > 0.0:
                    v.vel.y = 0.0

        # Ceiling (normal: (0, 1))
        if v.pos.y < ceiling:
            v.pos.y = ceiling

            if ideal_pos.y < ceiling and v.vel.y < 0.0:
                bounce_offset = ideal_pos - v.pos

                # Use only vertical component for ceiling bounce
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

        # Left wall (normal: (1, 0))
        if v.pos.x < left_wall:
            v.pos.x = left_wall

            if ideal_pos.x < left_wall and v.vel.x < 0.0:
                bounce_offset = ideal_pos - v.pos

                # Use only horizontal component for left wall bounce
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

        # Right wall (normal: (-1, 0))
        if v.pos.x > right_wall:
            v.pos.x = right_wall

            if ideal_pos.x > right_wall and v.vel.x > 0.0:
                bounce_offset = ideal_pos - v.pos

                # Use only horizontal component for right wall bounce
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

    # --- Explosion check (big wall hit) ---
    if body.radius >= MIN_EXPLOSION_RADIUS:
        if body.max_wall_impact_speed > EXPLOSION_SPEED_THRESHOLD:
            # Explode with some probability when impact speed is high enough
            if random.random() < EXPLOSION_CHANCE:
                body.exploded = True
                return  # Stop further processing for this body

    # --- Body-level impulse from bouncing vertices ---
    if bouncy_count > 0:
        avg_bounce = bounce_dir * (1.0 / bouncy_count)
        body.impulse = body.impulse + avg_bounce * RESTITUTION

    # Apply impulse to center velocity
    impulse_dt = body.impulse * dt
    if body.mass > 0.0:
        change_v = impulse_dt * (1.0 / body.mass)
        body.velocity = body.velocity - change_v

    # Apply gravity to center
    body.velocity.y += GRAVITY * dt

    # Center damping and integration
    body.velocity = body.velocity * DAMPING
    body.position = body.position + (body.velocity * SCALE) * dt

    # Push impulse and gravity back to each vertex (so they follow the center)
    n_vertices = len(body.vertices)
    if n_vertices > 0:
        per_vertex_impulse = body.impulse * (1.0 / n_vertices) * dt
        for v in body.vertices:
            v.vel = v.vel + per_vertex_impulse
            v.vel.y += GRAVITY * dt

    # Update global pressure based on area shrink/stretch
    current_area = polygon_area(body.vertices)
    if body.area > 0.0:
        body.pressure = max(0.0, 1.0 - current_area / body.area)
    else:
        body.pressure = 0.0


# ==========================
# Soft body vs soft body (vertex-based) collision
# ==========================

def _resolve_soft_collision(b1: RigidBodyCircle,
                            b2: RigidBodyCircle) -> None:
    """
    Resolve soft collision between two jelly bodies.
    Each vertex of one body collides against the other body approximated
    as a circle (center + radius).
    """

    r1 = b1.radius
    r2 = b2.radius

    def handle_vertices(a: RigidBodyCircle, b: RigidBodyCircle, b_radius: float) -> None:
        for v in a.vertices:
            # Vector from b's center to vertex
            delta = v.pos - b.position
            dist = delta.length()

            if dist == 0.0:
                n = Vec2(1.0, 0.0)
                dist_eps = 1e-6
            else:
                n = delta * (1.0 / dist)
                dist_eps = dist

            # Outside the circle ¡æ no collision
            if dist_eps >= b_radius:
                continue

            # Positional correction
            penetration = b_radius - dist_eps
            v.pos = v.pos + n * penetration

            # Relative velocity along normal
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

    # Symmetric handling
    handle_vertices(b1, b2, r2)
    handle_vertices(b2, b1, r1)


# ==========================
# Merging logic
# ==========================

def _maybe_merge_bodies(b1: RigidBodyCircle,
                        b2: RigidBodyCircle) -> RigidBodyCircle | None:
    """
    Try to merge two jelly bodies into a single larger one.
    Returns a new merged body if merge happens, otherwise None.
    """
    # Do not merge bodies that are already exploded or merged
    if b1.exploded or b2.exploded or b1.merged or b2.merged:
        return None

    # Distance between centers
    delta = b2.position - b1.position
    dist = delta.length()
    if dist == 0.0:
        dist = 1e-6

    # Check strong overlap
    if dist > (b1.radius + b2.radius) * MERGE_OVERLAP_RATIO:
        return None

    # Random chance to actually merge
    if random.random() >= MERGE_CHANCE:
        return None

    # Compute merged properties
    total_mass = b1.mass + b2.mass
    if total_mass <= 0.0:
        total_mass = 1.0

    w1 = b1.mass / total_mass
    w2 = b2.mass / total_mass

    # Mass-weighted center and velocity
    new_pos = (b1.position * w1) + (b2.position * w2)
    new_vel = (b1.velocity * w1) + (b2.velocity * w2)

    # New radius: combine area-like contribution and grow a bit
    base_radius = math.sqrt(b1.radius * b1.radius + b2.radius * b2.radius)
    new_radius = base_radius * MERGE_GROWTH_FACTOR

    # Resolution: keep more detailed one
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

    # Mark originals as merged
    b1.merged = True
    b2.merged = True

    return merged


# ==========================
# Global step function (same signature as your rigid engine)
# ==========================

def step(bodies: list[RigidBodyCircle],
         dt: float,
         width: int,
         height: int) -> None:
    """
    Run a full physics step:
    1) Simulate each soft circle (gravity, walls, internal pressure, explosion check).
    2) Resolve soft body-body collisions.
    3) Merge heavily overlapping bodies into one larger body.
    Note: exploded bodies are not removed here; main loop will handle them.
    """
    # 1) Per-body simulation
    for body in bodies:
        _simulate_soft_circle(body, dt, width, height)

    # 2) Pairwise collisions and 3) merging
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

            # Resolve soft collision (velocity/position correction)
            _resolve_soft_collision(b1, b2)

            # Try merging if they are deeply overlapping
            merged = _maybe_merge_bodies(b1, b2)
            if merged is not None:
                merged_bodies.append(merged)

    # Remove merged-into bodies and add new merged ones
    if merged_bodies or any(b.merged for b in bodies):
        remaining = [b for b in bodies if not b.merged]
        remaining.extend(merged_bodies)
        bodies[:] = remaining
