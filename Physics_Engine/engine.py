# engine.py
import math


# ------------------------------
# 2D vector
# ------------------------------
class Vec2:
    def __init__(self, x: float = 0.0, y: float = 0.0):
        self.x = x
        self.y = y

    def __add__(self, other: "Vec2") -> "Vec2":
        return Vec2(self.x + other.x, self.y + other.y)

    def __sub__(self, other: "Vec2") -> "Vec2":
        return Vec2(self.x - other.x, self.y - other.y)

    def __mul__(self, s: float) -> "Vec2":
        return Vec2(self.x * s, self.y * s)

    def __truediv__(self, s: float) -> "Vec2":
        return Vec2(self.x / s, self.y / s)

    def dot(self, other: "Vec2") -> float:
        return self.x * other.x + self.y * other.y

    def length(self) -> float:
        return math.sqrt(self.x * self.x + self.y * self.y)

    def normalized(self) -> "Vec2":
        L = self.length()
        if L == 0.0:
            return Vec2(0.0, 0.0)
        return Vec2(self.x / L, self.y / L)


# ------------------------------
# Particle (point mass)
# ------------------------------
class Particle:
    def __init__(self, pos: Vec2, radius: float = 8.0, mass: float = 1.0):
        # position, velocity, accumulated force
        self.pos = Vec2(pos.x, pos.y)
        self.vel = Vec2(0.0, 0.0)
        self.force = Vec2(0.0, 0.0)

        # physical properties
        self.radius = radius
        self.mass = mass


# ------------------------------
# Spring between two particles
# ------------------------------
class Spring:
    def __init__(
        self,
        i: int,
        j: int,
        rest_length: float,
        k: float = 500.0,
        damping: float = 2.0,
    ):
        # indices of the two particles in SoftBody.particles
        self.i = i
        self.j = j

        # rest length of the spring
        self.rest_length = rest_length

        # spring stiffness (Hooke's law)
        self.k = k

        # damping along the spring direction
        self.damping = damping


# ------------------------------
# Soft body (jelly-like object)
# center particle + outer ring + springs
# ------------------------------
class SoftBody:
    def __init__(
        self,
        center: Vec2,
        radius: float = 60.0,
        num_points: int = 20,
        # material parameters (can be customized per instance)
        k_struct: float = 1200.0,
        k_bend: float = 600.0,
        k_center: float = 1500.0,
        spring_damping: float = 4.0,
        velocity_damping: float = 0.995,
        wall_restitution: float = 0.25,
        collision_restitution: float = 0.2,
        pressure_strength: float = 2000.0,
    ):
        self.particles: list[Particle] = []
        self.springs: list[Spring] = []

        # indices and parameters for ring structure
        self.center_index: int = 0
        self.ring_start: int = 1
        self.num_points: int = num_points
        self.rest_radii: list[float] = []

        # base core radius (minimum); actual radius will be computed every frame
        self.core_radius: float = radius * 0.6

        # smaller collision radii for smoother contact
        center_collision_radius = 6.0
        outer_collision_radius = 4.0

        # store material parameters
        self.k_struct = k_struct
        self.k_bend = k_bend
        self.k_center = k_center
        self.spring_damping = spring_damping
        self.velocity_damping = velocity_damping
        self.wall_restitution = wall_restitution
        self.collision_restitution = collision_restitution
        self.pressure_strength = pressure_strength

        # 0) center particle (heavier, acts like a core)
        self.particles.append(
            Particle(
                Vec2(center.x, center.y),
                radius=center_collision_radius,
                mass=2.0,
            )
        )

        # 1) outer ring particles
        for n in range(num_points):
            angle = 2.0 * math.pi * n / num_points
            x = center.x + radius * math.cos(angle)
            y = center.y + radius * math.sin(angle)

            # store rest radial distance from center
            rest_r = (Vec2(x, y) - center).length()
            self.rest_radii.append(rest_r)

            self.particles.append(
                Particle(
                    Vec2(x, y),
                    radius=outer_collision_radius,
                    mass=1.0,
                )
            )

        # 2) center <-> each outer particle (radial / "pressure-like" support)
        for n in range(num_points):
            i = self.center_index
            j = self.ring_start + n
            p_i = self.particles[i]
            p_j = self.particles[j]
            rest = (p_j.pos - p_i.pos).length()
            self.springs.append(
                Spring(
                    i,
                    j,
                    rest,
                    k=self.k_center,
                    damping=self.spring_damping,
                )
            )

        # 3) outer structural springs (neighbor connections)
        for n in range(num_points):
            i = self.ring_start + n
            j = self.ring_start + ((n + 1) % num_points)
            p_i = self.particles[i]
            p_j = self.particles[j]
            rest = (p_j.pos - p_i.pos).length()
            self.springs.append(
                Spring(
                    i,
                    j,
                    rest,
                    k=self.k_struct,
                    damping=self.spring_damping,
                )
            )

        # 4) outer bending springs (skip one particle)
        for n in range(num_points):
            i = self.ring_start + n
            j = self.ring_start + ((n + 2) % num_points)
            p_i = self.particles[i]
            p_j = self.particles[j]
            rest = (p_j.pos - p_i.pos).length()
            self.springs.append(
                Spring(
                    i,
                    j,
                    rest,
                    k=self.k_bend,
                    damping=self.spring_damping,
                )
            )

        # --------------------------------------
        # Internal pressure (area-based) fields
        # --------------------------------------
        self.rest_area: float = self.compute_area()
        self.internal_pressure: float = 0.0

    # --------------------------
    # Compute polygon area of the outer ring
    # --------------------------
    def compute_area(self) -> float:
        """Compute polygon area formed by the outer ring particles."""
        if self.num_points < 3:
            return 0.0

        area = 0.0
        for i in range(self.num_points):
            a = self.particles[self.ring_start + i].pos
            b = self.particles[self.ring_start + ((i + 1) % self.num_points)].pos
            area += a.x * b.y - b.x * a.y

        return abs(area) * 0.5

    # --------------------------
    # Forces: gravity + global velocity damping
    # --------------------------
    def apply_forces(self, gravity: Vec2, velocity_damping: float):
        """Initialize per-particle forces with gravity and apply velocity damping."""
        for p in self.particles:
            p.force = gravity * p.mass
            p.vel *= velocity_damping

    # --------------------------
    # Spring forces (Hooke + damping)
    # --------------------------
    def apply_springs(self):
        """Apply Hooke's law and spring damping for all springs."""
        for s in self.springs:
            pa = self.particles[s.i]
            pb = self.particles[s.j]

            d = pb.pos - pa.pos
            L = d.length()
            if L == 0.0:
                continue

            n = d / L
            stretch = L - s.rest_length

            force_mag = s.k * stretch
            rel_vel = pb.vel - pa.vel
            damping_mag = s.damping * rel_vel.dot(n)

            F = n * (force_mag + damping_mag)

            pa.force += F
            pb.force -= F

    # --------------------------
    # Internal pressure force (area-based)
    # --------------------------
    def apply_internal_pressure(
        self,
        k_pressure: float,
        area_tolerance: float = 0.03,
        max_pressure: float = 1.5,
    ):
        """
        Apply outward force on the outer ring based on the difference
        between current area and rest_area.
        """
        current_area = self.compute_area()
        if self.rest_area <= 0.0:
            return

        ratio = current_area / self.rest_area

        if abs(1.0 - ratio) < area_tolerance:
            self.internal_pressure = 0.0
            return

        pressure_raw = 1.0 - ratio
        if pressure_raw <= 0.0:
            self.internal_pressure = 0.0
            return

        pressure = min(pressure_raw, max_pressure)
        self.internal_pressure = pressure

        center = self.particles[self.center_index].pos

        for n in range(self.num_points):
            idx = self.ring_start + n
            p = self.particles[idx]

            d = p.pos - center
            L = d.length()
            if L == 0.0:
                continue

            dir_vec = d / L

            rest_r = self.rest_radii[n]
            if rest_r > 1e-6:
                radial_ratio = L / rest_r
            else:
                radial_ratio = 1.0

            radial_compression = max(0.0, 1.0 - radial_ratio)
            local_pressure = pressure * (0.5 + radial_compression)

            p.force += dir_vec * (k_pressure * local_pressure)

    # --------------------------
    # Integrate motion (semi-implicit Euler)
    # --------------------------
    def integrate(self, dt: float):
        """Integrate velocities and positions using semi-implicit Euler."""
        max_speed = 800.0
        for p in self.particles:
            acc = p.force / p.mass
            p.vel += acc * dt

            speed = p.vel.length()
            if speed > max_speed:
                p.vel = p.vel * (max_speed / speed)

            p.pos += p.vel * dt

    # --------------------------
    # Radial constraints
    # --------------------------
    def enforce_radial_constraints(
        self,
        min_factor: float = 0.5,
        max_factor: float = 1.5,
    ):
        """Keep outer ring particles within a band of [min_factor, max_factor] of rest radius."""
        center = self.particles[self.center_index].pos

        for n in range(self.num_points):
            idx = self.ring_start + n
            p = self.particles[idx]

            rest_r = self.rest_radii[n]
            min_r = rest_r * min_factor
            max_r = rest_r * max_factor

            d = p.pos - center
            L = d.length()
            if L == 0.0:
                dir_vec = Vec2(1.0, 0.0)
                L = 1.0
            else:
                dir_vec = d / L

            target_r = None

            if L < min_r:
                target_r = min_r
            elif L > max_r:
                target_r = max_r

            if target_r is not None:
                p.pos = center + dir_vec * target_r

                radial_vel = p.vel.dot(dir_vec)
                if (L < min_r and radial_vel < 0.0) or (L > max_r and radial_vel > 0.0):
                    p.vel -= dir_vec * radial_vel

    # --------------------------
    # Center-circle vs wall (dynamic radius)
    # --------------------------
    def core_wall_limit(
        self,
        width: int,
        height: int,
        border_thickness: float = 0.0,
    ):
        """
        Treat the body as a single circle whose radius is the maximum
        distance from the center to any outer particle. Move the entire
        body together to resolve wall penetration. This prevents local
        dents and also avoids sinking into the floor.
        """
        center = self.particles[self.center_index]

        # compute current approximate bounding radius
        max_r = 0.0
        for p in self.particles[1:]:
            d = p.pos - center.pos
            max_r = max(max_r, d.length())
        # ensure at least core_radius
        r = max(self.core_radius, max_r)

        left = border_thickness
        right = width - border_thickness
        top = border_thickness
        bottom = height - border_thickness

        moved = False
        dx = 0.0
        dy = 0.0

        sleep_vel = 20.0

        # left wall
        if center.pos.x - r < left:
            dx = left + r - center.pos.x
            moved = True
            if center.vel.x < 0.0:
                if abs(center.vel.x) < sleep_vel:
                    center.vel.x = 0.0
                else:
                    center.vel.x *= -self.wall_restitution

        # right wall
        elif center.pos.x + r > right:
            dx = right - r - center.pos.x
            moved = True
            if center.vel.x > 0.0:
                if abs(center.vel.x) < sleep_vel:
                    center.vel.x = 0.0
                else:
                    center.vel.x *= -self.wall_restitution

        # top wall
        if center.pos.y - r < top:
            dy = top + r - center.pos.y
            moved = True
            if center.vel.y < 0.0:
                if abs(center.vel.y) < sleep_vel:
                    center.vel.y = 0.0
                else:
                    center.vel.y *= -self.wall_restitution

        # bottom wall
        elif center.pos.y + r > bottom:
            dy = bottom - r - center.pos.y
            moved = True
            if center.vel.y > 0.0:
                if abs(center.vel.y) < sleep_vel:
                    center.vel.y = 0.0
                else:
                    center.vel.y *= -self.wall_restitution

        if moved:
            for p in self.particles:
                p.pos.x += dx
                p.pos.y += dy

    # --------------------------
    # Side wall collision per particle (left/right only)
    # --------------------------
    def wall_collision(
        self,
        width: int,
        height: int,
        border_thickness: float = 0.0,
    ):
        """
        Per-particle collision against side walls (left/right).
        Top/bottom are handled by core_wall_limit().
        """
        left = border_thickness
        right = width - border_thickness

        sleep_vel = 20.0

        for p in self.particles:
            if p.pos.x - p.radius < left:
                p.pos.x = left + p.radius
                if p.vel.x < 0.0:
                    if abs(p.vel.x) < sleep_vel:
                        p.vel.x = 0.0
                    else:
                        p.vel.x *= -self.wall_restitution

            if p.pos.x + p.radius > right:
                p.pos.x = right - p.radius
                if p.vel.x > 0.0:
                    if abs(p.vel.x) < sleep_vel:
                        p.vel.x = 0.0
                    else:
                        p.vel.x *= -self.wall_restitution

    # --------------------------
    # Full update step
    # --------------------------
    def update(
        self,
        dt: float,
        gravity: Vec2 = Vec2(0.0, 500.0),
        velocity_damping: float | None = None,
        screen_size: tuple[int, int] = (800, 600),
        border_thickness: float = 0.0,
        pressure_strength: float | None = None,
    ):
        """
        Full simulation step:

        1) Forces (gravity + damping)
        2) Springs
        3) Internal pressure
        4) Integrate
        5) Radial constraints
        6) Side walls per particle
        7) Core-circle vs walls (floor/ceiling/side)
        """
        vd = self.velocity_damping if velocity_damping is None else velocity_damping
        ps = self.pressure_strength if pressure_strength is None else pressure_strength

        self.apply_forces(gravity, vd)
        self.apply_springs()
        self.apply_internal_pressure(k_pressure=ps)
        self.integrate(dt)

        self.enforce_radial_constraints(min_factor=0.7, max_factor=1.3)
        self.wall_collision(
            screen_size[0],
            screen_size[1],
            border_thickness=border_thickness,
        )
        self.core_wall_limit(
            screen_size[0],
            screen_size[1],
            border_thickness=border_thickness,
        )


# ------------------------------
# Particle-particle collision
# ------------------------------
def resolve_particle_collision(
    pa: Particle,
    pb: Particle,
    restitution: float = 0.2,
):
    """Resolve collision between two circular particles."""
    d = pb.pos - pa.pos
    L = d.length()
    min_dist = pa.radius + pb.radius

    if L == 0.0:
        n = Vec2(1.0, 0.0)
        L = min_dist
    else:
        n = d / L

    penetration = min_dist - L
    if penetration <= 0.0:
        return

    correction = n * (penetration * 0.5)
    pa.pos -= correction
    pb.pos += correction

    rel_vel = pb.vel - pa.vel
    vn = rel_vel.dot(n)
    if vn > 0.0:
        return

    invA = 1.0 / pa.mass
    invB = 1.0 / pb.mass

    j = -(1.0 + restitution) * vn / (invA + invB)
    impulse = n * j

    pa.vel -= impulse * invA
    pb.vel += impulse * invB


# ------------------------------
# Soft body vs soft body collision
# ------------------------------
def softbody_collision(
    A: SoftBody,
    B: SoftBody,
):
    """Bounding-circle approximation for soft body vs soft body collision."""
    ca = A.particles[0]
    cb = B.particles[0]

    ra = 0.0
    for p in A.particles[1:]:
        d = p.pos - ca.pos
        ra = max(ra, d.length())

    rb = 0.0
    for p in B.particles[1:]:
        d = p.pos - cb.pos
        rb = max(rb, d.length())

    margin = 0.0
    ra += margin
    rb += margin

    d = cb.pos - ca.pos
    dist = d.length()
    min_dist = ra + rb

    if dist == 0.0:
        n = Vec2(1.0, 0.0)
        dist = min_dist
    else:
        n = d * (1.0 / dist)

    penetration = min_dist - dist
    if penetration <= 0.0:
        return

    correction = n * (penetration * 0.5)
    for p in A.particles:
        p.pos -= correction
    for p in B.particles:
        p.pos += correction

    def average_velocity(body: SoftBody) -> Vec2:
        if not body.particles:
            return Vec2(0.0, 0.0)
        vx = 0.0
        vy = 0.0
        for pp in body.particles:
            vx += pp.vel.x
            vy += pp.vel.y
        inv_count = 1.0 / len(body.particles)
        return Vec2(vx * inv_count, vy * inv_count)

    va = average_velocity(A)
    vb = average_velocity(B)

    rel_vel = vb - va
    vn = rel_vel.dot(n)
    if vn > 0.0:
        return

    total_mass_A = sum(p.mass for p in A.particles)
    total_mass_B = sum(p.mass for p in B.particles)
    if total_mass_A == 0.0 or total_mass_B == 0.0:
        return

    invA = 1.0 / total_mass_A
    invB = 1.0 / total_mass_B

    restitution = min(A.collision_restitution, B.collision_restitution)
    j = -(1.0 + restitution) * vn / (invA + invB)
    impulse = n * j

    for p in A.particles:
        p.vel -= impulse * (1.0 / total_mass_A)
    for p in B.particles:
        p.vel += impulse * (1.0 / total_mass_B)
