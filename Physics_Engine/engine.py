#engine.py
import math

# ------------------------------
# 2D vector
# ------------------------------
class Vec2:
    def __init__(self, x=0.0, y=0.0):
        self.x = x
        self.y = y

    def __add__(self, other):
        return Vec2(self.x + other.x, self.y + other.y)

    def __sub__(self, other):
        return Vec2(self.x - other.x, self.y - other.y)

    def __mul__(self, s: float):
        return Vec2(self.x * s, self.y * s)

    def __truediv__(self, s: float):
        return Vec2(self.x / s, self.y / s)

    def dot(self, other) -> float:
        return self.x * other.x + self.y * other.y

    def length(self) -> float:
        return math.sqrt(self.x * self.x + self.y * self.y)

    def normalized(self):
        L = self.length()
        if L == 0.0:
            return Vec2(0.0, 0.0)
        return Vec2(self.x / L, self.y / L)


# ------------------------------
# Particle (point mass)
# ------------------------------
class Particle:
    def __init__(self, pos: Vec2, radius=8.0, mass=1.0):
        self.pos = Vec2(pos.x, pos.y)
        self.vel = Vec2(0.0, 0.0)
        self.force = Vec2(0.0, 0.0)
        self.radius = radius
        self.mass = mass


# ------------------------------
# Spring between two particles
# ------------------------------
class Spring:
    def __init__(self, i: int, j: int,
                 rest_length: float,
                 k: float = 500.0,
                 damping: float = 2.0):
        self.i = i
        self.j = j
        self.rest_length = rest_length
        self.k = k
        self.damping = damping


# ------------------------------
# Soft body (jelly-like object)
# center particle + outer ring + springs
# ------------------------------
class SoftBody:
    def __init__(self, center: Vec2,
                 radius: float = 60.0,
                 num_points: int = 20):
        self.particles: list[Particle] = []
        self.springs: list[Spring] = []

        # indices and rest radii for radial constraints
        self.center_index: int = 0
        self.ring_start: int = 1
        self.num_points: int = num_points
        self.rest_radii: list[float] = []

        self.core_radius: float = radius * 0.6

        # smaller collision radius to reduce visible gap
        center_collision_radius = 6.0
        outer_collision_radius = 4.0

        # 0) center particle
        self.particles.append(
            Particle(Vec2(center.x, center.y),
                     radius=center_collision_radius,
                     mass=2.0)
        )

        # 1) outer ring particles
        for n in range(num_points):
            angle = 2.0 * math.pi * n / num_points
            x = center.x + radius * math.cos(angle)
            y = center.y + radius * math.sin(angle)

            # rest radial distance from center
            rest_r = (Vec2(x, y) - center).length()
            self.rest_radii.append(rest_r)

            self.particles.append(
                Particle(Vec2(x, y),
                         radius=outer_collision_radius,
                         mass=1.0)
            )


        k_struct = 1200.0      # structural spring stiffness
        k_bend = 600.0         # bending spring stiffness
        k_center = 1500.0      # center-to-ring spring stiffness
        damping = 4.0          # spring damping

        # 2) center <-> each outer particle (acts like pressure)
        for n in range(num_points):
            i = self.center_index
            j = self.ring_start + n
            p_i = self.particles[i]
            p_j = self.particles[j]
            rest = (p_j.pos - p_i.pos).length()
            self.springs.append(Spring(i, j, rest, k_center, damping))

        # 3) outer structural springs (neighbor)
        for n in range(num_points):
            i = self.ring_start + n
            j = self.ring_start + ((n + 1) % num_points)
            p_i = self.particles[i]
            p_j = self.particles[j]
            rest = (p_j.pos - p_i.pos).length()
            self.springs.append(Spring(i, j, rest, k_struct, damping))

        # 4) outer bending springs (skip one)
        for n in range(num_points):
            i = self.ring_start + n
            j = self.ring_start + ((n + 2) % num_points)
            p_i = self.particles[i]
            p_j = self.particles[j]
            rest = (p_j.pos - p_i.pos).length()
            self.springs.append(Spring(i, j, rest, k_bend, damping))

    # --------------------------
    # Forces: gravity + damping
    # --------------------------
    def apply_forces(self, gravity: Vec2, velocity_damping: float):
        for p in self.particles:
            p.force = gravity * p.mass
            p.vel *= velocity_damping

    # --------------------------
    # Spring forces (Hooke + damping)
    # --------------------------
    def apply_springs(self):
        for s in self.springs:
            pa = self.particles[s.i]
            pb = self.particles[s.j]

            d = pb.pos - pa.pos
            L = d.length()
            if L == 0.0:
                continue

            n = d / L
            stretch = L - s.rest_length

            # Hooke force
            force_mag = s.k * stretch

            # Spring damping along the direction
            rel_vel = pb.vel - pa.vel
            damping_mag = s.damping * rel_vel.dot(n)

            F = n * (force_mag + damping_mag)

            pa.force += F
            pb.force -= F

    # --------------------------
    # Integrate motion (semi-implicit Euler)
    # --------------------------
    def integrate(self, dt: float):
        max_speed = 800.0
        for p in self.particles:
            acc = p.force / p.mass
            p.vel += acc * dt

            # clamp velocity to avoid explosion
            speed = p.vel.length()
            if speed > max_speed:
                p.vel = p.vel * (max_speed / speed)

            p.pos += p.vel * dt

    # --------------------------
    # Radial constraints: prevent outer points from crossing the center
    # --------------------------
    def enforce_radial_constraints(self,
                                   min_factor: float = 0.5,
                                   max_factor: float = 1.5):
        center = self.particles[self.center_index].pos

        for n in range(self.num_points):
            idx = self.ring_start + n
            p = self.particles[idx]

            rest_r = self.rest_radii[n]

            # desired min and max radius from center
            min_r = rest_r * min_factor
            max_r = rest_r * max_factor

            d = p.pos - center
            L = d.length()
            if L == 0.0:
                # if somehow exactly at center, give a default direction
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
                # project position back onto allowed radius
                p.pos = center + dir_vec * target_r

                # remove radial velocity component that pushes further out of range
                radial_vel = p.vel.dot(dir_vec)

                # if we were inside and moving further in, or outside and moving further out
                if (L < min_r and radial_vel < 0.0) or (L > max_r and radial_vel > 0.0):
                    p.vel -= dir_vec * radial_vel

    # --------------------------
    # Center-circle vs wall: keeps whole body from entering corners too deep
    # --------------------------
    def core_wall_limit(self,
                        width: int,
                        height: int,
                        border_thickness: float = 0.0,
                        restitution: float = 0.2):
        center = self.particles[self.center_index]

        left = border_thickness
        right = width - border_thickness
        top = border_thickness
        bottom = height - border_thickness

        moved = False
        dx = 0.0
        dy = 0.0

        # left wall
        if center.pos.x - self.core_radius < left:
            dx = left + self.core_radius - center.pos.x
            moved = True
            if center.vel.x < 0.0:
                center.vel.x *= -restitution

        # right wall
        elif center.pos.x + self.core_radius > right:
            dx = right - self.core_radius - center.pos.x
            moved = True
            if center.vel.x > 0.0:
                center.vel.x *= -restitution

        # top wall
        if center.pos.y - self.core_radius < top:
            dy = top + self.core_radius - center.pos.y
            moved = True
            if center.vel.y < 0.0:
                center.vel.y *= -restitution

        # bottom wall
        elif center.pos.y + self.core_radius > bottom:
            dy = bottom - self.core_radius - center.pos.y
            moved = True
            if center.vel.y > 0.0:
                center.vel.y *= -restitution

        if moved:
            # move all particles by the same offset so shape is preserved
            for p in self.particles:
                p.pos.x += dx
                p.pos.y += dy


    # --------------------------
    # Collision with screen boundaries (with inner border thickness)
    # --------------------------
    def wall_collision(self,
                       width: int,
                       height: int,
                       border_thickness: float = 0.0,
                       restitution: float = 0.25):
        # inner bounds where particles are allowed to move
        left = border_thickness
        right = width - border_thickness
        top = border_thickness
        bottom = height - border_thickness

        for p in self.particles:
            # bottom
            if p.pos.y + p.radius > bottom:
                p.pos.y = bottom - p.radius
                if p.vel.y > 0.0:
                    p.vel.y *= -restitution

            # top
            if p.pos.y - p.radius < top:
                p.pos.y = top + p.radius
                if p.vel.y < 0.0:
                    p.vel.y *= -restitution

            # left
            if p.pos.x - p.radius < left:
                p.pos.x = left + p.radius
                if p.vel.x < 0.0:
                    p.vel.x *= -restitution

            # right
            if p.pos.x + p.radius > right:
                p.pos.x = right - p.radius
                if p.vel.x > 0.0:
                    p.vel.x *= -restitution

    # --------------------------
    # Full update step
    # --------------------------
    def update(self,
               dt: float,
               gravity: Vec2 = Vec2(0.0, 500.0),
               velocity_damping: float = 0.995,
               screen_size: tuple[int, int] = (800, 600),
               border_thickness: float = 0.0):
        self.apply_forces(gravity, velocity_damping)
        self.apply_springs()
        self.integrate(dt)

        # keep outer points from crossing the center (soft shape constraint)
        self.enforce_radial_constraints(min_factor=0.7,
                                        max_factor=1.3)

        # per-particle vs wall (soft squish on contact)
        self.wall_collision(screen_size[0],
                            screen_size[1],
                            border_thickness=border_thickness)

        # center-circle vs wall (prevent extreme corner penetration)
        self.core_wall_limit(screen_size[0],
                             screen_size[1],
                             border_thickness=border_thickness)



# ------------------------------
# Particle-particle collision
# ------------------------------
def resolve_particle_collision(pa: Particle,
                               pb: Particle,
                               restitution: float = 0.2):
    d = pb.pos - pa.pos
    L = d.length()
    min_dist = pa.radius + pb.radius

    if L == 0.0:
        # avoid division by zero
        n = Vec2(1.0, 0.0)
        L = min_dist
    else:
        n = d / L

    penetration = min_dist - L
    if penetration <= 0.0:
        return

    # positional correction
    correction = n * (penetration * 0.5)
    pa.pos -= correction
    pb.pos += correction

    # relative velocity along normal
    rel_vel = pb.vel - pa.vel
    vn = rel_vel.dot(n)

    # already separating
    if vn > 0.0:
        return

    invA = 1.0 / pa.mass
    invB = 1.0 / pb.mass

    j = -(1.0 + restitution) * vn / (invA + invB)
    impulse = n * j

    pa.vel -= impulse * invA
    pb.vel += impulse * invB


def softbody_collision(A: SoftBody, B: SoftBody,
                       restitution: float = 0.2):
    """
    Soft body vs soft body collision using a single bounding circle
    for each body. Position and velocity corrections are applied to
    all particles of each body to keep the shape stable.
    """

    # use first particle as center reference
    ca = A.particles[0]
    cb = B.particles[0]

    # compute bounding radius for each body (max distance from center)
    ra = 0.0
    for p in A.particles[1:]:
        d = p.pos - ca.pos
        ra = max(ra, d.length())

    rb = 0.0
    for p in B.particles[1:]:
        d = p.pos - cb.pos
        rb = max(rb, d.length())

    # small margin to make collision a bit thicker
    margin = 0.0
    ra += margin
    rb += margin

    # relative center position
    d = cb.pos - ca.pos
    dist = d.length()
    min_dist = ra + rb

    if dist == 0.0:
        # avoid division by zero
        n = Vec2(1.0, 0.0)
        dist = min_dist
    else:
        n = d * (1.0 / dist)

    penetration = min_dist - dist
    if penetration <= 0.0:
        return

    # --------------------------
    # positional correction
    # (move both bodies apart)
    # --------------------------
    correction = n * (penetration * 0.5)

    for p in A.particles:
        p.pos -= correction

    for p in B.particles:
        p.pos += correction

    # --------------------------
    # compute average velocities
    # (approximate center-of-mass velocity)
    # --------------------------
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

    # already separating
    if vn > 0.0:
        return

    # --------------------------
    # impulse based on total mass
    # --------------------------
    total_mass_A = sum(p.mass for p in A.particles)
    total_mass_B = sum(p.mass for p in B.particles)

    if total_mass_A == 0.0 or total_mass_B == 0.0:
        return

    invA = 1.0 / total_mass_A
    invB = 1.0 / total_mass_B

    j = -(1.0 + restitution) * vn / (invA + invB)
    impulse = n * j

    # distribute impulse to all particles
    for p in A.particles:
        p.vel -= impulse * (1.0 / total_mass_A)

    for p in B.particles:
        p.vel += impulse * (1.0 / total_mass_B)