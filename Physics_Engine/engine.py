# engine.py
import math

class Vec2:
    def __init__(self, x=0.0, y=0.0):
        self.x = float(x)
        self.y = float(y)
    
    def __add__(self, other):
        return Vec2(self.x + other.x, self.y + other.y)
    
    def __sub__(self, other):
        return Vec2(self.x - other.x, self.y - other.y)
    
    def __mul__(self, scalar: float):
        return Vec2(self.x * scalar, self.y * scalar)
    
    __rmul__ = __mul__
    
    def dot(self, other) -> float:
        return self.x * other.x + self.y * other.y
    
    def length(self) -> float:
        return math.sqrt(self.x * self.x + self.y * self.y)
    
    def normalized(self):
        l = self.length()
        if l == 0:
            return Vec2(0, 0)
        return Vec2(self.x / l, self.y / l)
    
    def perp(self):
        """Returns a perpendicular vector (rotated 90 degrees)."""
        return Vec2(-self.y, self.x)
    
    def __repr__(self):
        return f"Vec2({self.x}, {self.y})"


class RigidBodyCircle:
    def __init__(self, position, radius, mass=1.0, restitution=0.8, shape="circle"):
        # 2D position and velocity in the X?Y plane (what we render)
        self.position = position
        self.velocity = Vec2(0, 0)
        
        # Z axis (not rendered yet, used conceptually for gravity)
        self.z = 0.0
        self.vz = 0.0
        
        # Basic physical properties
        self.radius = radius
        self.mass = float(mass)
        self.inv_mass = 0.0 if mass == 0 else 1.0 / mass
        self.restitution = restitution
        
        # Visual shape: "circle", "box", "triangle"
        self.shape = shape
        
        # If true, body is locked in X?Y (used while dragging)
        self.locked = False
        
        # Accumulated force in X?Y (cleared each frame)
        self.force_acc = Vec2(0, 0)


class Contact:
    def __init__(self, A, B, normal, penetration):
        self.A = A
        self.B = B
        self.normal = normal
        self.penetration = penetration


# Global physics parameters
GRAVITY_Z = -400.0          # Gravity along Z axis
WALL_RESTITUTION = 0.9      # Restitution for wall bounces
LINEAR_DAMPING = 0.5        # Linear damping in the plane
FRICTION_COEFF = 0.5        # Coulomb friction coefficient
CIRCLE_SEGMENTS = 10        # Segments for approximating circles as polygons


def apply_impulse(body: RigidBodyCircle, impulse: Vec2):
    """Applies an instantaneous impulse to a body (changes velocity)."""
    if body.inv_mass == 0.0:
        return
    body.velocity = body.velocity + impulse * body.inv_mass


def integrate(body: RigidBodyCircle, dt: float):
    """
    Integrates motion using semi-implicit Euler:
    - X-Y plane: uses accumulated forces and linear damping.
    - Z axis: gravity only.
    """
    if body.inv_mass == 0.0 or body.locked:
        return
    
    # X?Y: apply accumulated forces
    ax = body.force_acc.x * body.inv_mass
    ay = body.force_acc.y * body.inv_mass
    a_xy = Vec2(ax, ay)
    
    body.velocity = body.velocity + a_xy * dt
    body.position = body.position + body.velocity * dt
    
    # Linear damping (simple friction-like effect in the plane)
    damping_factor = max(0.0, 1.0 - LINEAR_DAMPING * dt)
    body.velocity = body.velocity * damping_factor
    
    # Z: gravity only (not rendered yet)
    az = GRAVITY_Z
    body.vz += az * dt
    body.z += body.vz * dt
    
    # Clear forces
    body.force_acc = Vec2(0, 0)


def circle_vs_circle(A: RigidBodyCircle, B: RigidBodyCircle):
    """Exact circle-circle collision in X-Y."""
    n = B.position - A.position
    dist = n.length()
    r = A.radius + B.radius
    
    if dist >= r:
        return None
    
    if dist == 0:
        normal = Vec2(1, 0)
        penetration = r
    else:
        normal = n * (1.0 / dist)
        penetration = r - dist
    
    return Contact(A, B, normal, penetration)


def build_polygon_vertices(body: RigidBodyCircle):
    """
    Builds a polygon representation of the body in world space.
    - circle: approximated by a regular polygon (CIRCLE_SEGMENTS sides)
    - box: axis-aligned square
    - triangle: simple upright triangle
    """
    cx = body.position.x
    cy = body.position.y
    r = body.radius
    
    if body.shape == "circle":
        verts = []
        for i in range(CIRCLE_SEGMENTS):
            angle = 2.0 * math.pi * i / CIRCLE_SEGMENTS
            verts.append(Vec2(cx + r * math.cos(angle),
                              cy + r * math.sin(angle)))
        return verts
    
    if body.shape == "box":
        return [
            Vec2(cx - r, cy - r),
            Vec2(cx + r, cy - r),
            Vec2(cx + r, cy + r),
            Vec2(cx - r, cy + r),
        ]
    
    if body.shape == "triangle":
        return [
            Vec2(cx,     cy - r),
            Vec2(cx - r, cy + r),
            Vec2(cx + r, cy + r),
        ]
    
    # Fallback: approximate as circle
    verts = []
    for i in range(CIRCLE_SEGMENTS):
        angle = 2.0 * math.pi * i / CIRCLE_SEGMENTS
        verts.append(Vec2(cx + r * math.cos(angle),
                          cy + r * math.sin(angle)))
    return verts


def project_polygon(axis: Vec2, verts):
    """Projects polygon vertices onto an axis and returns min/max scalar values."""
    min_proj = float("inf")
    max_proj = float("-inf")
    for v in verts:
        p = v.dot(axis)
        if p < min_proj:
            min_proj = p
        if p > max_proj:
            max_proj = p
    return min_proj, max_proj


def overlap_intervals(minA, maxA, minB, maxB):
    """Returns overlap between two intervals. 0 if they do not overlap."""
    if maxA < minB or maxB < minA:
        return 0.0
    return min(maxA, maxB) - max(minA, minB)


def polygon_vs_polygon(A: RigidBodyCircle, B: RigidBodyCircle):
    """
    SAT-based collision detection for two convex polygons.
    Circles are approximated as polygons if needed.
    """
    vertsA = build_polygon_vertices(A)
    vertsB = build_polygon_vertices(B)
    
    smallest_overlap = float("inf")
    smallest_axis = None
    
    def test_axes(verts):
        nonlocal smallest_overlap, smallest_axis
        count = len(verts)
        for i in range(count):
            p1 = verts[i]
            p2 = verts[(i + 1) % count]
            edge = p2 - p1
            axis = edge.perp().normalized()
            if axis.length() == 0:
                continue
            minA, maxA = project_polygon(axis, vertsA)
            minB, maxB = project_polygon(axis, vertsB)
            o = overlap_intervals(minA, maxA, minB, maxB)
            if o <= 0:
                return False
            if o < smallest_overlap:
                smallest_overlap = o
                smallest_axis = axis
        return True
    
    # Test axes from both polygons
    if not test_axes(vertsA):
        return None
    if not test_axes(vertsB):
        return None
    
    # Ensure normal points from A to B
    center_dir = B.position - A.position
    if center_dir.dot(smallest_axis) < 0:
        smallest_axis = smallest_axis * -1.0
    
    return Contact(A, B, smallest_axis, smallest_overlap)


def find_contact(A: RigidBodyCircle, B: RigidBodyCircle):
    """
    Dispatches to the appropriate collision function:
    - circle-circle: exact
    - otherwise: polygon SAT (circles approximated as polygons)
    """
    if A.shape == "circle" and B.shape == "circle":
        return circle_vs_circle(A, B)
    return polygon_vs_polygon(A, B)


def resolve_collision(contact: Contact):
    """
    Resolves collision using impulse-based response with friction:
    - Normal impulse controls bounce.
    - Tangent impulse approximates friction.
    """
    A = contact.A
    B = contact.B
    n = contact.normal
    
    # Relative velocity along normal
    rv = B.velocity - A.velocity
    vel_along_normal = rv.dot(n)
    
    if vel_along_normal > 0:
        return
    
    e = min(A.restitution, B.restitution)
    
    inv_mass_sum = A.inv_mass + B.inv_mass
    if inv_mass_sum == 0:
        return
    
    # Normal impulse
    j = -(1 + e) * vel_along_normal
    j /= inv_mass_sum
    
    impulse = n * j
    A.velocity = A.velocity - impulse * A.inv_mass
    B.velocity = B.velocity + impulse * B.inv_mass
    
    # Friction impulse
    rv = B.velocity - A.velocity
    tangent = rv - n * rv.dot(n)
    t_len = tangent.length()
    if t_len != 0:
        tangent = tangent * (1.0 / t_len)
    else:
        tangent = Vec2(0, 0)
    
    jt = -rv.dot(tangent)
    jt /= inv_mass_sum
    
    # Coulomb friction model
    if abs(jt) > j * FRICTION_COEFF:
        jt = j * FRICTION_COEFF * (1 if jt > 0 else -1)
    
    friction_impulse = tangent * jt
    A.velocity = A.velocity - friction_impulse * A.inv_mass
    B.velocity = B.velocity + friction_impulse * B.inv_mass


def positional_correction(contact: Contact, percent=0.4, slop=0.01):
    """Separates overlapping bodies to reduce sinking."""
    A = contact.A
    B = contact.B
    n = contact.normal
    
    inv_mass_sum = A.inv_mass + B.inv_mass
    if inv_mass_sum == 0:
        return
    
    correction_mag = max(contact.penetration - slop, 0.0) * percent / inv_mass_sum
    correction = n * correction_mag
    
    A.position = A.position - correction * A.inv_mass
    B.position = B.position + correction * B.inv_mass


def handle_bounds(body: RigidBodyCircle, width: int, height: int):
    """Keeps bodies inside the screen and makes them bounce off borders."""
    if body.inv_mass == 0.0:
        return
    
    # Left / Right
    if body.position.x - body.radius < 0:
        body.position.x = body.radius
        body.velocity.x *= -WALL_RESTITUTION
    elif body.position.x + body.radius > width:
        body.position.x = width - body.radius
        body.velocity.x *= -WALL_RESTITUTION
    
    # Top / Bottom
    if body.position.y - body.radius < 0:
        body.position.y = body.radius
        body.velocity.y *= -WALL_RESTITUTION
    elif body.position.y + body.radius > height:
        body.position.y = height - body.radius
        body.velocity.y *= -WALL_RESTITUTION


def step(bodies, dt, width, height):
    """Runs a full physics step on all bodies."""
    for body in bodies:
        integrate(body, dt)
    
    contacts = []
    n = len(bodies)
    for i in range(n):
        for j in range(i + 1, n):
            c = find_contact(bodies[i], bodies[j])
            if c is not None:
                contacts.append(c)
    
    for c in contacts:
        resolve_collision(c)
        positional_correction(c)
    
    for body in bodies:
        handle_bounds(body, width, height)
