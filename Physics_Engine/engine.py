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
    
    def __repr__(self):
        return f"Vec2({self.x}, {self.y})"


class RigidBodyCircle:
    def __init__(self, position, radius, mass=1.0, restitution=0.8, shape="circle"):
        # 2D position in the X?Y plane (what we render)
        self.position = position          # Vec2
        self.velocity = Vec2(0, 0)        # Vec2, velocity in X?Y
        
        # "Depth" along Z (coming out of the screen)
        # For a top-down view, we conceptually use this as height,
        # but it is not rendered yet.
        self.z = 0.0                      # position along Z
        self.vz = 0.0                     # velocity along Z
        
        # Physical properties
        self.radius = radius
        self.mass = float(mass)
        self.inv_mass = 0.0 if mass == 0 else 1.0 / mass
        self.restitution = restitution
        
        # For visualization only (circle / box / triangle, etc.)
        self.shape = shape

        # If true, body will not move in X?Y (used while dragging with the mouse)
        self.locked = False
        
        # Accumulated external force in X?Y (cleared each frame)
        self.force_acc = Vec2(0, 0)


class Contact:
    def __init__(self, A, B, normal, penetration):
        self.A = A
        self.B = B
        self.normal = normal
        self.penetration = penetration


# Global physics parameters
# We now think in 3D: (x, y) is the plane, z is "up".
# Gravity acts along the Z axis only (top-down view like curling).
GRAVITY_Z = -400.0          # acceleration along Z (units per second^2)
WALL_RESTITUTION = 0.9      # restitution when bouncing off screen borders (X?Y)


def integrate(body: RigidBodyCircle, dt: float):
    """
    Updates velocity and position using semi-implicit Euler integration.
    - X?Y plane: only user forces affect motion (no gravity in plane).
    - Z axis: gravity acts here (conceptual height).
    """
    if body.inv_mass == 0.0 or body.locked:
        return
    
    # X?Y: use accumulated force in the plane
    # There is no gravity in X?Y because gravity is along Z.
    ax = body.force_acc.x * body.inv_mass
    ay = body.force_acc.y * body.inv_mass
    a_xy = Vec2(ax, ay)
    
    body.velocity = body.velocity + a_xy * dt
    body.position = body.position + body.velocity * dt
    
    # Z: apply gravity along Z (not rendered yet, but kept for future 3D extension)
    az = GRAVITY_Z
    body.vz += az * dt
    body.z += body.vz * dt
    
    # Clear accumulated force in X?Y
    body.force_acc = Vec2(0, 0)


def circle_vs_circle(A: RigidBodyCircle, B: RigidBodyCircle):
    """
    Detects collision between two circles in the X?Y plane.
    Z is ignored here; we assume all bodies lie on the same plane for now.
    """
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


def resolve_collision(contact: Contact):
    """
    Velocity correction using impulse-based collision response
    in the X?Y plane.
    """
    A = contact.A
    B = contact.B
    n = contact.normal
    
    rv = B.velocity - A.velocity
    vel_along_normal = rv.dot(n)
    
    if vel_along_normal > 0:
        return

    e = min(A.restitution, B.restitution)
    
    inv_mass_sum = A.inv_mass + B.inv_mass
    if inv_mass_sum == 0:
        return
    
    j = -(1 + e) * vel_along_normal
    j /= inv_mass_sum
    
    impulse = n * j
    
    A.velocity = A.velocity - impulse * A.inv_mass
    B.velocity = B.velocity + impulse * B.inv_mass


def positional_correction(contact: Contact, percent=0.4, slop=0.01):
    """
    Prevents overlapping circles from sinking into each other
    by separating them slightly in the X?Y plane.
    """
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
    """
    Handles collisions against the screen boundaries in the X?Y plane.
    Gravity along Z does not affect this; we just keep objects inside the view.
    """
    if body.inv_mass == 0.0:
        return
    
    # Left / Right walls (X axis)
    if body.position.x - body.radius < 0:
        body.position.x = body.radius
        body.velocity.x *= -WALL_RESTITUTION
    elif body.position.x + body.radius > width:
        body.position.x = width - body.radius
        body.velocity.x *= -WALL_RESTITUTION
    
    # Top / Bottom walls (Y axis)
    if body.position.y - body.radius < 0:
        body.position.y = body.radius
        body.velocity.y *= -WALL_RESTITUTION
    elif body.position.y + body.radius > height:
        body.position.y = height - body.radius
        body.velocity.y *= -WALL_RESTITUTION


def step(bodies, dt, width, height):
    """
    Runs a full physics step:
    1) Integrate X?Y and Z.
    2) Resolve circle-circle collisions in X?Y.
    3) Handle screen borders in X?Y.
    """
    for body in bodies:
        integrate(body, dt)
    
    contacts = []
    n = len(bodies)
    for i in range(n):
        for j in range(i + 1, n):
            c = circle_vs_circle(bodies[i], bodies[j])
            if c is not None:
                contacts.append(c)
    
    for c in contacts:
        resolve_collision(c)
        positional_correction(c)
    
    for body in bodies:
        handle_bounds(body, width, height)