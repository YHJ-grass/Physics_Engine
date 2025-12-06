# main.py
import random
import pygame
from engine import SoftBody, softbody_collision, Vec2

pygame.init()

# window size
WIDTH, HEIGHT = 1200, 800
BORDER_THICKNESS = 20      # visible border only (physics uses window edge)

screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Softbody Jelly Demo")

clock = pygame.time.Clock()

# -------------------------------------------------
# Color definitions
# -------------------------------------------------
BG_COLOR = (0, 0, 0)           # black background
WALL_COLOR = (255, 255, 255)   # white walls (border)
DEFAULT_BALL_FILL = (120, 120, 120)    # fallback gray inside of the soft body
BALL_OUTLINE = (255, 255, 255)         # white outline for the soft body


# -------------------------------------------------
# Material presets
# -------------------------------------------------
MATERIAL_PRESETS = [
    {
        # Bouncy and relatively stiff like a rubber ball
        "name": "bouncy",
        "k_struct": 1600.0,
        "k_bend": 900.0,
        "k_center": 2000.0,
        "spring_damping": 3.0,
        "velocity_damping": 0.995,
        "wall_restitution": 0.8,
        "collision_restitution": 0.8,
        "pressure_strength": 1600.0,
    },
    {
        # Soft jelly: medium stiffness, medium damping
        "name": "jelly",
        "k_struct": 1100.0,
        "k_bend": 600.0,
        "k_center": 1600.0,
        "spring_damping": 5.0,
        "velocity_damping": 0.993,
        "wall_restitution": 0.4,
        "collision_restitution": 0.4,
        "pressure_strength": 2000.0,
    },
    {
        # Slime-like: low stiffness, high damping, low restitution
        "name": "slime",
        "k_struct": 700.0,
        "k_bend": 350.0,
        "k_center": 1200.0,
        "spring_damping": 7.0,
        "velocity_damping": 0.99,
        "wall_restitution": 0.15,
        "collision_restitution": 0.15,
        "pressure_strength": 1400.0,
    },
    {
        # Heavy gel: quite stiff but low bounce, feels heavy
        "name": "heavy_gel",
        "k_struct": 1400.0,
        "k_bend": 800.0,
        "k_center": 1900.0,
        "spring_damping": 6.0,
        "velocity_damping": 0.992,
        "wall_restitution": 0.25,
        "collision_restitution": 0.25,
        "pressure_strength": 1600.0,
    },
    {
        # Super bouncy: very elastic and light
        "name": "super_bouncy",
        "k_struct": 1800.0,
        "k_bend": 1100.0,
        "k_center": 2200.0,
        "spring_damping": 2.5,
        "velocity_damping": 0.997,
        "wall_restitution": 0.95,
        "collision_restitution": 0.95,
        "pressure_strength": 1500.0,
    },
]


def restitution_to_gray(restitution: float) -> tuple[int, int, int]:
    """
    Map restitution value to a gray color:
    - low restitution -> darker gray
    - high restitution -> lighter gray
    """
    min_r = 0.1
    max_r = 0.95
    r = max(min_r, min(max_r, restitution))

    t = (r - min_r) / (max_r - min_r)
    gray = int(80 + t * (210 - 80))
    return (gray, gray, gray)


def clamp_mouse() -> Vec2:
    """Return mouse position clamped to the window rectangle."""
    mx, my = pygame.mouse.get_pos()
    mx = max(0, min(WIDTH - 1, mx))
    my = max(0, min(HEIGHT - 1, my))
    return Vec2(float(mx), float(my))


def get_body_center(body: SoftBody) -> Vec2:
    """Return the current center position of the soft body."""
    return Vec2(body.particles[0].pos.x, body.particles[0].pos.y)


def get_body_radius(body: SoftBody) -> float:
    """Return approximate radius of the body from center to outer particles."""
    center = get_body_center(body)
    r = 0.0
    for p in body.particles[1:]:
        d = p.pos - center
        r = max(r, d.length())
    return r


def move_body_to(body: SoftBody, target: Vec2):
    """Move all particles so that the body's center becomes target (clamped to window)."""
    center = get_body_center(body)
    radius = get_body_radius(body)

    margin = 2.0
    min_x = BORDER_THICKNESS + radius + margin
    max_x = WIDTH - BORDER_THICKNESS - radius - margin
    min_y = BORDER_THICKNESS + radius + margin
    max_y = HEIGHT - BORDER_THICKNESS - radius - margin

    tx = max(min_x, min(max_x, target.x))
    ty = max(min_y, min(max_y, target.y))
    clamped_target = Vec2(tx, ty)

    delta = clamped_target - center
    for p in body.particles:
        p.pos.x += delta.x
        p.pos.y += delta.y


def set_body_velocity(body: SoftBody, vel: Vec2):
    """Set the same velocity to all particles of the body."""
    for p in body.particles:
        p.vel.x = vel.x
        p.vel.y = vel.y


def create_softbody_at(pos: Vec2, radius: float = 80.0, num_points: int = 20) -> SoftBody:
    """
    Create a new soft body at the given position using a random material preset.
    Also assign a fill color based on the preset's restitution.
    """
    preset = random.choice(MATERIAL_PRESETS)

    body = SoftBody(
        center=Vec2(pos.x, pos.y),
        radius=radius,
        num_points=num_points,
        k_struct=preset["k_struct"],
        k_bend=preset["k_bend"],
        k_center=preset["k_center"],
        spring_damping=preset["spring_damping"],
        velocity_damping=preset["velocity_damping"],
        wall_restitution=preset["wall_restitution"],
        collision_restitution=preset["collision_restitution"],
        pressure_strength=preset["pressure_strength"],
    )

    gray_color = restitution_to_gray(preset["wall_restitution"])
    body.fill_color = gray_color
    body.material_name = preset["name"]

    move_body_to(body, pos)
    return body


# initial bodies
bodies: list[SoftBody] = []
bodies.append(create_softbody_at(Vec2(500.0, 350.0), radius=80.0, num_points=20))
bodies.append(create_softbody_at(Vec2(750.0, 350.0), radius=80.0, num_points=20))

# drag state
dragging = False
drag_body: SoftBody | None = None
drag_start_mouse = Vec2(0.0, 0.0)
drag_start_center = Vec2(0.0, 0.0)

LAUNCH_STRENGTH = 6.0
PICK_RADIUS = 100.0
MAX_DRAG_DIST = 250.0


running = True
while running:
    dt = clock.tick(60) / 1000.0

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

        elif event.type == pygame.MOUSEBUTTONDOWN:
            # left click: drag
            if event.button == 1:
                mouse_pos = clamp_mouse()
                for body in bodies:
                    center = get_body_center(body)
                    if (center - mouse_pos).length() <= PICK_RADIUS:
                        dragging = True
                        drag_body = body
                        drag_start_mouse = mouse_pos
                        drag_start_center = center
                        set_body_velocity(drag_body, Vec2(0.0, 0.0))
                        break

            # right click: spawn new soft body
            elif event.button == 3:
                mouse_pos = clamp_mouse()
                new_body = create_softbody_at(mouse_pos, radius=80.0, num_points=20)
                bodies.append(new_body)

        elif event.type == pygame.MOUSEBUTTONUP and event.button == 1:
            if dragging and drag_body is not None:
                current_center = get_body_center(drag_body)
                drag_vec = drag_start_center - current_center

                length = drag_vec.length()
                if length > MAX_DRAG_DIST:
                    drag_vec = drag_vec * (MAX_DRAG_DIST / length)

                launch_vel = drag_vec * LAUNCH_STRENGTH
                set_body_velocity(drag_body, launch_vel)

            dragging = False
            drag_body = None

    # physics update
    for body in bodies:
        if dragging and body is drag_body:
            continue
        body.update(
            dt,
            screen_size=(WIDTH, HEIGHT),
            border_thickness=BORDER_THICKNESS,
        )

    # soft body vs soft body collision
    for i in range(len(bodies)):
        for j in range(i + 1, len(bodies)):
            softbody_collision(bodies[i], bodies[j])

    # while dragging, move the body with the mouse
    if dragging and drag_body is not None:
        mouse_pos = clamp_mouse()
        drag_vec = mouse_pos - drag_start_center
        length = drag_vec.length()
        if length > MAX_DRAG_DIST:
            drag_vec = drag_vec * (MAX_DRAG_DIST / length)
        target_pos = drag_start_center + drag_vec

        move_body_to(drag_body, target_pos)
        set_body_velocity(drag_body, Vec2(0.0, 0.0))

    # rendering
    screen.fill(BG_COLOR)

    pygame.draw.rect(
        screen,
        WALL_COLOR,
        pygame.Rect(0, 0, WIDTH, HEIGHT),
        width=BORDER_THICKNESS,
    )

    for body in bodies:
        outer_points = [
            (int(p.pos.x), int(p.pos.y))
            for p in body.particles[1:]
        ]

        if len(outer_points) >= 3:
            fill_color = getattr(body, "fill_color", DEFAULT_BALL_FILL)
            pygame.draw.polygon(screen, fill_color, outer_points)
            pygame.draw.polygon(
                screen,
                BALL_OUTLINE,
                outer_points,
                width=5,
            )

    if dragging and drag_body is not None:
        current_center = get_body_center(drag_body)
        pygame.draw.line(
            screen,
            WALL_COLOR,
            (int(drag_start_center.x), int(drag_start_center.y)),
            (int(current_center.x), int(current_center.y)),
            width=3,
        )
        pygame.draw.circle(
            screen,
            WALL_COLOR,
            (int(drag_start_center.x), int(drag_start_center.y)),
            6,
            width=2,
        )

    pygame.display.flip()

pygame.quit()
