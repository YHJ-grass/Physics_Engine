# main.py
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
BALL_FILL = (120, 120, 120)    # gray inside of the soft body
BALL_OUTLINE = (255, 255, 255) # white outline for the soft body

# create two soft bodies
body1 = SoftBody(Vec2(500.0, 350.0), radius=80.0, num_points=20)
body2 = SoftBody(Vec2(750.0, 350.0), radius=80.0, num_points=20)
bodies = [body1, body2]

# drag state
dragging = False
drag_body: SoftBody | None = None
drag_start_mouse = Vec2(0.0, 0.0)
drag_start_center = Vec2(0.0, 0.0)

LAUNCH_STRENGTH = 6.0      # throw power
PICK_RADIUS = 100.0        # how close you must click to grab a body
MAX_DRAG_DIST = 250.0      # clamp drag length (for power and stability)


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

    # clamp target inside the inner play area (inside the border)
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


running = True
while running:
    dt = clock.tick(60) / 1000.0  # delta time in seconds

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

        # start dragging on left click
        elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
            mouse_pos = clamp_mouse()

            # find body whose center is closest to the mouse
            for body in bodies:
                center = get_body_center(body)
                if (center - mouse_pos).length() <= PICK_RADIUS:
                    dragging = True
                    drag_body = body
                    drag_start_mouse = mouse_pos
                    drag_start_center = center
                    set_body_velocity(drag_body, Vec2(0.0, 0.0))
                    break

        # release mouse → launch
        elif event.type == pygame.MOUSEBUTTONUP and event.button == 1:
            if dragging and drag_body is not None:
                # body is already clamped inside window by move_body_to
                current_center = get_body_center(drag_body)
                drag_vec = drag_start_center - current_center

                # clamp max drag length
                length = drag_vec.length()
                if length > MAX_DRAG_DIST:
                    drag_vec = drag_vec * (MAX_DRAG_DIST / length)

                launch_vel = drag_vec * LAUNCH_STRENGTH
                set_body_velocity(drag_body, launch_vel)

            # stop dragging state
            dragging = False
            drag_body = None

    # physics update
    for body in bodies:
        if dragging and body is drag_body:
            continue  # do not integrate while dragging
        body.update(
            dt,
            screen_size=(WIDTH, HEIGHT),
            border_thickness=BORDER_THICKNESS,
        )

    # soft body vs soft body collision
    softbody_collision(body1, body2)

    # while dragging, move the body to follow the clamped mouse
    if dragging and drag_body is not None:
        mouse_pos = clamp_mouse()

        drag_vec = mouse_pos - drag_start_center
        length = drag_vec.length()
        if length > MAX_DRAG_DIST:
            drag_vec = drag_vec * (MAX_DRAG_DIST / length)
        target_pos = drag_start_center + drag_vec

        move_body_to(drag_body, target_pos)
        set_body_velocity(drag_body, Vec2(0.0, 0.0))

    # -------------------------------------------------
    # Rendering
    # -------------------------------------------------
    # fill background with black
    screen.fill(BG_COLOR)

    # draw outer border (exactly on the window edge) in white
    pygame.draw.rect(
        screen,
        WALL_COLOR,
        pygame.Rect(0, 0, WIDTH, HEIGHT),
        width=BORDER_THICKNESS,
    )

    # draw soft bodies
    for body in bodies:
        outer_points = [
            (int(p.pos.x), int(p.pos.y))
            for p in body.particles[1:]
        ]

        if len(outer_points) >= 3:
            # filled jelly (gray)
            pygame.draw.polygon(screen, BALL_FILL, outer_points)
            # outline (white)
            pygame.draw.polygon(
                screen,
                BALL_OUTLINE,
                outer_points,
                width=5,
            )

    # draw drag guide line (also in white for good contrast)
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
