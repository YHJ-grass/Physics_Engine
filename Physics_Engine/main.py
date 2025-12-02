# main.py
import random
import pygame
from engine import Vec2, RigidBodyCircle, step, apply_impulse

WIDTH, HEIGHT = 800, 600

# Light green background
BG_COLOR = (180, 220, 180)

# Default fallback colors (used if a body has no custom color)
CIRCLE_COLOR = (230, 230, 240)
BOX_COLOR = (200, 240, 200)
TRI_COLOR = (240, 200, 200)

# Colors for outlines and aim line
OUTLINE_COLOR = (20, 20, 20)   # dark outline for shapes
BORDER_COLOR = (0, 0, 0)       # map border (screen edge)
AIM_LINE_COLOR = (40, 40, 40)  # dark line to contrast with light background

# Scale factor from drag distance to impulse strength
LAUNCH_IMPULSE_SCALE = 12.0


def random_pastel_color():
    """Returns a random soft (pastel-like) RGB color."""
    r = random.randint(120, 255)
    g = random.randint(120, 255)
    b = random.randint(120, 255)
    return (r, g, b)


def create_bodies_random(count=12):
    """
    Creates 'count' bodies with:
    - random positions
    - random shapes
    - random colors
    """
    bodies = []
    margin = 60
    shapes = ["circle", "box", "triangle"]

    for _ in range(count):
        x = random.uniform(margin, WIDTH - margin)
        y = random.uniform(margin, HEIGHT - margin)
        radius = random.randint(18, 26)
        shape = random.choice(shapes)
        mass = radius * radius * 0.05

        body = RigidBodyCircle(
            position=Vec2(x, y),
            radius=radius,
            mass=mass,
            restitution=0.8,
            shape=shape
        )

        # Attach a random color to this body (visual only)
        body.color = random_pastel_color()

        bodies.append(body)

    return bodies


def draw_body(screen, body: RigidBodyCircle):
    """Renders a body with a fill color and a dark outline."""
    x = int(body.position.x)
    y = int(body.position.y)
    r = int(body.radius)

    # Use per-body color if it exists
    fill_color = getattr(body, "color", None)
    if fill_color is None:
        # Fallback per shape (should not happen if we always set body.color)
        if body.shape == "circle":
            fill_color = CIRCLE_COLOR
        elif body.shape == "box":
            fill_color = BOX_COLOR
        elif body.shape == "triangle":
            fill_color = TRI_COLOR
        else:
            fill_color = CIRCLE_COLOR

    if body.shape == "circle":
        # Filled circle
        pygame.draw.circle(screen, fill_color, (x, y), r)
        # Outline
        pygame.draw.circle(screen, OUTLINE_COLOR, (x, y), r, width=2)

    elif body.shape == "box":
        rect = pygame.Rect(x - r, y - r, 2 * r, 2 * r)
        # Filled box (with slightly rounded corners)
        pygame.draw.rect(screen, fill_color, rect, border_radius=4)
        # Outline
        pygame.draw.rect(screen, OUTLINE_COLOR, rect, width=2, border_radius=4)

    elif body.shape == "triangle":
        points = [
            (x,     y - r),
            (x - r, y + r),
            (x + r, y + r)
        ]
        # Filled triangle
        pygame.draw.polygon(screen, fill_color, points)
        # Outline
        pygame.draw.polygon(screen, OUTLINE_COLOR, points, width=2)

    else:
        # Fallback: circle
        pygame.draw.circle(screen, fill_color, (x, y), r)
        pygame.draw.circle(screen, OUTLINE_COLOR, (x, y), r, width=2)


def main():
    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("Physics demo: random shapes on green board")
    clock = pygame.time.Clock()

    # Random positions, shapes, and colors
    bodies = create_bodies_random(count=12)

    selected_body = None
    dragging = False
    drag_start = None
    current_mouse = None

    running = True
    while running:
        dt = clock.tick(60) / 1000.0

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

            # Mouse down: select a body
            if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                mx, my = event.pos
                mouse_pos = Vec2(mx, my)
                for body in reversed(bodies):
                    if (body.position - mouse_pos).length() <= body.radius:
                        selected_body = body
                        selected_body.locked = True
                        dragging = True
                        drag_start = mouse_pos
                        current_mouse = mouse_pos
                        break

            # Update drag position
            if event.type == pygame.MOUSEMOTION and dragging:
                mx, my = event.pos
                current_mouse = Vec2(mx, my)

            # Mouse up: apply impulse and release
            if event.type == pygame.MOUSEBUTTONUP and event.button == 1:
                if dragging and selected_body is not None and current_mouse is not None:
                    # Drag vector from mouse to body center
                    drag_vec = selected_body.position - current_mouse
                    impulse = drag_vec * LAUNCH_IMPULSE_SCALE
                    apply_impulse(selected_body, impulse)
                    selected_body.locked = False
                dragging = False
                selected_body = None
                drag_start = None
                current_mouse = None

        # Physics update
        step(bodies, dt, WIDTH, HEIGHT)

        # Rendering
        screen.fill(BG_COLOR)

        # Map border (screen edge outline)
        border_rect = pygame.Rect(0, 0, WIDTH - 1, HEIGHT - 1)
        pygame.draw.rect(screen, BORDER_COLOR, border_rect, width=3)

        # Draw bodies
        for body in bodies:
            draw_body(screen, body)

        # Draw aim line while dragging
        if dragging and selected_body is not None and current_mouse is not None:
            sx = int(selected_body.position.x)
            sy = int(selected_body.position.y)
            ex = int(current_mouse.x)
            ey = int(current_mouse.y)
            pygame.draw.line(screen, AIM_LINE_COLOR, (sx, sy), (ex, ey), 2)

        pygame.display.flip()

    pygame.quit()


if __name__ == "__main__":
    main()
