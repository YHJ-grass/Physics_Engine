import pygame
from engine import Vec2, RigidBodyCircle, step

WIDTH, HEIGHT = 800, 600

BG_COLOR = (10, 10, 30)
CIRCLE_COLOR = (230, 230, 240)
BOX_COLOR = (200, 240, 200)
TRI_COLOR = (240, 200, 200)
AIM_LINE_COLOR = (255, 255, 0)

# Controls the velocity applied when releasing a dragged body
LAUNCH_STRENGTH = 6.0


def create_bodies_grid():
    """Creates a 3x4 grid of objects with mixed shapes."""
    bodies = []
    rows, cols = 3, 4
    start_x = 150
    start_y = 150
    gap_x = 130
    gap_y = 100
    
    shapes = ["circle", "box", "triangle", "circle"]
    
    for r in range(rows):
        for c in range(cols):
            x = start_x + c * gap_x
            y = start_y + r * gap_y
            radius = 20
            shape = shapes[c % len(shapes)]
            mass = radius * radius * 0.05
            body = RigidBodyCircle(Vec2(x, y), radius,
                                   mass=mass,
                                   restitution=0.8,
                                   shape=shape)
            bodies.append(body)
    
    return bodies


def draw_body(screen, body: RigidBodyCircle):
    """Draws each rigid body depending on its visual shape type."""
    x = int(body.position.x)
    y = int(body.position.y)
    r = int(body.radius)
    
    if body.shape == "circle":
        pygame.draw.circle(screen, CIRCLE_COLOR, (x, y), r)
    elif body.shape == "box":
        rect = pygame.Rect(x - r, y - r, 2 * r, 2 * r)
        pygame.draw.rect(screen, BOX_COLOR, rect)
    elif body.shape == "triangle":
        points = [(x, y - r),
                  (x - r, y + r),
                  (x + r, y + r)]
        pygame.draw.polygon(screen, TRI_COLOR, points)
    else:
        pygame.draw.circle(screen, CIRCLE_COLOR, (x, y), r)


def main():
    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("Physics demo: top-down, drag to launch")
    clock = pygame.time.Clock()
    
    bodies = create_bodies_grid()
    
    selected_body = None
    dragging = False
    current_mouse = None
    
    running = True
    while running:
        dt = clock.tick(60) / 1000.0
        
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            
            # Select body on mouse press
            if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                mx, my = event.pos
                mouse_pos = Vec2(mx, my)
                for body in reversed(bodies):
                    if (body.position - mouse_pos).length() <= body.radius:
                        selected_body = body
                        selected_body.locked = True
                        dragging = True
                        current_mouse = mouse_pos
                        break
            
            # While dragging, update the mouse position
            if event.type == pygame.MOUSEMOTION and dragging:
                mx, my = event.pos
                current_mouse = Vec2(mx, my)
            
            # Release drag → launch the body
            if event.type == pygame.MOUSEBUTTONUP and event.button == 1:
                if dragging and selected_body is not None and current_mouse is not None:
                    direction = selected_body.position - current_mouse
                    selected_body.velocity = direction * LAUNCH_STRENGTH
                    selected_body.locked = False
                dragging = False
                selected_body = None
                current_mouse = None
        
        # Physics update (X–Y plane + Z gravity inside the engine)
        step(bodies, dt, WIDTH, HEIGHT)
        
        # Drawing (top-down, only X–Y is visible)
        screen.fill(BG_COLOR)
        
        for body in bodies:
            draw_body(screen, body)
        
        # Draw aiming line while dragging
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