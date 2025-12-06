# main.py

import math
import random
import pygame

from engine import Vec2, RigidBodyCircle, step, apply_impulse


WIDTH, HEIGHT = 1200, 800

GROUND_Y = HEIGHT - 1

BG_COLOR = (0, 0, 0)               # black background
WALL_COLOR = (255, 255, 255)       # white walls
OUTLINE_COLOR = (255, 255, 255)    # white outline for jelly
FILL_GRAY = (180, 180, 180)        # gray fill
AIM_LINE_COLOR = (255, 255, 255)   # white aim line

TICK = 60


def create_balls() -> list[RigidBodyCircle]:
    balls: list[RigidBodyCircle] = []

    center_y = HEIGHT * 0.5

    positions = [
        Vec2(WIDTH * 0.3, center_y),
        Vec2(WIDTH * 0.5, center_y),
        Vec2(WIDTH * 0.7, center_y),
    ]

    for pos in positions:
        ball = RigidBodyCircle(
            position=pos,
            radius=80.0,
            mass=1.0,
            restitution=0.9,
            shape="circle",
            resolution=10,
        )
        balls.append(ball)

    return balls


def spawn_ball_at(balls: list[RigidBodyCircle], pos: Vec2) -> None:
    ball = RigidBodyCircle(
        position=pos,
        radius=80.0,
        mass=1.0,
        restitution=0.9,
        shape="circle",
        resolution=10,
    )
    balls.append(ball)


def spawn_explosion_fragments(balls: list[RigidBodyCircle],
                              center: Vec2,
                              base_radius: float) -> None:
    num_fragments = 6
    fragment_radius = base_radius * 0.3
    fragment_mass = 0.3

    for _ in range(num_fragments):
        fragment = RigidBodyCircle(
            position=center,
            radius=fragment_radius,
            mass=fragment_mass,
            restitution=0.9,
            shape="circle",
            resolution=8,
        )

        angle = random.uniform(0.0, 2.0 * math.pi)
        impulse_strength = 10.0
        impulse = Vec2(
            math.cos(angle) * impulse_strength,
            math.sin(angle) * impulse_strength,
        )
        apply_impulse(fragment, impulse)

        balls.append(fragment)


def pick_ball(balls: list[RigidBodyCircle], mouse: Vec2) -> RigidBodyCircle | None:
    for ball in balls:
        if (ball.position - mouse).length() <= ball.radius:
            return ball
    return None


def move_ball_to_mouse(ball: RigidBodyCircle, mouse: Vec2) -> None:
    offset_center = ball.original_center

    for v in ball.vertices:
        local_offset = v.original_pos - offset_center
        v.pos = mouse + local_offset
        v.vel = Vec2(0.0, 0.0)

    ball.position = mouse
    ball.velocity = Vec2(0.0, 0.0)


def draw_ball(screen: pygame.Surface, ball: RigidBodyCircle) -> None:
    if len(ball.vertices) < 3:
        return

    points = [(int(v.pos.x), int(v.pos.y)) for v in ball.vertices]

    pygame.draw.polygon(screen, FILL_GRAY, points)
    pygame.draw.polygon(screen, OUTLINE_COLOR, points, width=2)


def main() -> None:
    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("Soft-body (jelly) on my rigid engine")
    clock = pygame.time.Clock()

    balls: list[RigidBodyCircle] = create_balls()

    mouse_down = False      
    active_ball: RigidBodyCircle | None = None
    mouse_pos = Vec2(0.0, 0.0)

    running = True
    while running:
        dt = clock.tick(TICK) / 1000.0

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

            elif event.type == pygame.MOUSEBUTTONDOWN:
                mx, my = event.pos
                mouse_pos = Vec2(float(mx), float(my))

                if event.button == 1:
                    picked = pick_ball(balls, mouse_pos)
                    if picked is not None:
                        active_ball = picked
                        mouse_down = True

                elif event.button == 3:
                    spawn_ball_at(balls, mouse_pos)

            elif event.type == pygame.MOUSEBUTTONUP and event.button == 1:
                mouse_down = False
                active_ball = None

            elif event.type == pygame.MOUSEMOTION:
                if mouse_down:
                    mx, my = event.pos
                    mouse_pos = Vec2(float(mx), float(my))


        if mouse_down and active_ball is not None:
            move_ball_to_mouse(active_ball, mouse_pos)

        step(balls, dt, WIDTH, HEIGHT)

        exploded_balls: list[RigidBodyCircle] = [b for b in balls if b.exploded]

        if exploded_balls:
            for b in exploded_balls:
                if b in balls:
                    balls.remove(b)
                spawn_explosion_fragments(balls, b.position, b.radius)

        screen.fill(BG_COLOR)

        pygame.draw.line(
            screen,
            WALL_COLOR,
            (0, GROUND_Y),
            (WIDTH, GROUND_Y),
            width=2,
        )

        pygame.draw.line(
            screen,
            WALL_COLOR,
            (0, 0),
            (WIDTH, 0),
            width=2,
        )

        pygame.draw.line(
            screen,
            WALL_COLOR,
            (0, 0),
            (0, GROUND_Y),
            width=2,
        )

        pygame.draw.line(
            screen,
            WALL_COLOR,
            (WIDTH - 1, 0),
            (WIDTH - 1, GROUND_Y),
            width=2,
        )

        for ball in balls:
            draw_ball(screen, ball)

        pygame.display.flip()

    pygame.quit()


if __name__ == "__main__":
    main()
