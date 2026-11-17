import math


def check_movement_safety(
        next_position,
        obstacle_position,
        static_obstacles,
        safety_radius=1.0):

    # Check static obstacles
    if next_position in static_obstacles:
        return False, "STATIC OBSTACLE DETECTED", 0

    # Calculate distance from robot's next position
    # to the moving obstacle
    dx = next_position[0] - obstacle_position[0]
    dy = next_position[1] - obstacle_position[1]

    distance = math.sqrt(dx * dx + dy * dy)

    # Check safety distance
    if distance <= safety_radius:
        return False, "EMERGENCY STOP: OBSTACLE TOO CLOSE", distance

    return True, "MOVEMENT SAFE", distance
