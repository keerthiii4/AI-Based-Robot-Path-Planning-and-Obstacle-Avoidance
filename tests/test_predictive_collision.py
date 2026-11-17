import sys
sys.path.append(".")

from controller.predictive_collision import check_predictive_collision


def run_test(name, robot, obstacle, velocity, arrival_time):

    safe, message, distance, predicted = check_predictive_collision(
        robot,
        obstacle,
        velocity,
        arrival_time
    )

    print("\n" + name)
    print("------------------------------")
    print("Robot position:", robot)
    print("Obstacle current position:", obstacle)
    print("Obstacle predicted position:", predicted)
    print("Distance at arrival:", round(distance, 2))
    print("Result:", message)


# Test 1: Head-on collision
run_test(
    "HEAD-ON MOVEMENT",
    (5, 6),
    (5, 8),
    (0, -1),
    2
)

# Test 2: Crossing movement
run_test(
    "CROSSING MOVEMENT",
    (5, 5),
    (3, 5),
    (1, 0),
    2
)

# Test 3: Parallel movement, moving away
run_test(
    "PARALLEL MOVEMENT",
    (5, 5),
    (5, 8),
    (0, 1),
    2
)
