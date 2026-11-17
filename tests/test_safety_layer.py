
import sys
sys.path.append(".")

from controller.safety_layer import check_movement_safety


static_obstacles = {(3, 3), (4, 4)}


print("\nTEST 1: Safe movement")

result = check_movement_safety(
    (2, 2),
    (5.0, 5.0),
    static_obstacles
)

print("Result:", result)


print("\nTEST 2: Moving obstacle nearby")

result = check_movement_safety(
    (5, 5),
    (5.5, 5.0),
    static_obstacles
)

print("Result:", result)


print("\nTEST 3: Static obstacle")

result = check_movement_safety(
    (3, 3),
    (8.0, 8.0),
    static_obstacles
)

print("Result:", result)

