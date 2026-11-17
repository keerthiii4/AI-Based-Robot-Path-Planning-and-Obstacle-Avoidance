import os
import math
import heapq
import pickle
import time


# =================================================
# 1. GRID CONFIGURATION
# =================================================

ROWS = 11
COLS = 20

START = (1, 1)
GOAL = (10, 17)

STATIC_OBSTACLES = set()

# Vertical wall
for r in range(1, 5):
    STATIC_OBSTACLES.add((r, 7))

# Second vertical wall
for r in range(5, 9):
    STATIC_OBSTACLES.add((r, 14))

# Horizontal wall
for c in range(1, 7):
    STATIC_OBSTACLES.add((6, c))

# Additional wall
for c in range(4, 9):
    STATIC_OBSTACLES.add((8, c))


# =================================================
# 2. LOAD TRAINED AI MODEL
# =================================================

BASE_DIR = os.path.dirname(os.path.abspath(__file__))

MODEL_PATH = os.path.join(
    BASE_DIR,
    "models",
    "obstacle_risk_model.pkl"
)

with open(MODEL_PATH, "rb") as file:
    saved_model = pickle.load(file)

# The pickle file contains a dictionary.
# Extract the trained Random Forest classifier.
model = saved_model["model"]

print("AI obstacle risk model loaded successfully.")


# =================================================
# 3. A* PATH PLANNING
# =================================================

def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def astar(start, goal, blocked):

    open_list = []
    heapq.heappush(open_list, (0, start))

    came_from = {}
    g_score = {start: 0}

    while open_list:

        current = heapq.heappop(open_list)[1]

        if current == goal:

            path = []

            while current in came_from:
                path.append(current)
                current = came_from[current]

            path.append(start)
            path.reverse()

            return path

        neighbors = [
            (current[0] - 1, current[1]),
            (current[0] + 1, current[1]),
            (current[0], current[1] - 1),
            (current[0], current[1] + 1)
        ]

        for neighbor in neighbors:

            r, c = neighbor

            if r < 0 or r >= ROWS:
                continue

            if c < 0 or c >= COLS:
                continue

            if neighbor in blocked:
                continue

            tentative_g = g_score[current] + 1

            if (neighbor not in g_score or
                    tentative_g < g_score[neighbor]):

                came_from[neighbor] = current
                g_score[neighbor] = tentative_g

                f_score = (
                    tentative_g +
                    heuristic(neighbor, goal)
                )

                heapq.heappush(
                    open_list,
                    (f_score, neighbor)
                )

    return []


# =================================================
# 4. AI COLLISION RISK PREDICTION
# =================================================

def predict_collision(robot, next_position,
                      obstacle, obstacle_velocity):

    # Robot's intended movement
    robot_velocity = (
        next_position[0] - robot[0],
        next_position[1] - robot[1]
    )

    # Relative position
    rx = obstacle[0] - robot[0]
    ry = obstacle[1] - robot[1]

    # Relative velocity
    vx = obstacle_velocity[0] - robot_velocity[0]
    vy = obstacle_velocity[1] - robot_velocity[1]

    distance = math.sqrt(rx * rx + ry * ry)

    relative_speed_squared = vx * vx + vy * vy

    if distance == 0:
        closing_speed = 0
    else:
        closing_speed = -(
            rx * vx + ry * vy
        ) / distance

    if relative_speed_squared == 0:
        time_to_closest = 0
    else:
        time_to_closest = -(
            rx * vx + ry * vy
        ) / relative_speed_squared

    time_to_closest = max(0, time_to_closest)

    closest_x = rx + vx * time_to_closest
    closest_y = ry + vy * time_to_closest

    closest_distance = math.sqrt(
        closest_x * closest_x +
        closest_y * closest_y
    )

    features = [[
        distance,
        closing_speed,
        time_to_closest,
        closest_distance
    ]]

    prediction = model.predict(features)[0]

    probabilities = model.predict_proba(features)[0]

    risk_index = list(model.classes_).index(1)

    risk_probability = probabilities[risk_index]

    return prediction, risk_probability


# =================================================
# 5. DISPLAY SIMULATION MAP
# =================================================

def display_map(robot, path, obstacle):

    path_set = set(path)

    obstacle_cell = (
        int(round(obstacle[0])),
        int(round(obstacle[1]))
    )

    print("\nROBOT NAVIGATION MAP")
    print("--------------------")

    for r in range(ROWS):

        line = ""

        for c in range(COLS):

            position = (r, c)

            if position == robot:
                line += "R"

            elif position == GOAL:
                line += "G"

            elif position == obstacle_cell:
                line += "X"

            elif position in STATIC_OBSTACLES:
                line += "#"

            elif position in path_set:
                line += "*"

            else:
                line += "."

        print(line)

    print("--------------------")


# =================================================
# 6. MAIN NAVIGATION SIMULATION
# =================================================

def main():

    robot = START

    obstacle = [5.0, 12.0]

    obstacle_velocity = (0.0, -0.5)

    blocked = set(STATIC_OBSTACLES)

    replans = 0
    movement_steps = 0

    MAX_STEPS = 100

    path = astar(robot, GOAL, blocked)

    if not path:
        print("Initial path could not be generated.")
        return

    print("\nStarting AI-assisted robot navigation...")
    print("Start:", START)
    print("Goal:", GOAL)

    while robot != GOAL and movement_steps < MAX_STEPS:

        print("\n================================")
        print("Movement step:", movement_steps + 1)
        print("Robot position:", robot)

        print(
            "Obstacle position:",
            (round(obstacle[0], 2),
             round(obstacle[1], 2))
        )

        # Ensure a valid route exists
        if len(path) < 2:

            path = astar(robot, GOAL, blocked)

            if len(path) < 2:
                print("No available path to the goal.")
                break

        next_position = path[1]

        # AI collision-risk prediction
        prediction, probability = predict_collision(
            robot,
            next_position,
            obstacle,
            obstacle_velocity
        )

        print(
            "AI collision risk:",
            round(probability * 100, 2),
            "%"
        )

        predicted_cell = (
            int(round(obstacle[0])),
            int(round(obstacle[1]))
        )

        obstacle_on_route = predicted_cell in path[1:]

        # Replan only when high risk is predicted
        # and the obstacle is on the planned route.
        if (prediction == 1 and
                probability >= 0.80 and
                obstacle_on_route):

            print("AI ALERT: COLLISION RISK DETECTED!")
            print("Obstacle is on the planned route.")

            if (0 <= predicted_cell[0] < ROWS and
                    0 <= predicted_cell[1] < COLS):

                blocked.add(predicted_cell)

            new_path = astar(robot, GOAL, blocked)

            replans += 1

            if len(new_path) < 2:
                print("No alternative route available.")
                break

            path = new_path

            print("Alternative route generated.")

            robot = path[1]

        else:

            if prediction == 1:
                print(
                    "AI WARNING: Risk predicted, "
                    "but obstacle is not blocking the route."
                )
            else:
                print("AI STATUS: SAFE")

            robot = next_position

        movement_steps += 1

        # Move the simulated obstacle
        obstacle[0] += obstacle_velocity[0]
        obstacle[1] += obstacle_velocity[1]

        # Keep obstacle inside the grid
        obstacle[0] = max(
            1,
            min(ROWS - 2, obstacle[0])
        )

        obstacle[1] = max(
            1,
            min(COLS - 2, obstacle[1])
        )

        # Recalculate route from the robot's new position.
        # This prevents the robot from repeatedly selecting
        # the same next cell.
        if robot != GOAL:
            path = astar(robot, GOAL, blocked)

        display_map(robot, path, obstacle)

        time.sleep(0.3)

    # =================================================
    # 7. FINAL RESULTS
    # =================================================

    print("\n================================")
    print("NAVIGATION RESULTS")
    print("================================")

    if robot == GOAL:
        print("SUCCESS: Robot reached the goal!")
    else:
        print("Navigation stopped before reaching the goal.")

    print("Final robot position:", robot)
    print("Total movement steps:", movement_steps)
    print("Replanning operations:", replans)


# =================================================
# 8. PROGRAM ENTRY POINT
# =================================================

if __name__ == "__main__":
    main()
