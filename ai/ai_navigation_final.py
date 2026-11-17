
import os
import sys
import math
import heapq
import pickle
import time

# =================================================
# 1. PROJECT CONFIGURATION
# =================================================

ROWS = 11
COLS = 20

START = (1, 1)
GOAL = (10, 17)

MAX_STEPS = 100
SAFETY_RADIUS = 1.0
AI_RISK_THRESHOLD = 0.80
ARRIVAL_TIME = 1.0
DELAY = 0.3

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.path.dirname(BASE_DIR)

sys.path.insert(0, PROJECT_ROOT)

from controller.safety_layer import check_movement_safety
from controller.predictive_collision import check_predictive_collision


# =================================================
# 2. STATIC ENVIRONMENT
# =================================================

STATIC_OBSTACLES = set()

for r in range(1, 5):
    STATIC_OBSTACLES.add((r, 7))

for r in range(5, 9):
    STATIC_OBSTACLES.add((r, 14))

for c in range(1, 7):
    STATIC_OBSTACLES.add((6, c))

for c in range(4, 9):
    STATIC_OBSTACLES.add((8, c))


# =================================================
# 3. LOAD AI MODEL
# =================================================

MODEL_PATH = os.path.join(
    BASE_DIR,
    "models",
    "obstacle_risk_model.pkl"
)

with open(MODEL_PATH, "rb") as file:
    saved_model = pickle.load(file)

model = saved_model["model"]

print("AI obstacle risk model loaded successfully.")


# =================================================
# 4. A* PATH PLANNING
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
# 5. AI COLLISION PREDICTION
# =================================================

def predict_collision(
        robot,
        next_position,
        obstacle,
        obstacle_velocity):

    robot_velocity = (
        next_position[0] - robot[0],
        next_position[1] - robot[1]
    )

    rx = obstacle[0] - robot[0]
    ry = obstacle[1] - robot[1]

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
# 6. MAP DISPLAY
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
# 7. UPDATE MOVING OBSTACLE
# =================================================

def move_obstacle(obstacle, velocity):

    obstacle[0] += velocity[0]
    obstacle[1] += velocity[1]

    obstacle[0] = max(
        1,
        min(ROWS - 2, obstacle[0])
    )

    obstacle[1] = max(
        1,
        min(COLS - 2, obstacle[1])
    )


# =================================================
# 8. MAIN NAVIGATION SYSTEM
# =================================================

def main():

    robot = START

    obstacle = [5.0, 12.0]
    obstacle_velocity = (0.0, -0.5)

    blocked = set(STATIC_OBSTACLES)

    movement_steps = 0
    replans = 0
    emergency_stops = 0
    predictive_alerts = 0
    ai_alerts = 0

    path = astar(robot, GOAL, blocked)

    if not path:
        print("Initial path could not be generated.")
        return

    print("\n================================")
    print("AI-BASED ROBOT NAVIGATION")
    print("================================")
    print("Start position:", START)
    print("Goal position:", GOAL)

    while robot != GOAL and movement_steps < MAX_STEPS:

        print("\n================================")
        print("Movement step:", movement_steps + 1)
        print("Robot position:", robot)
        print("Obstacle position:", obstacle)

        # Generate a path if required
        if len(path) < 2:

            path = astar(robot, GOAL, blocked)

            if len(path) < 2:
                print("No available path.")
                break

        next_position = path[1]

        # -------------------------------------------------
        # 8A. TIME-AWARE PREDICTIVE COLLISION CHECK
        # -------------------------------------------------

        predictive_safe, predictive_message, predictive_distance, predicted_obstacle = (
            check_predictive_collision(
                next_position,
                obstacle,
                obstacle_velocity,
                ARRIVAL_TIME,
                SAFETY_RADIUS
            )
        )

        print("\nPREDICTIVE CHECK:", predictive_message)
        print("Predicted obstacle position:",
              (round(predicted_obstacle[0], 2),
               round(predicted_obstacle[1], 2)))
        print("Predicted distance:",
              round(predictive_distance, 2))

        if not predictive_safe:

            predictive_alerts += 1
            emergency_stops += 1

            print("PREDICTIVE ALERT: MOVEMENT CANCELLED")

            predicted_cell = (
                int(round(predicted_obstacle[0])),
                int(round(predicted_obstacle[1]))
            )

            if (0 <= predicted_cell[0] < ROWS and
                    0 <= predicted_cell[1] < COLS):

                blocked.add(predicted_cell)

            new_path = astar(robot, GOAL, blocked)
            replans += 1

            if len(new_path) < 2:
                print("No alternative route available.")
                break

            path = new_path

            print("Predictive replanning completed.")

            move_obstacle(obstacle, obstacle_velocity)
            movement_steps += 1

            display_map(robot, path, obstacle)
            time.sleep(DELAY)

            continue

        # -------------------------------------------------
        # 8B. INDEPENDENT SAFETY CHECK
        # -------------------------------------------------

        is_safe, safety_message, safety_distance = (
            check_movement_safety(
                next_position,
                obstacle,
                STATIC_OBSTACLES,
                safety_radius=SAFETY_RADIUS
            )
        )

        print("\nSAFETY LAYER:", safety_message)
        print("Current obstacle distance:",
              round(safety_distance, 2))

        if not is_safe:

            emergency_stops += 1

            print("EMERGENCY STOP ACTIVATED")

            obstacle_cell = (
                int(round(obstacle[0])),
                int(round(obstacle[1]))
            )

            if (0 <= obstacle_cell[0] < ROWS and
                    0 <= obstacle_cell[1] < COLS):

                blocked.add(obstacle_cell)

            new_path = astar(robot, GOAL, blocked)
            replans += 1

            if len(new_path) < 2:
                print("No safe alternative route.")
                break

            path = new_path

            move_obstacle(obstacle, obstacle_velocity)
            movement_steps += 1

            display_map(robot, path, obstacle)
            time.sleep(DELAY)

            continue

        # -------------------------------------------------
        # 8C. AI COLLISION RISK PREDICTION
        # -------------------------------------------------

        prediction, probability = predict_collision(
            robot,
            next_position,
            obstacle,
            obstacle_velocity
        )

        print("\nAI COLLISION RISK:",
              round(probability * 100, 2), "%")

        obstacle_cell = (
            int(round(obstacle[0])),
            int(round(obstacle[1]))
        )

        obstacle_on_route = obstacle_cell in path[1:]

        if (prediction == 1 and
                probability >= AI_RISK_THRESHOLD and
                obstacle_on_route):

            ai_alerts += 1

            print("AI ALERT: HIGH COLLISION RISK")

            blocked.add(obstacle_cell)

            new_path = astar(robot, GOAL, blocked)
            replans += 1

            if len(new_path) < 2:
                print("No alternative route available.")
                break

            path = new_path

            print("AI-triggered replanning completed.")

            move_obstacle(obstacle, obstacle_velocity)
            movement_steps += 1

            display_map(robot, path, obstacle)
            time.sleep(DELAY)

            continue

        elif prediction == 1:

            print("AI WARNING: Collision risk predicted.")

        else:

            print("AI STATUS: SAFE")

        # -------------------------------------------------
        # 8D. ROBOT MOVEMENT
        # -------------------------------------------------

        robot = next_position

        print("Robot moved to:", robot)

        movement_steps += 1

        move_obstacle(obstacle, obstacle_velocity)

        if robot != GOAL:
            path = astar(robot, GOAL, blocked)

        display_map(robot, path, obstacle)

        time.sleep(DELAY)

    # =================================================
    # 9. FINAL RESULTS
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
    print("Emergency stops:", emergency_stops)
    print("Predictive alerts:", predictive_alerts)
    print("AI alerts:", ai_alerts)


# =================================================
# 10. PROGRAM ENTRY POINT
# =================================================

if __name__ == "__main__":
    main()

