
import heapq
import time

GRID = [
    "....................",
    ".S......#...........",
    "........#...........",
    "...##...#...........",
    "........#.....###...",
    "..............#.....",
    ".######.......#.....",
    "..............#.....",
    "....#####...........",
    "....................",
    ".................G.."
]

DYNAMIC_OBSTACLES = set()


def find_position(symbol):
    for row in range(len(GRID)):
        for col in range(len(GRID[row])):
            if GRID[row][col] == symbol:
                return (row, col)
    return None


def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def get_neighbors(position):
    row, col = position

    candidates = [
        (row - 1, col),
        (row + 1, col),
        (row, col - 1),
        (row, col + 1)
    ]

    neighbors = []

    for r, c in candidates:
        if 0 <= r < len(GRID) and 0 <= c < len(GRID[0]):
            if GRID[r][c] != "#" and (r, c) not in DYNAMIC_OBSTACLES:
                neighbors.append((r, c))

    return neighbors


def a_star(start, goal):
    frontier = []
    heapq.heappush(frontier, (heuristic(start, goal), start))

    came_from = {}
    cost_so_far = {start: 0}

    while frontier:
        priority, current = heapq.heappop(frontier)

        if current == goal:
            break

        for neighbor in get_neighbors(current):
            new_cost = cost_so_far[current] + 1

            if (neighbor not in cost_so_far or
                    new_cost < cost_so_far[neighbor]):

                cost_so_far[neighbor] = new_cost
                priority = new_cost + heuristic(neighbor, goal)

                heapq.heappush(frontier, (priority, neighbor))
                came_from[neighbor] = current

    if goal not in came_from and start != goal:
        return None

    path = []
    current = goal

    while current != start:
        path.append(current)
        current = came_from[current]

    path.append(start)
    path.reverse()

    return path


def display_map(robot_position, path):
    display = [list(row) for row in GRID]

    for row, col in DYNAMIC_OBSTACLES:
        display[row][col] = "X"

    if path:
        for row, col in path:
            if display[row][col] not in ("S", "G", "X"):
                display[row][col] = "*"

    row, col = robot_position
    display[row][col] = "R"

    print("\nRobot position:", robot_position)

    for line in display:
        print("".join(line))


def main():
    start = find_position("S")
    goal = find_position("G")
    robot = start

    print("ROBOT NAVIGATION SIMULATOR")
    print("R = Robot, X = Dynamic obstacle")

    path = a_star(robot, goal)

    if path is None:
        print("No path to the goal.")
        return

    step = 0
    obstacle_added = False
    total_replans = 0

    while robot != goal:
        display_map(robot, path)
        time.sleep(0.5)

        # Introduce an obstacle during the simulation.
        if step == 7 and not obstacle_added:
            obstacle = (5, 10)
            DYNAMIC_OBSTACLES.add(obstacle)
            obstacle_added = True

            print("\nA new obstacle appeared at:", obstacle)

        # Replan whenever the remaining path is blocked.
        if any(p in DYNAMIC_OBSTACLES for p in path
               if p != robot):
            path = a_star(robot, goal)
            total_replans += 1

            if path is None:
                print("No safe path remains.")
                return

            print("New path calculated.")

        if len(path) < 2:
            print("Path ended before reaching the goal.")
            return

        next_position = path[1]

        # Safety check immediately before moving.
        if next_position in DYNAMIC_OBSTACLES:
            print("Obstacle detected. Replanning.")
            path = a_star(robot, goal)
            total_replans += 1

            if path is None or len(path) < 2:
                print("Cannot continue safely.")
                return

            next_position = path[1]

        robot = next_position
        step += 1
        path = path[1:]

    display_map(robot, [])
    print("\nGoal reached!")
    print("Total movement steps:", step)
    print("Replanning operations:", total_replans)


if __name__ == "__main__":
    main()

