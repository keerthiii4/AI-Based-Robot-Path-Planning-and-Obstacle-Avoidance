
import tkinter as tk
import heapq

CELL = 32
ROWS = 11
COLS = 20

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

START = (1, 1)
GOAL = (10, 17)

COLORS = {
    "free": "#F4F7FB",
    "wall": "#344054",
    "path": "#A7D8F0",
    "robot": "#1685D1",
    "goal": "#22A06B",
    "obstacle": "#E5484D",
    "grid": "#D0D5DD"
}


def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def get_neighbors(position, obstacles):
    row, col = position

    candidates = [
        (row - 1, col),
        (row + 1, col),
        (row, col - 1),
        (row, col + 1)
    ]

    result = []

    for r, c in candidates:
        if 0 <= r < ROWS and 0 <= c < COLS:
            if GRID[r][c] != "#" and (r, c) not in obstacles:
                result.append((r, c))

    return result


def a_star(start, goal, obstacles):
    frontier = []
    heapq.heappush(frontier, (heuristic(start, goal), start))

    came_from = {}
    cost_so_far = {start: 0}

    while frontier:
        priority, current = heapq.heappop(frontier)

        if current == goal:
            break

        for neighbor in get_neighbors(current, obstacles):
            new_cost = cost_so_far[current] + 1

            if (neighbor not in cost_so_far or
                    new_cost < cost_so_far[neighbor]):

                cost_so_far[neighbor] = new_cost
                priority = new_cost + heuristic(neighbor, goal)

                heapq.heappush(frontier, (priority, neighbor))
                came_from[neighbor] = current

    if start == goal:
        return [start]

    if goal not in came_from:
        return None

    path = []
    current = goal

    while current != start:
        path.append(current)
        current = came_from[current]

    path.append(start)
    path.reverse()

    return path


class RobotSimulator:
    def __init__(self, root):
        self.root = root
        self.root.title("AI Robot Path Planning and Obstacle Avoidance")

        self.canvas = tk.Canvas(
            root,
            width=COLS * CELL,
            height=ROWS * CELL,
            bg="white"
        )
        self.canvas.pack(padx=10, pady=10)

        controls = tk.Frame(root)
        controls.pack(pady=5)

        self.start_button = tk.Button(
            controls,
            text="Start Simulation",
            command=self.start
        )
        self.start_button.pack(side=tk.LEFT, padx=5)

        self.reset_button = tk.Button(
            controls,
            text="Reset",
            command=self.reset
        )
        self.reset_button.pack(side=tk.LEFT, padx=5)

        self.status = tk.Label(
            root,
            text="Ready. Press Start Simulation.",
            font=("Arial", 11)
        )
        self.status.pack(pady=5)

        self.robot = START
        self.obstacles = set()
        self.path = []
        self.steps = 0
        self.replans = 0
        self.running = False
        self.obstacle_added = False
        self.after_id = None

        self.reset()

    def reset(self):
        if self.after_id is not None:
            self.root.after_cancel(self.after_id)
            self.after_id = None

        self.running = False
        self.robot = START
        self.obstacles = set()
        self.path = a_star(self.robot, GOAL, self.obstacles)
        self.steps = 0
        self.replans = 0
        self.obstacle_added = False

        self.start_button.config(state=tk.NORMAL)
        self.status.config(text="Ready. Press Start Simulation.")
        self.draw()

    def draw(self):
        self.canvas.delete("all")

        path_cells = set(self.path or [])

        for r in range(ROWS):
            for c in range(COLS):
                x1 = c * CELL
                y1 = r * CELL
                x2 = x1 + CELL
                y2 = y1 + CELL
                pos = (r, c)

                color = COLORS["free"]

                if GRID[r][c] == "#":
                    color = COLORS["wall"]
                elif pos in path_cells:
                    color = COLORS["path"]

                if pos in self.obstacles:
                    color = COLORS["obstacle"]

                if pos == GOAL:
                    color = COLORS["goal"]

                self.canvas.create_rectangle(
                    x1, y1, x2, y2,
                    fill=color,
                    outline=COLORS["grid"]
                )

                if GRID[r][c] == "#":
                    self.canvas.create_text(
                        x1 + CELL / 2,
                        y1 + CELL / 2,
                        text="#",
                        fill="white"
                    )

                if pos in self.obstacles:
                    self.canvas.create_text(
                        x1 + CELL / 2,
                        y1 + CELL / 2,
                        text="X",
                        fill="white",
                        font=("Arial", 12, "bold")
                    )

        # Draw the goal marker.
        gr, gc = GOAL
        self.canvas.create_text(
            gc * CELL + CELL / 2,
            gr * CELL + CELL / 2,
            text="G",
            fill="white",
            font=("Arial", 13, "bold")
        )

        # Draw the robot last so it stays visible.
        rr, rc = self.robot
        margin = 5

        self.canvas.create_oval(
            rc * CELL + margin,
            rr * CELL + margin,
            (rc + 1) * CELL - margin,
            (rr + 1) * CELL - margin,
            fill=COLORS["robot"],
            outline="white",
            width=2
        )

        self.canvas.create_text(
            rc * CELL + CELL / 2,
            rr * CELL + CELL / 2,
            text="R",
            fill="white",
            font=("Arial", 12, "bold")
        )

    def start(self):
        if self.running:
            return

        if self.robot == GOAL:
            self.reset()

        self.running = True
        self.start_button.config(state=tk.DISABLED)
        self.status.config(text="Robot is navigating...")
        self.tick()

    def tick(self):
        if not self.running:
            return

        if self.robot == GOAL:
            self.running = False
            self.start_button.config(state=tk.NORMAL)
            self.status.config(
                text="Goal reached! Steps: {0} | Replans: {1}".format(
                    self.steps, self.replans
                )
            )
            self.draw()
            return

        # Introduce an obstacle after several movement steps.
        if self.steps == 7 and not self.obstacle_added:
            obstacle = (5, 10)

            if obstacle != self.robot:
                self.obstacles.add(obstacle)

            self.obstacle_added = True
            self.path = a_star(self.robot, GOAL, self.obstacles)
            self.replans += 1

            self.status.config(
                text="New obstacle detected. Recalculating route..."
            )

            if self.path is None:
                self.running = False
                self.start_button.config(state=tk.NORMAL)
                self.status.config(text="No route available.")
                self.draw()
                return

        # Recalculate if an obstacle blocks the remaining path.
        if self.path is None or any(
                pos in self.obstacles for pos in self.path[1:]):
            self.path = a_star(self.robot, GOAL, self.obstacles)
            self.replans += 1

        if self.path is None or len(self.path) < 2:
            self.running = False
            self.start_button.config(state=tk.NORMAL)
            self.status.config(text="Navigation stopped: no safe path.")
            self.draw()
            return

        next_position = self.path[1]

        # Final collision check before each movement.
        if next_position in self.obstacles:
            self.path = a_star(self.robot, GOAL, self.obstacles)
            self.replans += 1

            if self.path is None or len(self.path) < 2:
                self.running = False
                self.start_button.config(state=tk.NORMAL)
                self.status.config(text="Navigation stopped safely.")
                self.draw()
                return

            next_position = self.path[1]

        self.robot = next_position
        self.path = self.path[1:]
        self.steps += 1

        self.draw()

        self.status.config(
            text="Position: {0} | Steps: {1} | Replans: {2}".format(
                self.robot, self.steps, self.replans
            )
        )

        self.after_id = self.root.after(350, self.tick)


if __name__ == "__main__":
    root = tk.Tk()
    app = RobotSimulator(root)
    root.mainloop()


