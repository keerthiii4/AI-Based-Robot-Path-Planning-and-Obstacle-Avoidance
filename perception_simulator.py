
import tkinter as tk
import heapq

CELL = 32
ROWS = 11
COLS = 20
SENSOR_RANGE = 2

WORLD = [
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
    "unknown": "#E8E8E8",
    "free": "#FFFFFF",
    "wall": "#344054",
    "path": "#A7D8F0",
    "robot": "#1685D1",
    "goal": "#22A06B",
    "obstacle": "#E5484D"
}


def heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


class PerceptionSimulator:
    def __init__(self, root):
        self.root = root
        self.root.title("Robot Perception and Navigation")

        self.canvas = tk.Canvas(
            root,
            width=COLS * CELL,
            height=ROWS * CELL
        )
        self.canvas.pack(padx=10, pady=10)

        controls = tk.Frame(root)
        controls.pack()

        self.start_button = tk.Button(
            controls, text="Start", command=self.start
        )
        self.start_button.pack(side=tk.LEFT, padx=5)

        tk.Button(
            controls, text="Reset", command=self.reset
        ).pack(side=tk.LEFT, padx=5)

        self.status = tk.Label(
            root, text="Grey cells are unexplored."
        )
        self.status.pack(pady=6)

        self.after_id = None
        self.reset()

    def reset(self):
        if self.after_id is not None:
            self.root.after_cancel(self.after_id)
            self.after_id = None

        self.running = False
        self.robot = START
        self.steps = 0
        self.replans = 0
        self.obstacle_added = False

        self.obstacles = set()

        # The robot initially knows almost nothing.
        self.known = [
            ["?" for c in range(COLS)]
            for r in range(ROWS)
        ]

        self.known[START[0]][START[1]] = "S"
        self.known[GOAL[0]][GOAL[1]] = "G"

        self.path = []
        self.scan()
        self.plan()

        self.start_button.config(state=tk.NORMAL)
        self.status.config(text="Ready to explore.")
        self.draw()

    def world_cell(self, pos):
        r, c = pos

        if pos in self.obstacles:
            return "X"

        if WORLD[r][c] == "#":
            return "#"

        return "."

    def scan(self):
        # Simplified proximity sensor using Manhattan distance.
        rr, rc = self.robot

        for r in range(ROWS):
            for c in range(COLS):
                distance = abs(r - rr) + abs(c - rc)

                if distance <= SENSOR_RANGE:
                    pos = (r, c)

                    if pos == START:
                        self.known[r][c] = "S"
                    elif pos == GOAL:
                        self.known[r][c] = "G"
                    else:
                        self.known[r][c] = self.world_cell(pos)

    def neighbors(self, pos):
        r, c = pos

        candidates = [
            (r - 1, c), (r + 1, c),
            (r, c - 1), (r, c + 1)
        ]

        result = []

        for nr, nc in candidates:
            if 0 <= nr < ROWS and 0 <= nc < COLS:
                # Unknown cells are assumed traversable until observed.
                if self.known[nr][nc] not in ("#", "X"):
                    result.append((nr, nc))

        return result

    def plan(self):
        frontier = []
        heapq.heappush(
            frontier, (heuristic(self.robot, GOAL), self.robot)
        )

        came_from = {}
        costs = {self.robot: 0}

        while frontier:
            priority, current = heapq.heappop(frontier)

            if current == GOAL:
                break

            for neighbor in self.neighbors(current):
                new_cost = costs[current] + 1

                if (neighbor not in costs or
                        new_cost < costs[neighbor]):

                    costs[neighbor] = new_cost
                    score = new_cost + heuristic(neighbor, GOAL)

                    heapq.heappush(frontier, (score, neighbor))
                    came_from[neighbor] = current

        if self.robot == GOAL:
            self.path = [GOAL]
            return True

        if GOAL not in came_from:
            self.path = []
            return False

        path = []
        current = GOAL

        while current != self.robot:
            path.append(current)
            current = came_from[current]

        path.append(self.robot)
        path.reverse()

        self.path = path
        return True

    def draw(self):
        self.canvas.delete("all")
        path_cells = set(self.path)

        for r in range(ROWS):
            for c in range(COLS):
                pos = (r, c)
                cell = self.known[r][c]

                color = COLORS["unknown"]

                if cell == "." or cell == "S":
                    color = COLORS["free"]
                elif cell == "#":
                    color = COLORS["wall"]
                elif cell == "X":
                    color = COLORS["obstacle"]
                elif cell == "G":
                    color = COLORS["goal"]

                if pos in path_cells and cell not in ("X", "#"):
                    color = COLORS["path"]

                x1, y1 = c * CELL, r * CELL

                self.canvas.create_rectangle(
                    x1, y1, x1 + CELL, y1 + CELL,
                    fill=color, outline="#C5CBD3"
                )

                if cell == "#":
                    self.canvas.create_text(
                        x1 + CELL / 2, y1 + CELL / 2,
                        text="#", fill="white"
                    )
                elif cell == "X":
                    self.canvas.create_text(
                        x1 + CELL / 2, y1 + CELL / 2,
                        text="X", fill="white",
                        font=("Arial", 12, "bold")
                    )
                elif cell == "G":
                    self.canvas.create_text(
                        x1 + CELL / 2, y1 + CELL / 2,
                        text="G", fill="white",
                        font=("Arial", 12, "bold")
                    )
                elif cell == "?":
                    self.canvas.create_text(
                        x1 + CELL / 2, y1 + CELL / 2,
                        text="?"
                    )

        r, c = self.robot

        self.canvas.create_oval(
            c * CELL + 5, r * CELL + 5,
            (c + 1) * CELL - 5, (r + 1) * CELL - 5,
            fill=COLORS["robot"], outline="white", width=2
        )

        self.canvas.create_text(
            c * CELL + CELL / 2, r * CELL + CELL / 2,
            text="R", fill="white",
            font=("Arial", 12, "bold")
        )

    def start(self):
        if self.running:
            return

        if self.robot == GOAL:
            self.reset()

        self.running = True
        self.start_button.config(state=tk.DISABLED)
        self.tick()

    def tick(self):
        if not self.running:
            return

        # Add an obstacle to the real world during the run.
        if self.steps == 7 and not self.obstacle_added:
            self.obstacles.add((5, 10))
            self.obstacle_added = True
            self.status.config(text="Environment changed.")
        
        # Update the robot's knowledge from local observations.
        old_known = [row[:] for row in self.known]
        self.scan()

        if self.known != old_known or self.obstacle_added:
            if not self.plan():
                self.running = False
                self.start_button.config(state=tk.NORMAL)
                self.status.config(text="No route currently available.")
                self.draw()
                return

        if self.robot == GOAL:
            self.running = False
            self.start_button.config(state=tk.NORMAL)
            self.draw()
            self.status.config(
                text="Goal reached! Steps: {0}, replans: {1}".format(
                    self.steps, self.replans
                )
            )
            return

        if len(self.path) < 2:
            self.running = False
            self.start_button.config(state=tk.NORMAL)
            self.status.config(text="No route available.")
            return

        next_pos = self.path[1]

        # Never enter a cell known to contain an obstacle.
        if self.known[next_pos[0]][next_pos[1]] in ("#", "X"):
            self.plan()
            self.replans += 1
            self.draw()
            self.after_id = self.root.after(350, self.tick)
            return

        # Safety guard for the simulated world.
        # If the sensor model misses an obstacle, stop and scan again.
        if self.world_cell(next_pos) in ("#", "X"):
            self.scan()
            self.plan()
            self.replans += 1
            self.status.config(text="Obstacle detected. Replanning.")
            self.draw()
            self.after_id = self.root.after(350, self.tick)
            return

        self.robot = next_pos
        self.steps += 1

        # Discover the surroundings from the new position.
        old_known = [row[:] for row in self.known]
        self.scan()

        if self.known != old_known:
            self.plan()
            self.replans += 1

        self.draw()

        self.status.config(
            text="Position: {0} | Steps: {1} | Replans: {2}".format(
                self.robot, self.steps, self.replans
            )
        )

        self.after_id = self.root.after(350, self.tick)


if __name__ == "__main__":
    root = tk.Tk()
    app = PerceptionSimulator(root)
    root.mainloop()


