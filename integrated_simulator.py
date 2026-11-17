
import tkinter as tk
import random
import heapq

CELL = 32
ROWS = 11
COLS = 20
SENSOR_RANGE = 2
SENSOR_ERROR = 0.20
SENSOR_SAMPLES = 5

START = (1, 1)
GOAL = (10, 17)
DYNAMIC_OBSTACLE = (5, 10)

WORLD = [
    "####################",
    "#S......#..........#",
    "#.......#..........#",
    "#......##..........#",
    "#.......#..........#",
    "#.............#....#",
    "#######.......#....#",
    "#.............#....#",
    "#...#####..........#",
    "#..................#",
    "#................G.#"
]


class SensorModel:
    def __init__(self, error_rate=0.20):
        self.error_rate = error_rate

    def read(self, true_cell):
        if random.random() < self.error_rate:
            return '#' if true_cell == '.' else '.'
        return true_cell

    def majority_vote(self, true_cell):
        readings = [
            self.read(true_cell)
            for _ in range(SENSOR_SAMPLES)
        ]
        return '#' if readings.count('#') > readings.count('.') else '.'


class RobotSimulator:
    def __init__(self, root):
        self.root = root
        self.root.title("Robot Navigation with Noisy Sensors")

        self.canvas = tk.Canvas(
            root,
            width=COLS * CELL,
            height=ROWS * CELL,
            bg="white"
        )
        self.canvas.pack()

        controls = tk.Frame(root)
        controls.pack(fill=tk.X)

        tk.Button(
            controls, text="Start",
            command=self.start
        ).pack(side=tk.LEFT, padx=5, pady=5)

        tk.Button(
            controls, text="Reset",
            command=self.reset
        ).pack(side=tk.LEFT, padx=5, pady=5)

        self.status = tk.Label(root, text="")
        self.status.pack()

        self.reset()

    def reset(self):
        self.robot = START
        self.steps = 0
        self.replans = 0
        self.running = False
        self.obstacle_added = False
        self.dynamic_obstacles = set()

        self.known = [
            ['?' for _ in range(COLS)]
            for _ in range(ROWS)
        ]

        self.known[START[0]][START[1]] = 'S'
        self.known[GOAL[0]][GOAL[1]] = 'G'

        self.sensor = SensorModel(SENSOR_ERROR)
        self.scan()
        self.path = self.plan()
        self.draw()

    def true_cell(self, pos):
        r, c = pos

        if pos in self.dynamic_obstacles:
            return '#'

        if not (0 <= r < ROWS and 0 <= c < COLS):
            return '#'

        return WORLD[r][c]

    def scan(self):
        old_known = [row[:] for row in self.known]
        r0, c0 = self.robot

        for r in range(max(0, r0 - SENSOR_RANGE),
                       min(ROWS, r0 + SENSOR_RANGE + 1)):
            for c in range(max(0, c0 - SENSOR_RANGE),
                           min(COLS, c0 + SENSOR_RANGE + 1)):

                if abs(r - r0) + abs(c - c0) > SENSOR_RANGE:
                    continue

                pos = (r, c)

                if pos == START or pos == GOAL:
                    continue

                true_value = self.true_cell(pos)

                if true_value == '#':
                    observed = self.sensor.majority_vote('#')
                else:
                    observed = self.sensor.majority_vote('.')

                self.known[r][c] = observed

        return old_known != self.known

    def neighbors(self, pos):
        r, c = pos
        candidates = [
            (r - 1, c),
            (r + 1, c),
            (r, c - 1),
            (r, c + 1)
        ]

        return [
            p for p in candidates
            if 0 <= p[0] < ROWS and 0 <= p[1] < COLS
            and self.known[p[0]][p[1]] not in ('#', 'X')
        ]

    def heuristic(self, a, b):
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def plan(self):
        frontier = [(0, self.robot)]
        came_from = {}
        cost = {self.robot: 0}

        while frontier:
            _, current = heapq.heappop(frontier)

            if current == GOAL:
                path = [current]

                while current in came_from:
                    current = came_from[current]
                    path.append(current)

                path.reverse()
                return path

            for nxt in self.neighbors(current):
                new_cost = cost[current] + 1

                if nxt not in cost or new_cost < cost[nxt]:
                    cost[nxt] = new_cost
                    priority = new_cost + self.heuristic(nxt, GOAL)
                    heapq.heappush(frontier, (priority, nxt))
                    came_from[nxt] = current

        return []

    def start(self):
        if not self.running:
            self.running = True
            self.tick()

    def tick(self):
        if not self.running:
            return

        if self.robot == GOAL:
            self.status.config(
                text="Goal reached! Steps: %d | Replans: %d"
                % (self.steps, self.replans)
            )
            self.running = False
            return

        if not self.obstacle_added and self.steps >= 7:
            self.dynamic_obstacles.add(DYNAMIC_OBSTACLE)
            self.obstacle_added = True

        changed = self.scan()

        if changed or not self.path:
            self.path = self.plan()
            self.replans += 1

        if not self.path:
            self.status.config(
                text="No safe route currently known. Scanning again..."
            )
            self.root.after(700, self.tick)
            return

        next_pos = self.path[1] if len(self.path) > 1 else self.robot

        # Safety check against the actual simulated world.
        if self.true_cell(next_pos) == '#':
            self.known[next_pos[0]][next_pos[1]] = '#'
            self.path = self.plan()
            self.replans += 1
            self.draw()
            self.root.after(350, self.tick)
            return

        self.robot = next_pos
        self.steps += 1

        self.scan()
        self.path = self.plan()
        self.draw()

        self.status.config(
            text="Steps: %d | Replans: %d | Sensor error: %d%%"
            % (self.steps, self.replans, int(SENSOR_ERROR * 100))
        )

        self.root.after(350, self.tick)

    def draw(self):
        self.canvas.delete("all")

        for r in range(ROWS):
            for c in range(COLS):
                x1 = c * CELL
                y1 = r * CELL
                x2 = x1 + CELL
                y2 = y1 + CELL

                value = self.known[r][c]
                color = "#bdbdbd"

                if value == '.':
                    color = "white"
                elif value == '#':
                    color = "#30343b"
                elif value == 'G':
                    color = "#40b86b"
                elif value == 'S':
                    color = "#d8ecff"

                if (r, c) in self.dynamic_obstacles:
                    color = "#e74c3c"

                self.canvas.create_rectangle(
                    x1, y1, x2, y2,
                    fill=color,
                    outline="#9aa0a6"
                )

                if value == '?':
                    self.canvas.create_text(
                        x1 + CELL / 2,
                        y1 + CELL / 2,
                        text="?",
                        fill="#555555"
                    )

        for pos in self.path:
            if pos != self.robot and pos != GOAL:
                r, c = pos
                x = c * CELL + CELL / 2
                y = r * CELL + CELL / 2
                self.canvas.create_oval(
                    x - 5, y - 5, x + 5, y + 5,
                    fill="#75bfff", outline=""
                )

        r, c = self.robot
        x = c * CELL + CELL / 2
        y = r * CELL + CELL / 2

        self.canvas.create_oval(
            x - 10, y - 10, x + 10, y + 10,
            fill="#1769d2", outline="white", width=2
        )


if __name__ == "__main__":
    random.seed()

    root = tk.Tk()
    app = RobotSimulator(root)
    root.mainloop()

