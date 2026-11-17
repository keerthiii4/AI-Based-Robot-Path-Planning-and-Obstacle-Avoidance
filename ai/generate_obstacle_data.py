
import numpy as np
import csv
import random
import os
import math

random.seed(42)
np.random.seed(42)

SAMPLES = 5000
SAFE_DISTANCE = 1.0
PREDICTION_TIME = 3.0

output_dir = "ai/datasets"

if not os.path.exists(output_dir):
    os.makedirs(output_dir)

output_file = os.path.join(
    output_dir, "obstacle_data.csv"
)

with open(output_file, "w") as file:
    writer = csv.writer(file)

    writer.writerow([
        "distance",
        "closing_speed",
        "time_to_closest",
        "closest_distance",
        "collision_risk"
    ])

    for i in range(SAMPLES):

        # Relative position of obstacle from robot
        x = random.uniform(-10, 10)
        y = random.uniform(-10, 10)

        # Relative velocity of obstacle
        vx = random.uniform(-3, 3)
        vy = random.uniform(-3, 3)

        distance = math.sqrt(x*x + y*y)

        if distance < 0.01:
            distance = 0.01

        speed = math.sqrt(vx*vx + vy*vy)

        # Rate at which distance is decreasing
        closing_speed = -(x*vx + y*vy) / distance

        if speed > 0.01:
            t_closest = -(x*vx + y*vy) / (speed*speed)
        else:
            t_closest = 0.0

        # Clamp closest-approach time to future
        if t_closest < 0:
            t_closest = 0.0

        closest_x = x + vx*t_closest
        closest_y = y + vy*t_closest

        closest_distance = math.sqrt(
            closest_x*closest_x +
            closest_y*closest_y
        )

        risk = 0

        if (0 <= t_closest <= PREDICTION_TIME
                and closest_distance <= SAFE_DISTANCE):
            risk = 1

        writer.writerow([
            round(distance, 3),
            round(closing_speed, 3),
            round(t_closest, 3),
            round(closest_distance, 3),
            risk
        ])

print("Dataset generated successfully.")
print("Total samples:", SAMPLES)
print("Saved to:", output_file)

