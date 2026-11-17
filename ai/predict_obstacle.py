
import pickle
import math
import os

MODEL_FILE = "ai/models/obstacle_risk_model.pkl"

# Load trained model
with open(MODEL_FILE, "rb") as file:
    saved_data = pickle.load(file)

model = saved_data["model"]

SAFE_DISTANCE = 1.0
PREDICTION_TIME = 3.0


def predict_risk(x, y, vx, vy):

    distance = math.sqrt(x*x + y*y)

    if distance < 0.01:
        distance = 0.01

    speed_squared = vx*vx + vy*vy
    speed = math.sqrt(speed_squared)

    closing_speed = -(x*vx + y*vy) / distance

    if speed_squared > 0.0001:
        t_closest = -(x*vx + y*vy) / speed_squared
    else:
        t_closest = 0.0

    if t_closest < 0:
        t_closest = 0.0

    closest_x = x + vx*t_closest
    closest_y = y + vy*t_closest

    closest_distance = math.sqrt(
        closest_x*closest_x +
        closest_y*closest_y
    )

    features = [[
        round(distance, 3),
        round(closing_speed, 3),
        round(t_closest, 3),
        round(closest_distance, 3)
    ]]

    prediction = model.predict(features)[0]

    probabilities = model.predict_proba(features)[0]
    risk_probability = probabilities[1] * 100

    print("\nOBSTACLE ANALYSIS")
    print("---------------------------")
    print("Distance: %.2f" % distance)
    print("Closing speed: %.2f" % closing_speed)
    print("Time to closest approach: %.2f s" % t_closest)
    print("Closest distance: %.2f" % closest_distance)

    print("Predicted risk probability: %.2f%%"
          % risk_probability)

    if prediction == 1:
        print("RESULT: COLLISION RISK")
        print("Recommendation: Avoid this obstacle.")
    else:
        print("RESULT: SAFE")
        print("Recommendation: Continue monitoring.")

    return prediction


if __name__ == "__main__":

    print("ROBOT OBSTACLE RISK PREDICTION")
    print("==============================")

    while True:
        print("\nEnter obstacle position and velocity.")
        print("Position: x, y")
        print("Velocity: vx, vy")

        try:
            x = float(input("Enter x: "))
            y = float(input("Enter y: "))
            vx = float(input("Enter vx: "))
            vy = float(input("Enter vy: "))

            predict_risk(x, y, vx, vy)

        except ValueError:
            print("Please enter valid numerical values.")

        choice = input(
            "\nTest another obstacle? (y/n): "
        )

        if choice.lower() != 'y':
            break

