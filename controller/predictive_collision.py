import math


def predict_obstacle_position(position, velocity, time):
    """
    Predict obstacle position after a given time.

    Coordinates use (row, column).
    Velocity uses (row_velocity, column_velocity).
    """

    future_row = position[0] + velocity[0] * time
    future_col = position[1] + velocity[1] * time

    return future_row, future_col


def check_predictive_collision(
        robot_position,
        obstacle_position,
        obstacle_velocity,
        arrival_time,
        safety_radius=1.0):

    predicted_obstacle = predict_obstacle_position(
        obstacle_position,
        obstacle_velocity,
        arrival_time
    )

    row_difference = robot_position[0] - predicted_obstacle[0]
    col_difference = robot_position[1] - predicted_obstacle[1]

    distance = math.sqrt(
        row_difference ** 2 + col_difference ** 2
    )

    if distance <= safety_radius:
        return False, "PREDICTED COLLISION", distance, predicted_obstacle

    return True, "PATH CLEAR", distance, predicted_obstacle
