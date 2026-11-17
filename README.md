
# AI-Based Robot Path Planning and Obstacle Avoidance

## Project Overview

This project implements an intelligent robot navigation system that plans paths, detects obstacles, predicts collision risks, and performs route replanning in a simulated environment.

It combines classical path-planning algorithms with machine learning and independent safety checks.

## Objectives

- Find an efficient path from a start position to a goal.
- Navigate around static and dynamic obstacles.
- Simulate sensor-based environmental perception.
- Use machine learning to estimate collision risk.
- Predict future obstacle positions.
- Perform emergency stops and route replanning.

## Technologies Used

- Python 3
- A* Search Algorithm
- Tkinter
- NumPy and Pandas
- Scikit-learn
- Random Forest Classifier
- Pickle

## Project Architecture

```text
robot-path-planning/
|
|-- ai/
|   |-- ai_navigation.py
|   |-- ai_navigation_safe.py
|   |-- ai_navigation_final.py
|   |-- generate_obstacle_data.py
|   |-- train_obstacle_model.py
|   |-- predict_obstacle.py
|   |-- datasets/
|   |   |-- obstacle_data.csv
|   |-- models/
|       |-- obstacle_risk_model.pkl
|
|-- algorithms/
|
|-- controller/
|   |-- safety_layer.py
|   |-- predictive_collision.py
|
|-- simulation/
|
|-- tests/
|   |-- test_safety_layer.py
|   |-- test_predictive_collision.py
|
|-- main.py
|-- visual_simulator.py
|-- perception_simulator.py
|-- integrated_simulator.py
|-- sensor_model.py
|-- README.md
|-- .gitignore
```

## Main Features

### 1. A* Path Planning
Uses the A* search algorithm and Manhattan distance heuristic to find a path through a grid-based environment.

### 2. Obstacle Perception
Simulates an initially unknown environment and discovers obstacles using simulated sensor readings.

### 3. Sensor Noise Reduction
Uses repeated sensor observations and majority voting to reduce the effect of noisy readings.

### 4. AI-Based Collision Risk Prediction
A Random Forest classifier estimates collision risk using:
- Distance
- Closing speed
- Time to closest approach
- Closest predicted distance

### 5. Predictive Collision Checking
Estimates the moving obstacle's future position and checks its distance from the robot's expected next position.

### 6. Independent Safety Layer
Checks for static obstacles and nearby dynamic obstacles before allowing movement.

### 7. Dynamic Path Replanning
Recalculates the robot's route when a potential collision or blocked path is detected.

## Dataset and Machine Learning

A synthetic dataset containing 5,000 samples is generated for training the obstacle-risk classifier.

The dataset includes safe and collision-risk scenarios.

The model is trained using a Random Forest classifier and evaluated on a held-out test set.

**Note:** The reported model performance is based on synthetic data. It does not establish real-world collision detection accuracy.

## How to Run

Clone the repository:

```bash
git clone YOUR_GITHUB_REPOSITORY_URL
cd robot-path-planning
```

Run the basic A* planner:

```bash
python3 main.py
```

Run the graphical simulator:

```bash
python3 visual_simulator.py
```

Run the perception simulator:

```bash
python3 perception_simulator.py
```

Run the integrated simulator:

```bash
python3 integrated_simulator.py
```

Run the final AI-assisted navigation system:

```bash
python3 ai/ai_navigation_final.py
```

Run the tests:

```bash
python3 tests/test_safety_layer.py
python3 tests/test_predictive_collision.py
```

## Limitations

- The environment and obstacle movement are simulated.
- The machine-learning dataset is synthetic.
- Predictive collision checking currently evaluates a fixed arrival-time estimate rather than continuous trajectory intersection.
- The system has not been validated on a physical robot.
- Safety checks are prototype-level and are not certified for real-world deployment.

## Future Enhancements

- Integrate physical distance sensors.
- Implement real-time sensor fusion.
- Add ROS-based robot simulation.
- Improve trajectory prediction.
- Test navigation in a physics-based simulation environment.
- Deploy the navigation system on a mobile robot.

## Project Status

Software-based prototype completed with AI-assisted navigation, predictive obstacle checking, independent safety checks, and automated test programs.

---

**Project:** AI-Based Robot Path Planning and Obstacle Avoidance

