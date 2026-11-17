
import csv
import os
import pickle

from sklearn.cross_validation import train_test_split
from sklearn.ensemble import RandomForestClassifier
from sklearn.metrics import (
    accuracy_score,
    classification_report,
    confusion_matrix
)

DATA_FILE = "ai/datasets/obstacle_data.csv"
MODEL_DIR = "ai/models"
MODEL_FILE = os.path.join(
    MODEL_DIR, "obstacle_risk_model.pkl"
)

FEATURES = [
    "distance",
    "closing_speed",
    "time_to_closest",
    "closest_distance"
]

X = []
y = []

# Load dataset
with open(DATA_FILE, "r") as file:
    reader = csv.DictReader(file)

    for row in reader:
        values = [
            float(row[name]) for name in FEATURES
        ]

        X.append(values)
        y.append(int(row["collision_risk"]))

print("Dataset loaded successfully")
print("Total samples:", len(X))
print("Safe samples:", y.count(0))
print("Risk samples:", y.count(1))

# Split dataset
X_train, X_test, y_train, y_test = train_test_split(
    X,
    y,
    test_size=0.20,
    random_state=42,
    stratify=y
)

# Create classifier
model = RandomForestClassifier(
    n_estimators=100,
    max_depth=10,
    random_state=42,
    class_weight="balanced"
)

# Train
print("\nTraining model...")
model.fit(X_train, y_train)

# Evaluate
predictions = model.predict(X_test)

accuracy = accuracy_score(y_test, predictions)

print("\nMODEL EVALUATION")
print("------------------------------")
print("Accuracy: %.2f%%" % (accuracy * 100))

print(classification_report(
    y_test,
    predictions,
    target_names=["SAFE", "COLLISION RISK"]
))

print("Confusion Matrix:")
print(confusion_matrix(y_test, predictions))

# Save model
if not os.path.exists(MODEL_DIR):
    os.makedirs(MODEL_DIR)

model_data = {
    "model": model,
    "features": FEATURES
}

with open(MODEL_FILE, "wb") as file:
    pickle.dump(model_data, file)

print("\nModel saved successfully:")
print(MODEL_FILE)

