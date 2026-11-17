
import random


class SensorModel:
    def __init__(self, error_rate=0.10):
        self.error_rate = error_rate

    def read(self, true_cell):
        if random.random() < self.error_rate:
            return '#' if true_cell == '.' else '.'

        return true_cell

    def majority_vote(self, true_cell, samples=5):
        readings = [
            self.read(true_cell)
            for _ in range(samples)
        ]

        obstacles = readings.count('#')
        free_cells = readings.count('.')

        if obstacles > free_cells:
            result = '#'
        else:
            result = '.'

        return result, readings


if __name__ == "__main__":
    random.seed()

    sensor = SensorModel(error_rate=0.20)

    for true_cell in ['.', '#']:
        correct = 0
        total = 1000

        for _ in range(total):
            result, readings = sensor.majority_vote(
                true_cell, samples=5
            )

            if result == true_cell:
                correct += 1

        accuracy = (correct * 100.0) / total

        print("True cell:", true_cell)
        print("Majority-vote accuracy: %.2f%%" % accuracy)
        print()

