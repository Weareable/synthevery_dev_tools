from collections import deque

class MovingWindow:
    def __init__(self, window_size: int):
        self.window_size = window_size
        self.values = deque()
        self.current_average = 0.0
        self.current_variance = 0.0

    def update(self, value: float):
        if len(self.values) >= self.window_size:
            self.values.popleft()
        self.values.append(value)

        self.current_average = sum(self.values) / len(self.values)

        variance = sum((v - self.current_average) ** 2 for v in self.values)
        self.current_variance = variance / len(self.values)

    def average(self) -> float:
        return self.current_average

# Difference クラス
class Difference:
    def __init__(self):
        self.previous_value = 0.0
        self.previous_time = 0.0
        self.current_difference = 0.0
        self.current_time_difference = 0.0

    def update(self, value: float, time: float):
        self.current_difference = value - self.previous_value
        self.current_time_difference = time - self.previous_time
        self.previous_value = value
        self.previous_time = time

    def difference(self) -> float:
        return self.current_difference

    def time_difference(self) -> float:
        if self.current_time_difference == 0:
            return 0.0
        return self.current_time_difference

    def velocity(self) -> float:
        return self.current_difference / self.current_time_difference if self.current_time_difference != 0 else 0.0

# Peak クラス
class Peak:
    def __init__(self, window_size: int):
        self.values = []
        self.current_max_peak = 0.0
        self.current_min_peak = 0.0
        self.window_size = window_size

    def update(self, value: float):
        if len(self.values) >= self.window_size:
            if self.values[0] == self.current_max_peak:
                self.current_max_peak = max(self.values[1:])
            if self.values[0] == self.current_min_peak:
                self.current_min_peak = min(self.values[1:])
            self.values.pop(0)

        self.values.append(value)

        if value > self.current_max_peak:
            self.current_max_peak = value
        if value < self.current_min_peak:
            self.current_min_peak = value

    def max_peak(self) -> float:
        return self.current_max_peak

    def min_peak(self) -> float:
        return self.current_min_peak

    def peak_to_peak(self) -> float:
        return self.current_max_peak - self.current_min_peak

# LPF クラス
class LPF:
    def __init__(self, alpha: float):
        self.alpha = alpha
        self.value_ = 0.0

    def update(self, value: float):
        self.value_ = self.value_ * (1 - self.alpha) + value * self.alpha

    def value(self) -> float:
        return self.value_
