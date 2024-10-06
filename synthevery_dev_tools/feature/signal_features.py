from collections import deque
import math

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
class BackwardDifferenceLPF:
    def __init__(self, alpha: float):
        self.alpha = alpha
        self.value_ = 0.0

    def update(self, value: float):
        self.value_ = self.value_ * (1 - self.alpha) + value * self.alpha

    def value(self) -> float:
        return self.value_
    
    @staticmethod
    def calc_coef(cutoff_frequency: float, sample_time: float):
        cutoff_rads = 2.0 * math.pi * cutoff_frequency
        alpha = sample_time * cutoff_rads / (1.0 + sample_time * cutoff_rads)
        return alpha
    
    def set_coef(self, alpha: float):
        self.alpha = alpha

    @staticmethod
    def from_cutoff_frequency(cutoff_frequency: float, sample_time: float):
        return BackwardDifferenceLPF(BackwardDifferenceLPF.calc_coef(cutoff_frequency, sample_time))
    
class BilinearTransformLPF:
    def __init__(self, alpha: float, beta: float):
        self.alpha = alpha
        self.beta = beta
        self.previous_input = 0.0
        self.previous_output = 0.0
        self.current_output = 0.0

    @staticmethod
    def calc_coef(cutoff_frequency: float, sample_time: float):
        cutoff_rads = 2.0 * math.pi * cutoff_frequency
        alpha = sample_time * cutoff_rads / (2.0 + sample_time * cutoff_rads)
        beta = (sample_time * cutoff_rads - 2.0) / (sample_time * cutoff_rads + 2.0)
        return alpha, beta
    
    def set_coef(self, alpha: float, beta: float):
        self.alpha = alpha
        self.beta = beta

    @staticmethod
    def from_cutoff_frequency(cutoff_frequency: float, sample_time: float):
        return BilinearTransformLPF(*BilinearTransformLPF.calc_coef(cutoff_frequency, sample_time))

    def update(self, current_input: float):
        self.current_output = self.alpha * (current_input + self.previous_input) - self.beta * self.previous_output
        self.previous_input = current_input
        self.previous_output = self.current_output

    def value(self) -> float:
        return self.current_output