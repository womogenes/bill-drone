"""
Convert between ESC commands and thrust, and average stationary gyro bias.
The thrust table uses the median of three calibrated sweeps in sw/slop/comparison.csv.
Throttle 200 is treated as approximately zero thrust, based on the bench observation.
"""

THROTTLES = (0, 200, 250, 300, 350, 400, 450, 500, 550, 600, 650, 700, 750, 800, 850, 900, 950, 1000, 1050, 1100, 1150, 1200)
THRUSTS = (0.0, 0.0, 0.758689175, 1.140325966, 1.544788528, 1.955279404, 2.353557228, 2.775707658, 3.261558474, 3.673818136, 4.264941843, 4.898708952, 5.566624474, 6.313341249, 7.172694844, 8.00416899, 8.815765343, 9.61023936, 10.438741463, 11.216827129, 11.052149476, 10.992227314)
PEAK = THRUSTS.index(max(THRUSTS))


def interpolate(value, xs, ys):
    """
    Interpolate linearly between table entries; clamp to the endpoint values.
    """
    if value <= xs[0]:
        return ys[0]
    for index in range(1, len(xs)):
        if value <= xs[index]:
            fraction = (value - xs[index - 1]) / (xs[index] - xs[index - 1])
            return ys[index - 1] + fraction * (ys[index] - ys[index - 1])
    return ys[-1]


def thrust(throttle):
    """
    Convert an ESC command to newtons, including the measured decline above 1100.
    """
    return interpolate(throttle, THROTTLES, THRUSTS)


def throttle(force):
    """
    Convert newtons to an ESC command using the rising curve, capped at 1100.
    """
    return interpolate(force, THRUSTS[:PEAK + 1], THROTTLES[:PEAK + 1])


class GyroCalibration:
    def __init__(self):
        self.bias = [0.0, 0.0, 0.0]
        self.remaining = 0

    def start(self, samples=200):
        self.samples = samples
        self.remaining = samples
        self.total = [0.0, 0.0, 0.0]

    def update(self, gyro):
        if self.remaining:
            for axis in range(3):
                self.total[axis] += gyro[axis]
            self.remaining -= 1
            count = self.samples - self.remaining
            self.bias = [value / count for value in self.total]

        return [value - bias for value, bias in zip(gyro, self.bias)]
