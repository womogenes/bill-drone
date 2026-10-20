"""
Convert between ESC commands and thrust, and average stationary gyro bias.
The thrust table uses the median of three calibrated sweeps in sw/slop/comparison.csv.
Throttle 200 is treated as approximately zero thrust, based on the bench observation.
"""

# Raw sensor axes to body X forward, Y left, Z up.
# Confirmed by nose-down and left-side-up checks.
SENSOR_TO_BODY = (1, -1, -1)

# [throttle, thrust_N]

THRUST_TABLE = [
    [0, 0.0],
    [200, 0.0],
    [250, 0.758689175],
    [300, 1.140325966],
    [350, 1.544788528],
    [400, 1.955279404],
    [450, 2.353557228],
    [500, 2.775707658],
    [550, 3.261558474],
    [600, 3.673818136],
    [650, 4.264941843],
    [700, 4.898708952],
    [750, 5.566624474],
    [800, 6.313341249],
    [850, 7.172694844],
    [900, 8.00416899],
    [950, 8.815765343],
    [1000, 9.61023936],
    [1050, 10.438741463],
    [1100, 11.216827129],
    [1150, 11.052149476],
    [1200, 10.992227314],
]
PEAK = max(range(len(THRUST_TABLE)), key=lambda index: THRUST_TABLE[index][1])


def interpolate(value, points, input_column=0):
    """
    Interpolate between table rows; clamp to the endpoint values.
    """
    output_column = 1 - input_column
    if value <= points[0][input_column]:
        return points[0][output_column]

    for before, after in zip(points, points[1:]):
        if value <= after[input_column]:
            fraction = (value - before[input_column]) / (after[input_column] - before[input_column])
            return before[output_column] + fraction * (after[output_column] - before[output_column])

    return points[-1][output_column]


def thrust(throttle):
    """
    Convert an ESC command to newtons, including the measured decline above 1100.
    """
    return interpolate(throttle, THRUST_TABLE)


def throttle(force):
    """
    Convert newtons to an ESC command using the rising curve, capped at 1100.
    """
    return interpolate(force, THRUST_TABLE[:PEAK + 1], input_column=1)


class GyroCalibration:
    def __init__(self):
        self.bias = [0.0, 0.0, 0.0]
        self.remaining = 0

    def start(self, samples=200):
        self.samples = samples
        self.total = [0.0, 0.0, 0.0]
        self.remaining = samples

    def update(self, gyro):
        if self.remaining:
            for axis in range(3):
                self.total[axis] += gyro[axis]
            self.remaining -= 1
            count = self.samples - self.remaining
            self.bias = [value / count for value in self.total]

        return [value - bias for value, bias in zip(gyro, self.bias)]
