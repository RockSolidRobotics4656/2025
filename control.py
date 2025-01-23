import math
import wpimath
import ncoms
import random

clamp_mag = lambda mag, x: max(-mag, min(mag, x))

class SwerveFunc:
    def __init__(self):
        from wpimath.controller import PIDController
        from wpimath.filter import LinearFilter
        self.pid = PIDController(1.0, 0.0, 0.0)
        self.pid.enableContinuousInput(-1, 1)
        self.deos = LinearFilter.singlePoleIIR(0.1, 0.02)

    def __call__(self, current: float, target: float):
        # Norm
        while target < 0: target += 360
        target = target % 360
        target /= 180 - 1 

        while current < 0: current += 360
        current = current % 360
        current /= 180 - 1

        # Logic
        self.pid.setSetpoint(target)
        value = self.pid.calculate(current)
        value *=- 1 # PID is backwards by default

        return self.deos.calculate(clamp_mag(1.0, value))