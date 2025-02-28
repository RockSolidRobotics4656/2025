from typing import *
import math
import commands2
from wpimath import geometry, filter, controller, trajectory
import drive
import control

def normangle(x: float) -> float:
    while x <= 0: x += 360
    return x % 360

class Move(commands2.Command):
    max_linear = 0.8
    tol_linear = 0.05
    max_turn = 0.2
    tol_turn = 1
    def __init__(self, d: drive.SwerveDrive, position: geometry.Translation2d, angle: Optional[float]):
        super().__init__()
        self.drive = d
        self.addRequirements(self.drive)
        self.destination = position
        self.fangle = angle
    
    def initialize(self):
        self.linear_pid = controller.ProfiledPIDController(1.0, 0, 0,
            trajectory.TrapezoidProfile.Constraints(1000, 0.2))
        self.linear_pid.setTolerance(self.tol_linear)
        self.angular_pid = controller.PIDController(0.04, 0, 0)
        self.angular_pid.enableContinuousInput(0, 360)
        self.angular_pid.setTolerance(self.tol_turn)
    
    def execute(self):
        pose = self.drive.odometry.getPose()

        # Linear 
        dx = self.destination.X() - pose.X()
        dy = self.destination.Y() - pose.Y()
        linear_dst = math.sqrt(dx**2 + dy**2)
        linear_spd = control.clamp_mag(self.max_linear, -self.linear_pid.calculate(linear_dst, 0))
        trans_angle = normangle(math.degrees(math.atan2(dy, dx)))
        trans = drive.Polar(linear_spd, trans_angle)

        # Angular
        angular_spd = 0
        if self.fangle:
            angular_spd = control.clamp_mag(self.max_turn, -self.angular_pid.calculate(
                pose.rotation().degrees(), self.fangle))

        self.drive.polar_drive(trans, angular_spd, False)
    
    def isFinished(self):
        return self.linear_pid.atGoal() and (self.angular_pid.atSetpoint() or not self.fangle)
    
    def end(self, interrupted):
        print("Completed!")

