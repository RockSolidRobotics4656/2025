from typing import *
import drive
import commands2
from wpimath import geometry, controller, trajectory, kinematics
import math
import ncoms
import control
import april
import wpimath
import wpilib
import dataclasses

def odo_adapter(incorrect: geometry.Pose2d) -> geometry.Pose2d:
    return incorrect.rotateBy(geometry.Rotation2d.fromDegrees(0))
def normangle(x: float) -> float:
    while x <= 0: x += 360
    return x % 360

class RelativeMove(commands2.Command):
    threshold_dist = 0.03
    max_speed = 0.3
    max_turn = 0.2
    threshold_angle = 3
    def __init__(self, drivetrain: drive.SwerveDrive, destination: geometry.Translation2d, angle: Optional[float]):
        super().__init__()
        self.drivetrain = drivetrain
        self.addRequirements(self.drivetrain)
        self.destination = destination
        # TODO: Tweak
        self.trans_controller = controller.ProfiledPIDController(1.5, 0.0, 0.0,
            trajectory.TrapezoidProfile.Constraints(1000, 0.1))
        self.trans_controller.setTolerance(self.threshold_dist)

        self.turn_controller = None
        if angle:
            self.turn_controller = controller.PIDController(0.015, 0, 0)
            self.turn_controller.enableContinuousInput(0, 360)
            self.turn_controller.setTolerance(self.threshold_angle)
            self.turn_controller.setSetpoint(angle)
    
    def initialize(self):
        self.odometry = self.drivetrain.create_odometry()
    
    def execute(self):
        self.drivetrain.update_odo(self.odometry)
        desired_trans_correction = self.trans_controller.calculate(self.distance_to_target())
        trans_power = control.clamp_mag(self.max_speed, desired_trans_correction)
        trans = drive.Polar(
            trans_power,
            self.angle_to_target()
        )
        # Turning
        rotate_correction = 0
        if self.turn_controller is not None:
            desired_rotate_correction = -self.turn_controller.calculate(self.drivetrain.gyro())
            rotate_correction = control.clamp_mag(self.max_turn, desired_rotate_correction)
        self.drivetrain.polar_drive(trans, rotate_correction)
    
    def dydx(self) -> Tuple[float, float]:
        pose = odo_adapter(self.odometry.getPose())
        return (
            pose.Y() - self.destination.Y(),
            pose.X() - self.destination.X()
        )
    def angle_to_target(self) -> float:
        dy, dx = self.dydx()
        return normangle(math.degrees(math.atan2(dy, dx)))

    def distance_to_target(self) -> float:
        dy, dx = self.dydx()
        return math.sqrt(dy**2 + dx**2)

    def isFinished(self) -> bool:
        turn_correct = True
        if self.turn_controller is not None:
            turn_correct = self.turn_controller.atSetpoint()
        return self.trans_controller.atGoal() and turn_correct
    
    def end(self, _interrupted: bool):
        trans = drive.Polar(0.0, 0)
        self.drivetrain.polar_drive(trans, 0)
