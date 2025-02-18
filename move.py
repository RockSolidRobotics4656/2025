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

class AbsoluteRotate(commands2.Command):
    max_turn = 0.2
    def __init__(self, drivetrain: drive.SwerveDrive, destination: float):
        self.drivetrain = drivetrain
        self.addRequirements(self.drivetrain)
        self.destination = destination
        self.turn_controller = controller.ProfiledPIDController(0.01, 0, 0, 
            trajectory.TrapezoidProfile.Constraints(10000, 0.1))
    
    def initialize(self):
        self.turn_controller.reset(self.drivetrain.gyro())
        self.turn_controller.enableContinuousInput(0, 360)
        self.turn_controller.setTolerance(1)

    def execute(self):
        value = self.turn_controller.calculate(self.drivetrain.gyro(), self.destination)
        print(f"({self.drivetrain.gyro()}, {self.destination})")
        clamped = control.clamp_mag(self.max_turn, value)
        print(clamped)
        self.drivetrain.polar_drive(drive.Polar(0, 0), clamped)

    def end(self, _interrupted: bool):
        self.drivetrain.polar_drive(drive.Polar(0, 0), 0)

    def isFinished(self) -> bool:
        return self.turn_controller.atGoal()

class RelativeMove(commands2.Command):
    threshold_dist = 0.05
    max_speed = 0.2
    def __init__(self, drivetrain: drive.SwerveDrive, destination: geometry.Translation2d):
        super().__init__()
        self.drivetrain = drivetrain
        self.addRequirements(self.drivetrain)
        self.destination = destination
        # TODO: Tweak
        self.trans_controller = controller.ProfiledPIDController(0.5, 0.0, 0.0,
            trajectory.TrapezoidProfile.Constraints(1000, 0.1))
        self.trans_controller.setTolerance(self.threshold_dist)
    
    def initialize(self):
        self.odometry = self.drivetrain.create_odometry()
        self.trans_controller.reset(0)
    
    def execute(self):
        self.drivetrain.update_odo(self.odometry)
        desired_trans_correction = self.trans_controller.calculate(self.distance_to_target())
        trans_power = control.clamp_mag(self.max_speed, desired_trans_correction)
        trans = drive.Polar(
            trans_power,
            self.angle_to_target()
        )
        self.drivetrain.polar_drive(trans, 0)
    
    def dydx(self) -> Tuple[float, float]:
        pose = odo_adapter(self.odometry.getPose())
        return (
            pose.Y() - self.destination.Y(),
            pose.X() - self.destination.X()
        )
    def angle_to_target(self) -> float:
        dy, dx = self.dydx()
        tmp = math.degrees(math.atan2(dy, dx))
        while tmp < 360: tmp += 360
        tmp = tmp % 360
        return tmp

    def distance_to_target(self) -> float:
        dy, dx = self.dydx()
        return math.sqrt(dy**2 + dx**2)

    def isFinished(self) -> bool:
        return self.trans_controller.atGoal()
    
    def end(self, _interrupted: bool):
        trans = drive.Polar(0.0, 0)
        self.drivetrain.polar_drive(trans, 0)

# April Tag Align
# TODO: pass in controller parameters as suppliers for offsets - could be nice
class AprilAlign(commands2.Command):
    def __init__(self, vision: april.VisionSystem, drivetrain: drive.SwerveDrive):
        super().__init__()

        # Drivetrain
        self.addRequirements(drivetrain)
        self.addRequirements(vision)
        self.drivetrain = drivetrain

        self.turn_controller = controller.ProfiledPIDController(0.1, 0, 0, 
            trajectory.TrapezoidProfile.Constraints(10000, 0.1))
        self.turn_controller.enableContinuousInput(0, 360)
        self.turn_controller.setTolerance(1)

        self.lateral_controller = controller.ProfiledPIDController(1, 0, 0,
            trajectory.TrapezoidProfile.Constraints(10000, 0.1))
        self.lateral_controller.setTolerance(0.02)

        self.vision = vision

    def initialize(self):
        if info := self.vision():
            self.tagid = info[0]
            self.odometry = self.drivetrain.create_odometry()
            self.odometry.resetPose(info[1])
        else:
            self.cancel()
    
    def execute(self):
        self.drivetrain.update_odo(self.odometry)
        pose = self.odometry.getPose()
        current_angle = pose.rotation().degrees()
        while current_angle < 0: current_angle += 360
        current_angle %= 360
        
        self.turn_controller.setGoal(0)
        yaw_correction = self.turn_controller.calculate(current_angle)
        yaw_clamped = control.clamp_mag(0.2, yaw_correction)

        self.lateral_controller.setGoal(0.15) # Center ourselves
        lat_correction = self.lateral_controller.calculate(pose.X())
        lat_clamped = control.clamp_mag(0.2, lat_correction)
        
        lat_correct = drive.Polar(
            lat_clamped, 0
        )
        translate = lat_correct
        self.drivetrain.polar_drive(translate, yaw_clamped, relative=True)

    def end(self, interrupted: bool):
        self.drivetrain.polar_drive(drive.Polar(0,self.drivetrain.gyro()), 0)
    
    def isFinished(self):
        return self.turn_controller.atGoal() and self.lateral_controller.atGoal()


def tmpapalign(vision: april.VisionSystem, d: drive.SwerveDrive) -> commands2.Command:
    fwd = commands2.RunCommand(lambda: d.polar_drive(drive.Polar(0.1, 90), 0, True))
    return AprilAlign(vision, d).andThen(fwd)