from typing import *
import drive
import commands2
from wpimath import geometry, controller, trajectory
import math
import ncoms
import control
import april
import dataclasses

@dataclasses.dataclass
class MoveProperties:
    speed: float = 0.1

# TODO: Boring Movement, needs to be callibrated, and it has quite a few bus
class StandardMove(commands2.Command):
    def __init__(self, drivetrain: drive.SwerveDrive, initial: geometry.Pose2d, final: geometry.Pose2d, prop: MoveProperties):
        super().__init__()
        self.addRequirements(drivetrain)
        self.drivetrain = drivetrain

        self.properties = prop

        self.odometry = self.drivetrain.create_odometry()
        self.odometry.resetPose(initial)
        self.dest = final
        
        self.translate_controller = controller.ProfiledPIDController(0.1, 0, 0,
            trajectory.TrapezoidProfile.Constraints(10000, 0.1))
        
    def initialize(self):
        print("Starting")
    
    def execute(self):
        self.drivetrain.update_odo(self.odometry)
        current = self.odometry.getPose()
        dy = self.dest.Y() - current.Y()
        dx = self.dest.X() - current.X()
        angle_to_translate = math.degrees(math.atan2(dy, dx))
        dist = math.sqrt(dy**2 + dx**2)
        value = self.translate_controller.calculate(dist)
        value = control.clamp_mag(0.2, value)
        value = 0.1
        translation = drive.Polar(
            value, angle_to_translate
        )
        self.drivetrain.polar_drive(translation, 0)
    
    def end(self, interrupted: bool):
        self.drivetrain.polar_drive(drive.Polar(0,self.drivetrain.gyro()), 0)
        print("Ending")

    def isFinished(self):
        self.drivetrain.update_odo(self.odometry)
        current = self.odometry.getPose()
        dy = self.dest.Y() - current.Y()
        dx = self.dest.X() - current.X()
        dist = math.sqrt(dy**2 + dx**2)
        return dist < 0.05 


# April Tag Align
class AprilAlign(commands2.Command):
    def __init__(self, vision: april.VisionSystem, drivetrain: drive.SwerveDrive, properties: MoveProperties):
        super().__init__()

        # Drivetrain
        self.addRequirements(drivetrain)
        self.addRequirements(vision)
        self.drivetrain = drivetrain
        self.prop = properties

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

        self.lateral_controller.setGoal(0) # Center ourselves
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


