from typing import *
import drive
import commands2
from wpimath import geometry, controller, trajectory, kinematics
import math
import ncoms
import control
import april
import dataclasses

def forward(drivetrain: drive.SwerveDrive) -> commands2.Command:
    odometry = drivetrain.create_odometry()
    final = geometry.Pose2d(0.5, 0.5, geometry.Rotation2d.fromDegrees(drivetrain.gyro()))
    return commands2.ParallelDeadlineGroup(
        StandardMove(drivetrain, odometry.getPose, final),
        commands2.RunCommand(lambda: drivetrain.update_odo(odometry))
    )

class StandardMove(commands2.Command):
    def __init__(self, drivetrain: drive.SwerveDrive, current_supp: Callable[[], geometry.Pose2d],
                final: geometry.Pose2d):
        super().__init__()
        self.addRequirements(drivetrain)
        self.drivetrain = drivetrain

        self.current_supp = current_supp
        self.dest = final
        self.start = self.current_supp()
        
        self.translate_controller = controller.ProfiledPIDController(1.0, 0, 0,
            trajectory.TrapezoidProfile.Constraints(10000, 0.1))
        
    def initialize(self):
        print("Starting Move")
    
    def distance(self) -> float:
        current = self.current_supp()
        dy = self.dest.Y() - (current.Y() - self.start.Y())
        dx = self.dest.X() - (current.X() - self.start.X())
        return math.sqrt(dy**2 + dx**2)
    def angleto(self) -> float: # Degrees
        current = self.current_supp()
        dy = self.dest.Y() - (current.Y() - self.start.Y())
        dx = self.dest.X() - (current.X() - self.start.X())
        return math.degrees(math.atan2(dy, dx))
    
    def execute(self):
        value = self.translate_controller.calculate(self.distance())
        value = control.clamp_mag(0.2, value)
        translation = drive.Polar(
            value, self.angleto()
        )
        self.drivetrain.polar_drive(translation, 0)
        print("Whatever:")
        print(self.current_supp(), self.dest, self.angleto())
    
    def end(self, interrupted: bool):
        self.drivetrain.polar_drive(drive.Polar(0,self.drivetrain.gyro()), 0)
        print("Ending Move")

    def isFinished(self):
        return self.distance() < 0.05


# April Tag Align
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