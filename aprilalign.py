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
import move

# April Tag Align
# TODO: pass in controller parameters as suppliers for offsets - could be nice
class AprilAlign(commands2.Command):
    def __init__(self, vision: april.VisionSystem, drivetrain: drive.SwerveDrive, fwd: float,
            off_supp: Callable[[], float]):
        super().__init__()
        # Drivetrain
        self.fwd = fwd
        self.addRequirements(drivetrain)
        self.addRequirements(vision)
        self.drivetrain = drivetrain
        self.vision = vision
        self.off_supp = off_supp

        self.turn_controller = controller.PIDController(0.01, 0, 0)
        self.turn_controller.enableContinuousInput(0, 360)
        self.turn_controller.setTolerance(1)

        self.lateral_controller = controller.ProfiledPIDController(1, 0, 0,
            trajectory.TrapezoidProfile.Constraints(10000, 0.1))
        self.lateral_controller.setTolerance(0.02)


    def initialize(self):
        if info := self.vision():
            self.tagid = info[0]
            self.odometry = self.drivetrain.create_odometry()
            self.odometry.resetPose(info[1])
        else:
            print("No april tag found (Aborting)")
            self.cancel()
    
    def execute(self):
        self.drivetrain.update_odo(self.odometry)
        pose = self.odometry.getPose()
        current_angle = move.normangle(pose.rotation().degrees())
        
        self.turn_controller.setSetpoint(0)
        yaw_correction = -self.turn_controller.calculate(current_angle)
        yaw_clamped = control.clamp_mag(0.2, yaw_correction)

        self.lateral_controller.setGoal(self.off_supp()) # Center ourselves
        lat_correction = self.lateral_controller.calculate(pose.X())
        lat_clamped = control.clamp_mag(0.2, lat_correction)
        
        lat_correct = drive.Polar(
            lat_clamped, 360 - current_angle 
        )
        fwd = drive.Polar(0.1, move.normangle(360 - current_angle + self.fwd)) 
        translate = drive.mix_polar(fwd, lat_correct)
        self.drivetrain.polar_drive(translate, yaw_clamped, relative=True)

        ncoms.dbg_tab.putNumber("RelX", pose.X())
        ncoms.dbg_tab.putNumber("RelAError", self.turn_controller.getError())

    def end(self, _interrupted: bool):
        self.drivetrain.polar_drive(drive.Polar(0,0), 0)
    
    def isFinished(self):
        return False
        return self.turn_controller.atSetpoint() and self.lateral_controller.atGoal()


def tmpapalign(vision: april.VisionSystem, d: drive.SwerveDrive) -> commands2.Command:
    return AprilAlign(vision, d)