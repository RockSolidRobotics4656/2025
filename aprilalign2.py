from typing import *
import commands2
import control
import ncoms
import drive
import april
from wpimath import controller, geometry, filter

angle_lookup_table = {
    6: 300,
    7: 0,
    8: 60,
    17: 60,
    18: 0,
    19: 300,

}

class Align(commands2.Command):
    def __init__(self, vision: april.VisionSystem, drivetrain: drive.SwerveDrive, fwddir: float, ps: Callable[[], drive.Polar]):
        self.drivetrain = drivetrain
        self.vision = vision
        self.addRequirements(self.drivetrain)
        self.addRequirements(self.vision)
        self.id = None
        self.fwddir = fwddir
        self.ps = ps
    
    def initialize(self):
        self.odometry = self.drivetrain.create_odometry()
        if ready := self.vision():
            self.tagid = ready[0]
            self.odometry.resetPose(ready[1])
        else:
            print("Cannot begin April Alignment because no april tag is detected!")
            self.cancel()
        self.turn_controller = controller.PIDController(0.01, 0, 0)
        self.turn_controller.enableContinuousInput(0, 360)
        self.turn_controller.setTolerance(1)
        self.trans_controller = controller.PIDController(1.0, 0, 0)

    def execute(self):
        self.drivetrain.update_odo(self.odometry)
        desired_angle = angle_lookup_table[self.tagid]
        current_angle = self.drivetrain.gyro()
        yaw_correction = -self.turn_controller.calculate(current_angle, desired_angle)
        yaw_clamped = control.clamp_mag(0.2, yaw_correction)
        cont = self.ps()
        fwd = drive.Polar(
            0.15, self.drivetrain.gyro() + self.fwddir
        )
        trans = drive.mix_polar(fwd, cont)
        self.drivetrain.polar_drive(trans, yaw_clamped, False)
    
    def end(self, _interrupted: bool):
        self.drivetrain.polar_drive(drive.Polar(0, 0), 0, True)