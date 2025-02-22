import commands2
import control
import ncoms
import drive
import april
from wpimath import controller

angle_lookup_table = {
    1: 60,
    12: 60,
}

class Align(commands2.Command):
    def __init__(self, vision: april.VisionSystem, drivetrain: drive.SwerveDrive, fwddir: float):
        self.drivetrain = drivetrain
        self.vision = vision
        self.addRequirements(self.drivetrain)
        self.addRequirements(self.vision)
        self.id = None
        self.fwddir = fwddir
    
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
        if detected := self.vision():
            if abs(detected[1].Y()) > 1.0:
                self.odometry.resetPose(detected[1])
        pose = self.odometry.getPose()
        desired_angle = angle_lookup_table[self.tagid]
        current_angle = self.drivetrain.gyro()
        yaw_correction = -self.turn_controller.calculate(current_angle, desired_angle)
        yaw_clamped = control.clamp_mag(0.2, yaw_correction)

        fwd = drive.Polar(
            0.15, self.fwddir
        )
        ncoms.dbg_tab.putNumber("Relx", pose.X())
        val = self.trans_controller.calculate(pose.X(), 0)
        lat = drive.Polar(
            control.clamp_mag(0.2, val), 0
        )
        trans = drive.mix_polar(lat, fwd)
        self.drivetrain.polar_drive(trans, yaw_clamped, True)
