from typing import *
import commands2
import control
import ncoms
import drive
import april
from wpimath import controller, geometry, filter, trajectory

angle_lookup_table = {
    # Blue Reef
    17: 60,
    18: 0,
    19: 300,
    20: 60 + 180,
    21: 180,
    22: 300 - 180,
    # Blue Stations
    12: 60,
    13: 300,
    # Red Reef
    8: 60,
    7: 0,
    6: 300,
    11: 60 + 180,
    10: 180,
    9: 300 - 180,
    # Red Stations
    2: 60,
    1: 300,

}

reef_leftoff = 0.2
reef_rightoff = 0.5
reef_leftoff_sweep = 0.1
reef_rightoff_sweep = 0.4

class Align(commands2.Command):
    max_blackhole_effect = 0.3
    def __init__(self, vision: april.VisionSystem, drivetrain: drive.SwerveDrive, fwddir: float, ps: Callable[[], drive.Polar], targetlat: float, latpid: float = 0.2, spd=0.05):
        self.drivetrain = drivetrain
        self.vision = vision
        self.addRequirements(self.drivetrain)
        self.addRequirements(self.vision)
        self.id = None
        self.fwddir = fwddir
        self.ps = ps
        self.targetlat = targetlat
        self.latpid = latpid
        self.spd = spd

    def initialize(self):
        self.odometry = self.drivetrain.create_odometry()
        self.tagid = None
        if ready := self.vision():
            self.tagid = ready[0]
            self.odometry.resetPose(ready[1])
        else:
            print("Cannot begin April Alignment because no april tag is detected!")
        self.turn_controller = controller.PIDController(0.01, 0, 0)
        self.turn_controller.enableContinuousInput(0, 360)
        self.turn_controller.setTolerance(1)
        self.trans_controller = controller.PIDController(1.0, 0, 0)

        self.lat_pid = controller.PIDController(self.latpid, 0, 0)
        self.lat_filter = filter.MedianFilter(10)
        self.alignx = 0
    def dy(self):
        pose = self.odometry.getPose()
        return abs(pose.Y())
    def execute(self):
        # Spinout condition
        if self.tagid is None: return
        self.drivetrain.update_odo(self.odometry)

        # Angular
        desired_angle = angle_lookup_table[self.tagid]
        current_angle = self.drivetrain.gyro()
        yaw_correction = -self.turn_controller.calculate(current_angle, desired_angle)
        yaw_clamped = control.clamp_mag(0.2, yaw_correction)

        # Translate
        fwd = drive.Polar(
            self.spd, self.drivetrain.gyro() + self.fwddir
        )
        cont = self.ps()

        # Blackhole
        if detect := self.vision():
            pose = detect[1]
            self.alignx = self.lat_filter.calculate(pose.X())
        xcor = control.clamp_mag(self.max_blackhole_effect, self.lat_pid.calculate(self.alignx, self.targetlat))
        blackhole = drive.Polar(
            xcor, self.drivetrain.gyro()
        )

        # Final Mix
        trans = drive.mix_polar(drive.mix_polar(fwd, cont), blackhole)
        self.drivetrain.polar_drive(trans, yaw_clamped, False)
    
    def end(self, _interrupted: bool):
        self.drivetrain.polar_drive(drive.Polar(0, 0), 0, True)