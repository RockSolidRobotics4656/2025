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

    # Testing
    17: 0
}

reef_leftoff = 0.3
reef_rightoff = 0.5
reef_middle_sweep = (reef_leftoff + reef_rightoff) / 2
reef_leftoff_sweep = 0.1
reef_rightoff_sweep = 0.4

class Align(commands2.Command):
    max_blackhole_effect = 0.3
    def __init__(self, vision: april.VisionSystem, drivetrain: drive.SwerveDrive, fwddir: float, ps: Callable[[], drive.Polar], targetlat: float, latpid: float = 0.2, spd=0.05):
        self.drivetrain = drivetrain
        self.vision = vision
        self.addRequirements(self.drivetrain)
        self.addRequirements(self.vision)
        self.fwddir = fwddir
        self.ps = ps
        self.targetlat = targetlat
        self.latpid = latpid
        self.spd = spd
    
    def isvalid(self) -> bool: return not self.tagid == 0

    def initialize(self):
        self.odometry = self.drivetrain.create_odometry()
        self.tagid = 0
        if ready := self.vision():
            self.tagid = ready[0]
            # Don't touch
            adjusted = geometry.Pose2d(-ready[1].X(), ready[1].Y(), ready[1].rotation())
            self.odometry.resetPose(adjusted)
        else: print("Invalid Command: No Tag")
        self.turn_controller = controller.PIDController(0.013, 0, 0)
        self.turn_controller.enableContinuousInput(0, 360)
        self.turn_controller.setTolerance(1)
        self.lat_pid = controller.PIDController(self.latpid, 0, 0)
    def execute(self):
        self.drivetrain.update_odo(self.odometry)
        if not self.isvalid(): return
        # Angular
        yaw_clamped = 0
        desired_angle = angle_lookup_table[self.tagid]
        current_angle = self.drivetrain.gyro()
        yaw_correction = -self.turn_controller.calculate(current_angle, desired_angle)
        yaw_clamped = control.clamp_mag(0.2, yaw_correction)

        # Manual Translate Offset
        fwd = drive.Polar(
            self.spd, self.drivetrain.gyro() + self.fwddir
        )
        cont = self.ps()

        # Update the known position(Currently Nothing)
        if detect := self.vision():
            tagid = detect[0]
            pose = detect[1]
            # Only update if we see a tag, we can update maybe
            if tagid == self.tagid:
                adjusted = geometry.Pose2d(-pose.X(), pose.Y(), pose.rotation())
                self.odometry.resetPose(adjusted)

        # Apply a lateral correction
        current_lat = -self.odometry.getPose().X()
        ncoms.dbg_tab.putNumber("Currentlat", current_lat)
        xcor = control.clamp_mag(self.max_blackhole_effect, self.lat_pid.calculate(current_lat, self.targetlat))
        blackhole = drive.Polar(
            xcor, self.drivetrain.gyro()
        )

        # Final Mix
        trans = drive.mix_polar(drive.mix_polar(fwd, cont), blackhole)
        self.drivetrain.polar_drive(trans, yaw_clamped, False)
    
    def end(self, _interrupted: bool):
        self.drivetrain.polar_drive(drive.Polar(0, 0), 0, True)