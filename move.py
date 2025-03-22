from typing import *
import math
import commands2
from wpimath import geometry, filter, controller 
import wpimath
import drive
import control
import ncoms
import trajectory
import abc

def normangle(x: float) -> float:
    while x <= 0: x += 360
    return x % 360

class GeneralMove(commands2.Command):
    def __init__(self, d: drive.SwerveDrive, position: geometry.Translation2d, angle: Optional[float],
            max_linear=0.8, tol_linear=0.05, max_turn=0.2, tol_turn=1):
        super().__init__()
        self.drive = d
        self.addRequirements(self.drive)
        self.destination = position
        self.fangle = angle
        self.max_linear = max_linear
        self.tol_linear = tol_linear
        self.max_turn = max_turn
        self.tol_turn = tol_turn
    
    def initialize(self):
        self.linear_pid = controller.ProfiledPIDController(1.0, 0, 0,
            wpimath.trajectory.TrapezoidProfile.Constraints(1000, 0.2))
        self.linear_pid.setTolerance(self.tol_linear)
        self.angular_pid = controller.PIDController(0.04, 0, 0)
        self.angular_pid.enableContinuousInput(0, 360)
        self.angular_pid.setTolerance(self.tol_turn)
        self.postinit()
    def postinit(self): pass
    
    def dst(self) -> float: raise NotImplemented("Implement for the specific move case")
    def update_carrot(self) -> None: pass
    def execute(self):
        self.update_carrot()
        pose = self.drive.odometry.getPose()

        # Linear 
        dx = self.destination.X() - pose.X()
        dy = self.destination.Y() - pose.Y()
        linear_dst = self.dst()
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

class Move(GeneralMove):
    def dst(self) -> float:
        pose = self.drive.odometry.getPose()
        dx = self.destination.X() - pose.X()
        dy = self.destination.Y() - pose.Y()
        return math.sqrt(dx**2 + dy**2)

class TrajectoryMove(GeneralMove):
    def __init__(self, d: drive.SwerveDrive, position: geometry.Translation2d, angle: Optional[float],
                 linear_spd: float, turn_spd: float,
                 linear_tol=0.05, turn_tol=1):
        super().__init__(d, position, angle, linear_spd, linear_tol, turn_spd, turn_tol)
    def postinit(self):
        self.traj = ncoms.get_trajectory()
    def dst(self) -> float:
        current_position = self.drive.odometry.getPose().translation()
        t = self.traj.find_closest_t_value(trajectory.Point(current_position.X(), current_position.Y()))
        total_len = self.traj.total_length()
        return total_len - t
    def update_carrot(self) -> None:
        current_position = self.drive.odometry.getPose().translation()
        t = self.traj.find_closest_t_value(trajectory.Point(current_position.X(), current_position.Y()))
        follow_t = self.traj.generate_carrot(t)
        carrot = self.traj.sample(follow_t)
        self.destination = geometry.Translation2d(carrot.x, carrot.y)