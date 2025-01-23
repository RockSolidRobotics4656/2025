import wpilib
import wpimath
import commands2
import mathutil
import math

import wpimath.geometry
import wpimath.kinematics
import drive
import ncoms

class Continuity:
    def __init__(self):
        self.xbox = commands2.button.CommandXboxController(0)
        self.drivetrain = drive.SwerveDrive()
        self.configure_bindings()

    def configure_bindings(self):
        get_xbox_angle = lambda: math.degrees(math.atan2(
            -self.xbox.getRightY(), self.xbox.getRightX()
        )) 
        get_xbox_mag = lambda: mathutil.distance(0, 0, self.xbox.getRightX(), self.xbox.getRightY()) * 0.3
        self.drivetrain.setDefaultCommand(
            commands2.ParallelCommandGroup(
                self.drivetrain.polar_drive(get_xbox_angle, get_xbox_mag),
                self.drivetrain.telemetry(ncoms.drtelem_tab),
                commands2.RunCommand(lambda: ncoms.drtelem_tab.putNumber("xbox", get_xbox_angle()))
            )
        )

    def get_auto(self) -> commands2.Command:
        return commands2.ParallelCommandGroup(
            commands2.PrintCommand("Hello, World(Driver Station)!"),
            self.drivetrain.telemetry(ncoms.drtelem_tab)
        )
