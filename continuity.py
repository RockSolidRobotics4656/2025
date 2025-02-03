import wpilib
import wpimath
import commands2
import mathutil
import math

import wpimath.geometry
import wpimath.kinematics
import drive
import elevate
import april
import depo
import ncoms

class Continuity:
    def __init__(self):
        self.xbox = commands2.button.CommandXboxController(0)
        self.drivetrain = drive.SwerveDrive()
        self.wheels = depo.DepositorWheels(15, 1)
        self.wrist = depo.DepositorWrist(16, 0, enc=(2,3))
        self.elevator = elevate.Elevator(13, 14, 4)
        self.configure_bindings()

    def configure_bindings(self):
        ncoms.drtelem_tab.putString("Debug", "What")
        if True: # Enable Drivetrain
            get_xbox_angle = lambda: math.degrees(math.atan2(
                -self.xbox.getRightY(), self.xbox.getRightX()
            )) 
            get_xbox_mag = lambda: mathutil.distance(0, 0, self.xbox.getRightX(), self.xbox.getRightY()) * 0.3
            get_xbox_turner = lambda: -self.xbox.getLeftX()
            self.drivetrain.setDefaultCommand(
                commands2.ParallelCommandGroup(
                    self.drivetrain.polar_drive(get_xbox_angle, get_xbox_mag, get_xbox_turner),
                    self.drivetrain.telemetry(ncoms.drtelem_tab),
                    commands2.RunCommand(lambda: ncoms.drtelem_tab.putNumber("xbox", get_xbox_angle()))
                )
            )
        if True: # Enable Wrist
            self.wrist.setDefaultCommand(
                self.wrist.telemetry(ncoms.drtelem_tab)
            )
            self.xbox.a().whileTrue(self.wrist.test(0.3))
            self.xbox.y().whileTrue(self.wrist.test(-0.3))
            self.xbox.b().onTrue(self.wrist.home())
        
        if True: # Enable Wheels
            self.wheels.setDefaultCommand(
                self.wheels.telemetry(ncoms.drtelem_tab)
            )
            self.xbox.leftTrigger().whileTrue(
                self.wheels.eject(0.8)
            )
            self.xbox.rightTrigger().whileTrue(
                self.wheels.intake(0.8)
            )

        if True: # Enable Elevator
            self.elevator.setDefaultCommand(
                commands2.ParallelCommandGroup(
                    self.elevator.telemetry(ncoms.drtelem_tab),
                    commands2.RunCommand(
                        lambda: april.test(ncoms.coproc_tab)
                    ) # cheat
                )
            )
            self.xbox.povUp().whileTrue(
                self.elevator.set_setpoint(0.0)
            )
            self.xbox.povRight().whileTrue(
                self.elevator.set_setpoint(0.1)
            )
            self.xbox.povDown().whileTrue(
                self.elevator.set_setpoint(0.2)
            )
        

    def get_auto(self) -> commands2.Command:
        return commands2.ParallelCommandGroup(
            commands2.PrintCommand("Hello, World(Driver Station)!"),
            self.drivetrain.telemetry(ncoms.drtelem_tab)
        )
