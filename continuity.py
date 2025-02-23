import wpilib
import wpimath
import commands2
import wpimath.filter
import mathutil
import math
from wpimath import applyDeadband

import aprilalign2
import drive
import elevate
import action
import april
import depo
import move
import const
import lock
import funnel

def dbnc(x: commands2.button.Trigger) -> commands2.button.Trigger:
    return x.debounce(0.1, wpimath.filter.Debouncer.DebounceType.kBoth)

class Continuity:
    def __init__(self):
        # Subsystems
        self.xbox = commands2.button.CommandXboxController(0)
        self.drivetrain = drive.SwerveDrive(enc_off=const.enc_offsets)
        self.wheels = depo.DepositorWheels(const.janky, 15, 1)
        self.wrist = depo.DepositorWrist(const.janky, 16, 0, enc=(2, 3))
        self.elevator = elevate.Elevator(13, 14, 4)
        self.fvision = april.VisionSystem("front", True)
        self.bvision = april.VisionSystem("rear", True)
        self.lock = lock.ClimbLock(6)
        self.funnel = funnel.Funnel(5)

        # Modes
        self.test_mode(self.xbox)

    def xbox_warning(self):
        return commands2.StartEndCommand(
            lambda: self.xbox.setRumble(wpilib.XboxController.RumbleType.kBothRumble, 1.0),
            lambda: self.xbox.setRumble(wpilib.XboxController.RumbleType.kBothRumble, 0.0),
        ).withTimeout(0.7)

    def get_xbox_turner(self) -> float:
        return wpimath.applyDeadband(self.xbox.getLeftX(), 0.05)
    def get_control(self) -> drive.Polar:
            angle = math.degrees(math.atan2(
                -self.xbox.getRightY(), self.xbox.getRightX()
            )) 
            mag = wpimath.applyDeadband(mathutil.distance(0, 0, self.xbox.getRightX(), self.xbox.getRightY()), 0.05)
            return drive.Polar(mag, angle)

    def test_mode(self, controller: commands2.button.CommandXboxController):
        self.drivetrain.setDefaultCommand(
            self.drivetrain.controller_drive(self.get_control, self.get_xbox_turner),
        )
        controller.y().whileTrue(move.Move(self.drivetrain,
            wpimath.geometry.Translation2d(0.0, 1.0), None))
        controller.b().whileTrue(move.Move(self.drivetrain,
            wpimath.geometry.Translation2d(1.0, 0.0), 315))
        controller.x().whileTrue(move.Move(self.drivetrain,
            wpimath.geometry.Translation2d(-1.0, 0.0), 45))
        controller.a().whileTrue(move.Move(self.drivetrain,
            wpimath.geometry.Translation2d(0.0, 0.0), 180))
        """
        controller.leftBumper().whileTrue(
            aprilalign2.reef_left(self.fvision, self.drivetrain, self.get_control)
        )
        controller.rightBumper().whileTrue(
            aprilalign2.reef_right(self.fvision, self.drivetrain, self.get_control)
        )
        """
        controller.leftBumper().whileTrue(
            aprilalign2.reef_align(self.fvision, self.drivetrain, self.get_control)
        )

    def dumb_mode(self, controller: commands2.button.CommandXboxController):
        if True: # Enable Drivetrain
            self.drivetrain.setDefaultCommand(
                self.drivetrain.controller_drive(self.get_control, self.get_xbox_turner),
            )
        if True:
            controller.povUp().whileTrue(self.elevator.test(0.2))
            controller.povDown().whileTrue(self.elevator.test(-0.1))
            controller.povLeft().whileTrue(self.elevator.test(-0.45))
        if True:
            controller.y().whileTrue(self.wrist.test(0.35))
            controller.a().whileTrue(self.wrist.test(-0.35))
        if True:
            controller.rightTrigger().onTrue(self.wheels.pickup())
            controller.leftTrigger().onTrue(self.wheels.deposite())

    def smart_mode(self, controller: commands2.button.CommandXboxController):
        if True: # Enable Warnings
            self.wheels.trigger().onTrue(self.xbox_warning())
            self.funnel.trigger().onTrue(self.xbox_warning())
        if True: # Enable Drivetrain
            self.drivetrain.setDefaultCommand(
                self.drivetrain.controller_drive(self.get_control, self.get_xbox_turner),
            )
        if True: # Enable Elevator
            self.elevator.setDefaultCommand(self.elevator.update())
        if True: # Enable Wrist
            self.wrist.setDefaultCommand(self.wrist.update())
        if True: # Enable Pickup
            controller.rightTrigger().whileTrue(action.forward(self.drivetrain, 270, 0.1))
            controller.rightTrigger().onFalse(
                    action.stash_coral(self.elevator, self.wrist, self.wheels)
                )
        if True: # Enable L Setpoints
            controller.povUp().onTrue(action.goto_l4(self.fvision, self.get_control, lambda: not controller.povUp().getAsBoolean(),  self.wheels, self.drivetrain, self.elevator, self.wrist))
            controller.povLeft().onTrue(action.goto_l1(self.fvision, self.get_control, lambda: not controller.povLeft().getAsBoolean(),  self.wheels, self.drivetrain, self.elevator, self.wrist))
            controller.povRight().onTrue(action.goto_l3(self.fvision, self.get_control, lambda: not controller.povRight().getAsBoolean(), self.wheels, self.drivetrain, self.elevator, self.wrist))
            controller.povDown().onTrue(action.goto_l2(self.fvision, self.get_control, lambda: not controller.povDown().getAsBoolean(), self.wheels, self.drivetrain, self.elevator, self.wrist))
        if True: # Enable Cage
            controller.leftTrigger().onTrue(action.upcage(self.elevator, self.wrist))
            controller.leftTrigger().onFalse(action.downcage(self.elevator, self.wrist, self.lock))
        if True: # Algae Setpoint
            controller.a().onTrue(action.low_algae(self.elevator, self.wrist, self.wheels))
            controller.b().onTrue(action.high_algae(self.elevator, self.wrist, self.wheels))
            controller.x().onFalse(action.receive(self.elevator, self.wrist))
    

    def get_tele(self) -> commands2.Command:
        return commands2.ParallelCommandGroup(
            action.receive(self.elevator, self.wrist),
            self.lock.unlock(),
            )
    def get_auto(self) -> commands2.Command:
        return commands2.ParallelCommandGroup(
            commands2.PrintCommand("Hello, World(Driver Station)!")
        )
