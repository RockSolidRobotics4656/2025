import wpilib
import wpimath
import commands2
import mathutil
import math
from wpimath import applyDeadband

import wpimath.geometry
import wpimath.kinematics
import drive
import elevate
import action
import april
import depo
import ncoms
import move
import const
import lock
import funnel


debug = False

class Continuity:
    def __init__(self):
        self.xbox = commands2.button.CommandXboxController(0)
        self.drivetrain = drive.SwerveDrive(enc_off=const.enc_offsets)
        self.wheels = depo.DepositorWheels(const.janky, 15, 1)
        self.wrist = depo.DepositorWrist(const.janky, 16, 0, enc=(2, 3))
        self.elevator = elevate.Elevator(13, 14, 4)
        self.vision = april.VisionSystem("front")
        self.lock = lock.ClimbLock(6)
        self.funnel = funnel.Funnel(5)

        self.configure_bindings()

    def configure_bindings(self):
        ncoms.drtelem_tab.putString("Debug", "No")
        if True: # Enable Drivetrain
            def get_control():
                    angle = math.degrees(math.atan2(
                        -self.xbox.getRightY(), self.xbox.getRightX()
                    )) 
                    mag = wpimath.applyDeadband(mathutil.distance(0, 0, self.xbox.getRightX(), self.xbox.getRightY()) * 0.3, 0.05)
                    return drive.Polar(mag, angle)
            get_xbox_turner = lambda: wpimath.applyDeadband(-self.xbox.getLeftX(), 0.05)
            self.drivetrain.setDefaultCommand(
                self.drivetrain.controller_drive(get_control, get_xbox_turner),
            )

            # Major TODOS
            self.xbox.b().onTrue(move.RelativeMove(self.drivetrain,
                wpimath.geometry.Translation2d(0.5, 0.5)))
            self.xbox.y().onTrue(move.AbsoluteRotate(self.drivetrain, 45))
            self.xbox.a().onTrue(move.tmpapalign(self.vision, self.drivetrain))

        if True and not debug: # Enable Elevator
            self.elevator.setDefaultCommand(self.elevator.update())
        if True and debug:
            self.xbox.povUp().whileTrue(self.elevator.test(0.2))
            self.xbox.povDown().whileTrue(self.elevator.test(-0.1))
            self.xbox.povLeft().whileTrue(self.elevator.test(-0.45))

        if True and not debug: # Enable Wrist
            self.wrist.setDefaultCommand(self.wrist.update())
        if True and debug:
            self.xbox.y().whileTrue(self.wrist.test(0.2))
            self.xbox.a().whileTrue(self.wrist.test(-0.2))
        
        if True: # Enable Wheels
            self.xbox.rightTrigger().onTrue(self.wheels.pickup())
        if True and debug:
            self.xbox.leftTrigger().onTrue(self.wheels.deposite())
        
        if True and not debug: # Enable setpoints
            self.xbox.povLeft().onTrue(action.goto_l1(self.elevator, self.wrist))
            self.xbox.povUp().onTrue(action.goto_l4(self.elevator, self.wrist))
            self.xbox.povRight().onTrue(action.goto_l3(self.elevator, self.wrist))
            self.xbox.povDown().onTrue(action.goto_l2(self.elevator, self.wrist))

            either = self.xbox.povLeft() or self.xbox.povUp() or self.xbox.povRight() or self.xbox.povDown()
            either.onFalse(action.deploy(self.elevator, self.wrist, self.wheels))

            self.xbox.leftTrigger().onTrue(action.upcage(self.elevator, self.wrist))
            self.xbox.leftTrigger().onFalse(action.downcage(self.elevator, self.wrist))
        
        if True:
            self.xbox.leftBumper().onTrue(self.lock.lock())
            self.xbox.rightBumper().onTrue(self.lock.unlock())
        if False: # Enable Polling the Vision Findings
            self.xbox.rightBumper().onTrue(
                commands2.InstantCommand(self.vision)
            )
        
    
    def teleop_start(self) -> commands2.Command:
        return commands2.SequentialCommandGroup(
            self.elevator.home(),
            action.receive(self.elevator, self.wrist)
            )
        
    def get_auto(self) -> commands2.Command:
        return commands2.ParallelCommandGroup(
            commands2.PrintCommand("Hello, World(Driver Station)!")
        )
