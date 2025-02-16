import wpilib
import wpimath
import commands2
import mathutil
import math

import wpimath.geometry
import wpimath.kinematics
import drive
import elevate
import action
import april
import depo
import ncoms
import move

from wpimath import applyDeadband

debug = False

class Continuity:
    def __init__(self):
        self.xbox = commands2.button.CommandXboxController(0)
        self.drivetrain = drive.SwerveDrive()
        self.global_odo = self.drivetrain.create_odometry()
        self.wheels = depo.DepositorWheels(15, 1)
        self.wrist = depo.DepositorWrist(16, 0, enc=(2, 3))
        self.elevator = elevate.Elevator(13, 14, 4)
        self.vision = april.VisionSystem("front")

        self.configure_bindings()

    def telemetry(self, telem) -> commands2.Command:
        def odo_telem():
            pose = self.global_odo.getPose()
            telem.putNumber("gx", pose.X())
            telem.putNumber("gy", pose.Y())
            telem.putNumber("ga", pose.rotation().degrees())
        return commands2.RunCommand(odo_telem)
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
            self.drivetrain.setDefaultCommand(commands2.ParallelCommandGroup(
                self.drivetrain.controller_drive(get_control, get_xbox_turner),
                commands2.RunCommand(lambda: self.drivetrain.update_odo(self.global_odo))
            ))

            self.xbox.b().onTrue(move.forward(self.drivetrain))
            self.xbox.a().onTrue(move.tmpapalign(self.vision, self.drivetrain))

        if True and not debug: # Enable Elevator
            self.elevator.setDefaultCommand(self.elevator.update())
        if True and debug:
            self.xbox.povUp().whileTrue(self.elevator.test(0.2))
            self.xbox.povDown().whileTrue(self.elevator.test(-0.1))

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
        
        if True and not debug: #Enable cage
            self.xbox.leftTrigger().onTrue(action.upcage(self.elevator, self.wrist))
            self.xbox.leftTrigger().onFalse(action.downcage(self.elevator, self.wrist))
        
        if True:
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
