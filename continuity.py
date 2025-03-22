import wpilib
import wpimath
import commands2
import wpimath.filter
import wpimath.geometry
import mathutil
import math
import auto
import action2
import position
import aprilalign2
import drive
import elevate
import action
import april
import depo
import move
import ncoms
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
        self.positional = position.Position(
            elevate.Elevator(13, 14, 4),
            depo.DepositorWrist(const.janky, 16, 0, enc=(2, 3))
        )
        self.fvision = april.VisionSystem("front", True)
        self.bvision = april.VisionSystem("rear", True)
        self.lock = lock.ClimbLock(6)
        self.funnel = funnel.Funnel(5)

        # Did you remember the fake start in teleop enable
        self.smart_mode(self.xbox)

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
        void = lambda: drive.Polar(0,0)
        controller.rightBumper().whileTrue(
            aprilalign2.Align(self.fvision, self.drivetrain, 90, void, aprilalign2.reef_middle_sweep, spd=0.2, latpid=0.5).until(self.drivetrain.is_front_grounded)
        )
        gobtn = commands2.button.Trigger(lambda: ncoms.dsprog_tab.getBoolean("start", False))
        gobtn.onChange(
            commands2.ParallelCommandGroup(
                commands2.PrintCommand("Starting Follow!"),
                move.TrajectoryMove(self.drivetrain, wpimath.geometry.Translation2d(0,0), None, 0.1, 0, 0.1, 0)
            )
        )

    def dumb_mode(self, controller: commands2.button.CommandXboxController):
        if True: # Enable Drivetrain
            self.drivetrain.setDefaultCommand(
                self.drivetrain.controller_drive(self.get_control, self.get_xbox_turner),
            )
        if True:
            controller.povUp().whileTrue(self.positional.elevator.test(0.2))
            controller.povDown().whileTrue(self.positional.elevator.test(-0.1))
            controller.povLeft().whileTrue(self.positional.elevator.test(-0.45))
        if True:
            controller.y().whileTrue(self.positional.wrist.test(0.35))
            controller.a().whileTrue(self.positional.wrist.test(-0.35))
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
        if True: # Enable Pickup
            controller.rightTrigger().whileTrue(action2.approach_coral(self.positional, self.wheels, self.drivetrain))
            controller.rightTrigger().onFalse(action2.retreat_coral(self.positional, self.wheels))
        if True: # Enable L Setpoints
            # Score L4 on the left
            controller.povUp().and_(controller.leftBumper()).onTrue(action2.score_coral4(self.drivetrain, self.fvision, self.positional, self.wheels, self.funnel, True))
            controller.povUp().and_(controller.rightBumper()).onTrue(action2.score_coral4(self.drivetrain, self.fvision, self.positional, self.wheels, self.funnel, False))
            # Score L3 on the left
            controller.povRight().and_(controller.leftBumper()).onTrue(action2.score_coral(self.drivetrain, self.fvision, self.positional, self.wheels, self.funnel, position.P_L3, True))
            controller.povRight().and_(controller.rightBumper()).onTrue(action2.score_coral(self.drivetrain, self.fvision, self.positional, self.wheels, self.funnel, position.P_L3, False))
            # Score L2 on the left
            controller.povDown().and_(controller.leftBumper()).onTrue(action2.score_coral(self.drivetrain, self.fvision, self.positional, self.wheels, self.funnel, position.P_L2, True))
            controller.povDown().and_(controller.rightBumper()).onTrue(action2.score_coral(self.drivetrain, self.fvision, self.positional, self.wheels, self.funnel, position.P_L2, False))
        if True: # Enable Algae
            controller.a().onTrue(action2.release_low_algae(self.drivetrain, self.fvision, self.positional, self.wheels))
            controller.b().onTrue(action2.release_high_algae(self.drivetrain, self.fvision, self.positional, self.wheels))
            controller.x().onTrue(self.positional.stow())
            #controller.y().onTrue(action2.release_low_algae(self.drivetrain, self.fvision, self.positional, self.wheels))
        if True: # Enable Climb
            controller.leftTrigger().onTrue(action2.upcage(self.positional))
            controller.leftTrigger().onFalse(action2.downcage(self.positional, self.lock))

    def get_tele(self) -> commands2.Command:
        return self.lock.async_unlock()
    def get_auto(self) -> commands2.Command:
        return auto.get_autonomous(self.drivetrain, self.elevator, self.wrist, self.wheels, self.funnel, self.fvision, self.bvision, self.lock)
