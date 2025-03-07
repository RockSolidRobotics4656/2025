from typing import *

import wpimath.filter
import ncoms
import commands2
import control
import wpimath.controller
import encoder
import wpilib
import phoenix5
import rev
import wpimath

TMP_RESET_ANGLE_AT_HOME = -30 # BUG, TMP, TODO: This is a temporary value for the elevator
class DepositorWrist(commands2.Subsystem):
    def __init__(self, janky: bool, id: int, limit_slot: int, enc: Tuple[int, int]):
        super().__init__()
        self.janky = janky
        if self.janky:
            self.wrist_motor = phoenix5.VictorSPX(id)
        else:
            self.wrist_motor = rev.SparkMax(id, rev.SparkMax.MotorType.kBrushed)
        self.switch = wpilib.DigitalInput(limit_slot)
        tmp = wpilib.Encoder(enc[0], enc[1])
        self.encoder = encoder.EncoderAdapter(tmp.get)
        self.encoder.set_ticks_per_unit(-710 / 225)
        # BUG: Fix with the height new 
        self.controller = wpimath.controller.PIDController(0.2, 0, 0)
        self.controller.setTolerance(1)


    # UNSAFE, BUG fakehome
    def fake_home(self) -> commands2.Command:
        return commands2.InstantCommand(self.encoder.reset(TMP_RESET_ANGLE_AT_HOME))
    # Unsafe Operation / Unchecked
    def _move_raw(self, speed: float) -> None:
        if self.janky:
            self.wrist_motor.set(phoenix5.VictorSPXControlMode.PercentOutput, -speed),
        else:
            self.wrist_motor.set(-speed)
    
    def move(self, speed: float) -> None:
        if speed < 0 and self.is_home():
            speed *= 0
            self.encoder.reset(TMP_RESET_ANGLE_AT_HOME)
        self._move_raw(speed)
    
    def update(self, clamp=0.8) -> commands2.Command:
        def up():
            measurement = self.encoder()
            correction = self.controller.calculate(measurement)
            self.move(control.clamp_mag(clamp, correction))
        return commands2.RunCommand(up, self)
    
    def test(self, speed: float) -> commands2.Command:
        return commands2.StartEndCommand(
            lambda: self.move(speed),
            lambda: self.move(0),
        )
    
    def at_setpoint(self) -> bool:
        return self.controller.atSetpoint()

    def set_setpoint(self, setpoint: float) -> commands2.Command:
        return commands2.InstantCommand(
            lambda: self.controller.setSetpoint(setpoint), self
        )

    def goto(self, setpoint: float, clamp=0.8) -> commands2.Command:
        return commands2.SequentialCommandGroup(
            self.set_setpoint(setpoint),
            self.update(clamp=clamp).until(self.at_setpoint),
        ).handleInterrupt(lambda: self.move(0))

    def is_home(self) -> bool:
        return not self.switch.get()

    def home(self) -> commands2.Command:
        return commands2.SequentialCommandGroup(
            commands2.RunCommand(
                lambda: self.move(-0.35), self
                ).until(self.is_home),
            commands2.InstantCommand(lambda: self.encoder.reset(TMP_RESET_ANGLE_AT_HOME), self),
        ).withTimeout(5.0)

    def periodic(self):
        ncoms.wrist_tab.putBoolean("Homed", self.is_home())
        ncoms.wrist_tab.putNumber("Wrist Value", self.encoder())

class DepositorWheels(commands2.Subsystem):
    def __init__(self, janky: bool, id: int, limit_slot: int):
        super().__init__()
        self.janky = janky
        if janky:
            self.wheel_motor = phoenix5.VictorSPX(id)
        else:
            self.wheel_motor = rev.SparkMax(id, rev.SparkMax.MotorType.kBrushed)
        self.switch = wpilib.DigitalInput(limit_slot)
    def intake(self, speed: float) -> commands2.Command:
        return self.move_wheels(-speed)
    def eject(self, speed: float) -> commands2.Command:
        return self.move_wheels(speed)
    def _raw(self, speed: float):
        if self.janky:
            self.wheel_motor.set(phoenix5.VictorSPXControlMode.PercentOutput, speed),
        else:
            self.wheel_motor.set(-speed)
    def move_wheels(self, speed: float) -> commands2.Command:
        return commands2.StartEndCommand(
            lambda: self._raw(speed),
            lambda: self._raw(0), 
            self
        )
    def is_queued(self) -> bool:
        return not self.switch.get()
    def trigger(self) -> commands2.button.Trigger:
        return commands2.button.Trigger(self.is_queued)
    def pickup(self) -> commands2.Command:
        return self.intake(1.0).until(self.is_queued).withTimeout(2.0)
    def deposite(self) -> commands2.Command:
        return self.eject(1.0).withTimeout(1.3)
    def periodic(self):
        ncoms.wheel_tab.putBoolean("Queued", self.is_queued())