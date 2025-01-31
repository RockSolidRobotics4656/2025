from typing import *
import commands2
import wpimath.controller
import encoder
import wpilib
import phoenix5
import wpimath

class DepositorWrist(commands2.Subsystem):
    def __init__(self, id: int, limit_slot: int, enc: Tuple[int, int]):
        super().__init__()
        self.wrist_motor = phoenix5.VictorSPX(id)
        self.switch = wpilib.DigitalInput(limit_slot)
        tmp = wpilib.Encoder(enc[0], enc[1])
        self.encoder = encoder.EncoderAdapter(tmp.get)
        self.encoder.set_ticks_per_unit(-600/180)
        self.controller = wpimath.controller.PIDController(0.1, 0, 0)

    # Unsafe Operation / Unchecked
    def _move_raw(self, speed: float) -> None:
        self.wrist_motor.set(phoenix5.VictorSPXControlMode.PercentOutput, speed),
    
    def move(self, speed: float) -> None:
        if speed > 0 and self.is_home():
            speed *= 0
            self.encoder.reset()
        self._move_raw(speed)
    
    def test(self, speed: float) -> commands2.Command:
        return commands2.RunCommand(
            lambda: self.move(speed),
            lambda: self.move(0),
            self
        )
    
    def is_home(self) -> bool:
        return self.switch.get()

    def home(self) -> commands2.Command:
        return commands2.RunCommand(
            lambda: self.move(0.2)
            ).until(self.is_home)

    def telemetry(self, telem) -> commands2.Command:
        def tel_func():
            val = self.switch.get()
            telem.putBoolean("Wrist Limit", val)
            telem.putNumber("Wrist Value", self.encoder())
        return commands2.RunCommand(tel_func, self) ##BUG:!!!!!!!!!! Fix this should not depend on self

class DepositorWheels(commands2.Subsystem):
    def __init__(self, id: int, limit_slot: int):
        super().__init__()
        self.wheel_motor = phoenix5.VictorSPX(id)
        self.switch = wpilib.DigitalInput(limit_slot)
    def intake(self, speed: float) -> commands2.Command:
        return self.move_wheels(-speed)
    def eject(self, speed: float) -> commands2.Command:
        return self.move_wheels(speed)
    def move_wheels(self, speed: float) -> commands2.Command:
        return commands2.StartEndCommand(
            lambda: self.wheel_motor.set(phoenix5.VictorSPXControlMode.PercentOutput, speed),
            lambda: self.wheel_motor.set(phoenix5.VictorSPXControlMode.PercentOutput, 0), 
            self
        )
    def is_queued(self) -> bool:
        return not self.switch.get()
    def pickup(self, speed: float) -> commands2.Command:
        return self.intake(speed).until(self.is_queued).withTimeout(5.0)
    def deposite(self, speed: float) -> commands2.Command:
        return self.eject(speed).withTimeout(5.0)
    def telemetry(self, telem) -> commands2.Command:
        # BUG::: Should not depend on self - this is temporary
        return commands2.RunCommand(
            lambda: telem.putBoolean("Queued", self.is_queued()),
            self
        )