import commands2
import rev
import wpilib
import wpimath
import wpimath.controller
import encoder
import control

class Elevator(commands2.Subsystem):
    def __init__(self, ida: int, idb: int, swi_slot: int):
        super().__init__()
        self.motora = rev.SparkMax(ida, rev.SparkMax.MotorType.kBrushless)
        self.motorb = rev.SparkMax(idb, rev.SparkMax.MotorType.kBrushless)
        enca = self.motora.getEncoder()
        encb = self.motorb.getEncoder()
        func = lambda: enca.getPosition() + -encb.getPosition()
        self.height = encoder.EncoderAdapter(func)
        self.height.set_ticks_per_unit(45.66)
        self.switch = wpilib.DigitalInput(swi_slot)
        self.controller = wpimath.controller.PIDController(.1, 0, 0) # TODO: Tune
        self.controller.setTolerance(0.01)
    
    """
    def periodic(self) -> None:
        measurement = self.height()
        correction = self.controller.calculate(measurement)
        clamp = 0.15
        self.move(control.clamp_mag(clamp, correction))
    """

    def test(self, speed: float) -> commands2.Command:
        return commands2.StartEndCommand(
            lambda: self.move(speed),
            lambda: self.move(0),
            self
        )
    
    def at_setpoint(self) -> bool:
        return self.controller.atSetpoint()
    def set_setpoint(self, setpoint: float) -> commands2.Command:
        return commands2.InstantCommand(
            lambda: self.controller.setSetpoint(setpoint)
        )
    def goto(self, setpoint: float) -> commands2.Command:
        return commands2.SequentialCommandGroup(
            self.set_setpoint(setpoint),
            commands2.WaitUntilCommand(self.at_setpoint),
            commands2.InstantCommand(lambda: self.move(0))
        )
    
    def _move_raw(self, speed: float) -> None:
        self.motora.set(-speed)
        self.motorb.set(-speed)
    
    def move(self, speed: float) -> None:
        if self.is_bottomed() and speed < 0:
            speed *= 0
            self.height.reset()
        if speed > 0 and self.height() > 1:
            speed *= 0
        self._move_raw(speed)
    
    def home(self) -> commands2.Command:
        return commands2.RunCommand(
            lambda: self.move(-0.1),
            self
        ).until(self.is_bottomed)
    def is_bottomed(self) -> bool:
        return self.switch.get()
    def telemetry(self, telem) -> commands2.Command:
        def log():
            telem.putNumber("EHeight", self.height()),
            telem.putBoolean("AtBottom", self.is_bottomed()),
        return commands2.RunCommand(
            log, self
        )