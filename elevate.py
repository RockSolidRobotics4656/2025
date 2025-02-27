import commands2
import ncoms
import rev
import wpilib
import wpimath
import wpimath.controller
import wpimath.trajectory
import encoder
import control

class Elevator(commands2.Subsystem):
    def __init__(self, ida: int, idb: int, swi_slot: int):
        super().__init__()
        self.motora = rev.SparkMax(ida, rev.SparkMax.MotorType.kBrushless)
        self.motorb = rev.SparkMax(idb, rev.SparkMax.MotorType.kBrushless)
        enca = self.motora.getEncoder()
        encb = self.motorb.getEncoder()
        func = lambda: enca.getPosition() + encb.getPosition()
        self.height = encoder.EncoderAdapter(func)
        self.height.set_ticks_per_unit(-110.0)
        self.switch = wpilib.DigitalInput(swi_slot)
        #self.controller = wpimath.controller.PIDController(5.0, 0, 0) # TODO: Tune
        self.controller = wpimath.controller.ProfiledPIDController(6.5, 0, 0,
            wpimath.trajectory.TrapezoidProfile.Constraints(1, 0.2))
        self.controller.setTolerance(0.005)
    
    def update(self, clamp = 0.6) -> commands2.Command:
        def up():
            measurement = self.height()
            correction = self.controller.calculate(measurement)
            self.move(control.clamp_mag(clamp, correction))
        return commands2.RunCommand(up, self)

    def test(self, speed: float) -> commands2.Command:
        return commands2.StartEndCommand(
            lambda: self.move(speed),
            lambda: self.move(0),
        )
    
    def at_setpoint(self) -> bool:
        return self.controller.atGoal()

    def set_setpoint(self, setpoint: float) -> commands2.Command:
        return commands2.InstantCommand(
            lambda: self.controller.setGoal(setpoint), self
        )

    def goto(self, setpoint: float) -> commands2.Command:
        return commands2.SequentialCommandGroup(
            self.set_setpoint(setpoint),
            self.update().until(self.at_setpoint),
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
    
    def home(self, dft=-.15) -> commands2.Command:
        return commands2.SequentialCommandGroup(
            commands2.RunCommand(
                lambda: self.move(dft),
                self
            ).until(self.is_bottomed),
            commands2.InstantCommand(lambda: self.height.reset(), self),
        ).withTimeout(5.0)

    def is_bottomed(self) -> bool:
        return self.switch.get()

    def periodic(self):
        ncoms.ele_tab.putNumber("EHeight", self.height()),
        ncoms.ele_tab.putNumber("THieght", float(self.controller.getGoal().position))
        ncoms.ele_tab.putBoolean("AtBottom", self.is_bottomed()),