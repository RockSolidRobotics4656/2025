import commands2
import rev

class Elevator(commands2.Subsystem):
    def __init__(self, ida: int, idb: int):
        super().__init__()
        self.motora = rev.SparkMax(ida, rev.SparkMax.MotorType.kBrushless)
        self.motorb = rev.SparkMax(idb, rev.SparkMax.MotorType.kBrushless)
    
    def _move_raw(self, speed: float) -> commands2.Command:
        def mv(speed: float):
            self.motora.set(speed)
            self.motorb.set(-speed)
        return commands2.StartEndCommand(lambda: mv(speed), lambda: mv(0), self)