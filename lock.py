import commands2
import wpilib

class ClimbLock(commands2.Subsystem):
    lock_pos = 0
    unlock_pos = 70
    tolerance = 3
    def __init__(self, dio: int):
        self.servo = wpilib.Servo(dio)
    def is_locked(self):
        return self.tolerance > abs(self.servo.getAngle()-self.lock_pos)
    def is_unlocked(self):
        return self.tolerance > abs(self.servo.getAngle()-self.unlock_pos)
    def unlock(self) -> commands2.Command:
        return commands2.SequentialCommandGroup(
            self.async_unlock(),
            commands2.WaitUntilCommand(self.is_unlocked)
        )
    def async_lock(self) -> commands2.Command:
        return commands2.InstantCommand(lambda: self.servo.setAngle(self.lock_pos), self)
    def async_unlock(self) -> commands2.Command:
        return commands2.InstantCommand(lambda: self.servo.setAngle(self.unlock_pos), self)