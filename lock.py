import commands2
import wpilib

class ClimbLock(commands2.Subsystem):
    lock_pos = 0
    unlock_pos = 180
    tolerance = 3
    def __init__(self, dio: int):
        self.servo = wpilib.Servo(dio)
    
    def is_locked(self):
        return 3 > abs(self.servo.getAngle()-self.lock_pos)
    def is_unlocked(self):
        return 3 > abs(self.servo.getAngle()-self.unlock_pos)
    def lock(self) -> commands2.Command:
        return commands2.InstantCommand(lambda: self.servo.setAngle(self.lock_pos))
    def unlock(self) -> commands2.Command:
        return commands2.InstantCommand(lambda: self.servo.setAngle(self.unlock_pos))