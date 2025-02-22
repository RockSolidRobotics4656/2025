from typing import *
import wpilib
import commands2
import continuity


class Robot(commands2.TimedCommandRobot):
    auto = None
    def robotInit(self):
        
        self.container = continuity.Continuity()
    def robotPeriodic(self):
        commands2.CommandScheduler.getInstance().run()

    def autonomousInit(self):
        self.auto = self.container.get_auto()
        if self.auto:
            self.auto.schedule()
    def teleopInit(self):
        if self.auto:
            self.auto.cancel()
        self.container.get_tele().schedule()
    
    def testInit(): pass
    def disabledInit(self): pass