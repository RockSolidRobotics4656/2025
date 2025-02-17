import commands2
import wpilib
import ncoms

class Funnel(commands2.Subsystem):
    def __init__(self, dio: int):
        self.proximity = wpilib.DigitalInput(dio)
    
    def is_on_target(self) -> bool:
        return not self.proximity.get()
    
    def periodic(self):
        ncoms.funn_tab.putBoolean("Aligned", self.is_on_target())