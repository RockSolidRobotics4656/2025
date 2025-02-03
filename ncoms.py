import itertools
import ntcore

networktables = ntcore.NetworkTableInstance.getDefault()
shuffleboard = networktables.getTable("Shuffleboard")
drtelem_tab = shuffleboard.getSubTable("4656 Driver Telemetry")
coproc_tab = shuffleboard.getSubTable("4656 Coprocessor")


_uname_iter = itertools.count(start=1, step=1)
def uname() -> str:
    return str(next(_uname_iter))