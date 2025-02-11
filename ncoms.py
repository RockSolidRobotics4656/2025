import itertools
import ntcore

networktables = ntcore.NetworkTableInstance.getDefault()
shuffleboard = networktables.getTable("Shuffleboard")

drtelem_tab = shuffleboard.getSubTable("4656 Driver Telemetry")
coproc_tab = shuffleboard.getSubTable("4656 Coprocessor")
ele_tab = shuffleboard.getSubTable("4656 Elevator")
wrist_tab = shuffleboard.getSubTable("4656 Wrist")
wheel_tab = shuffleboard.getSubTable("4656 Wheels")
dbg_tab = shuffleboard.getSubTable("4656 Debug")


_uname_iter = itertools.count(start=1, step=1)
def uname() -> str:
    return str(next(_uname_iter))