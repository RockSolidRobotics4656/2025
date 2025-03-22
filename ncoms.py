import itertools
import ntcore
from wpimath import geometry
import trajectory

networktables = ntcore.NetworkTableInstance.getDefault()
shuffleboard = networktables.getTable("Shuffleboard")

drtelem_tab = shuffleboard.getSubTable("4656 Driver")
coproc_tab = shuffleboard.getSubTable("4656 Coprocessor")
ele_tab = shuffleboard.getSubTable("4656 Elevator")
wrist_tab = shuffleboard.getSubTable("4656 Wrist")
wheel_tab = shuffleboard.getSubTable("4656 Wheels")
dbg_tab = shuffleboard.getSubTable("4656 Debug")
funn_tab = shuffleboard.getSubTable("4656 Funnel")
aut_tab = shuffleboard.getSubTable("4656 Autonomous")
dsprog_tab = shuffleboard.getSubTable("4656 DS Program")

def get_trajectory():
    string = dsprog_tab.getString("trajectory", "")
    return trajectory.deserialize(string)

_uname_iter = itertools.count(start=1, step=1)
# Generate a unique name
def uname() -> str:
    return str(next(_uname_iter))