from typing import *
import commands2
import ncoms
import functools
import drive
import lock
import april
import aprilalign2
import elevate
import depo
import move
import functools
import action2
import funnel
import const
import position
from wpimath.geometry import *

def auto_start(p: position.Position) -> commands2.Command:
    return commands2.SequentialCommandGroup(
        p.wrist.fake_home(),
        p.elevator.home(),
    )


void = lambda: drive.Polar(0, 0)
def get_autonomous(d: drive.SwerveDrive, p: position.Position,
    wh: depo.DepositorWheels, f: funnel.Funnel,
    fv: april.VisionSystem, bv: april.VisionSystem,
    lk: lock.ClimbLock) -> commands2.Command:
    return main_auto(d, p, wh, f, fv, bv, lk)

def main_auto(d: drive.SwerveDrive, p: position.Position,
    wh: depo.DepositorWheels, f: funnel.Funnel,
    fv: april.VisionSystem, bv: april.VisionSystem,
    lk: lock.ClimbLock) -> commands2.Command:
    return commands2.SequentialCommandGroup(
        commands2.PrintCommand("Homing!"),
        auto_start(p),
        commands2.PrintCommand("Driving to Reef!"),
        commands2.ParallelDeadlineGroup(
            move.Move(d, Translation2d(1.3, -4.9), 300, max_linear=1.0),
            p.stow(),
        ),
        commands2.PrintCommand("Scoring!"),
        action2.score_coral4(d, fv, p, wh, f, True),
        commands2.PrintCommand("Driving to Station!"),
        commands2.ParallelDeadlineGroup(
            move.Move(d, Translation2d(0.5, -7), 300, max_linear=1.0),
            p.acquire(),
        ),
        action2.approach_coral(p, wh, d).withTimeout(4.0),
        action2.retreat_coral(p, wh),
        commands2.PrintCommand("Driving to Reef!"),
        commands2.ParallelDeadlineGroup(
            move.Move(d, Translation2d(1.3, -4.9), 300, max_linear=1.0),
            p.stow(),
        ),
        commands2.PrintCommand("Scoring!"),
        action2.score_coral4(d, fv, p, wh, f, False),
    )