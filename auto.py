from typing import *
import commands2
import ncoms
import functools
import drive
import april
import aprilalign2
import elevate
import depo
import move
import functools
import action
import funnel
import const

from wpimath.geometry import *

def get_autonomous(d: drive.SwerveDrive, e: elevate.Elevator,
    wr: depo.DepositorWrist, wh: depo.DepositorWheels, f: funnel.Funnel,
    fv: april.VisionSystem, bv: april.VisionSystem) -> commands2.Command:
    start = Pose2d(-1.3, -5.3, Rotation2d.fromDegrees(60))
    end = Pose2d(-0.5, -7, Rotation2d.fromDegrees(60))
    return simple_auto(start, end, d, e, wr, f, wh, fv)

def backup_auto(startpoint: Pose2d, endpoint: Pose2d, d: drive.SwerveDrive,
    e: elevate.Elevator, wr: depo.DepositorWrist, f: funnel.Funnel,
    wh: depo.DepositorWheels, fv: april.VisionSystem) -> commands2.Command:
    return move.Move(d, Translation2d(0, -5), None)

def simple_auto(startpoint: Pose2d, endpoint: Pose2d, d: drive.SwerveDrive,
    e: elevate.Elevator, wr: depo.DepositorWrist, f: funnel.Funnel,
    wh: depo.DepositorWheels, fv: april.VisionSystem) -> commands2.Command:
    void = lambda: drive.Polar(0, 0)
    return commands2.SequentialCommandGroup(
        commands2.PrintCommand("Starting Move to Reef"),
        move.Move(d, startpoint.translation(), startpoint.rotation().degrees()),
        commands2.PrintCommand("Starting April Align"),
        action.coral_w_sweep(fv, void, d, e, wr, const.l3_ext, const.l_wrist_angle, f, aprilalign2.reef_leftoff_sweep),
        commands2.PrintCommand("Deploying"),
        action.deploy(d, e, wr, wh),
        commands2.PrintCommand("Starting Move to Human Station"),
        commands2.ParallelCommandGroup(
            move.Move(d, endpoint.translation(), endpoint.rotation().degrees()),
            action.receive(e, wr),
        )
    )