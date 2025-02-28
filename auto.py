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
import action
import funnel
import const

from wpimath.geometry import *

void = lambda: drive.Polar(0, 0)
def get_autonomous(d: drive.SwerveDrive, e: elevate.Elevator,
    wr: depo.DepositorWrist, wh: depo.DepositorWheels, f: funnel.Funnel,
    fv: april.VisionSystem, bv: april.VisionSystem, lk: lock.ClimbLock) -> commands2.Command:
    return simple_l2_auto(d, e, wr, f, wh, fv, lk)
    #return backup_auto(d, e, wr, f, wh, fv, lk)

def complex_auto(d: drive.SwerveDrive,
    e: elevate.Elevator, wr: depo.DepositorWrist, f: funnel.Funnel,
    wh: depo.DepositorWheels, fv: april.VisionSystem, lk: lock.ClimbLock) -> commands2.Command:
    startpoint = Pose2d(1.3, -5.3, Rotation2d.fromDegrees(300))
    endpoint = Pose2d(0.5, -7, Rotation2d.fromDegrees(300))
    return commands2.SequentialCommandGroup(
        action.fake_start(wr, e, lk),
        commands2.PrintCommand("Starting Move to Reef"),
        commands2.ParallelCommandGroup(
            move.Move(d, startpoint.translation(), startpoint.rotation().degrees()),
            action.receive(e, wr),
        ),
        commands2.PrintCommand("Scoring"),
        action.coral_w_sweep(fv, void, d, e, wr, const.l3_ext, const.l_wrist_angle, f, aprilalign2.reef_leftoff_sweep),
        action.deploy(d, e, wr, wh),
        commands2.PrintCommand("Starting Move to Human Station"),
        commands2.ParallelCommandGroup(
            move.Move(d, endpoint.translation(), endpoint.rotation().degrees()),
            action.receive(e, wr),
        ),
        action.forward(d, 270, 0.3).withTimeout(4),
        action.stash_coral(e, wr, wh),
        ###################### Left 3
        commands2.PrintCommand("Starting Move to Scoring Station"),
        move.Move(d, startpoint.translation(), startpoint.rotation().degrees()),
        commands2.PrintCommand("Scoring"),
        action.coral_w_sweep(fv, void, d, e, wr, const.l3_ext, const.l_wrist_angle, f, aprilalign2.reef_leftoff_sweep),
        action.deploy(d, e, wr, wh),
        commands2.PrintCommand("Starting Move to Human Station"),
        commands2.ParallelCommandGroup(
            move.Move(d, endpoint.translation(), endpoint.rotation().degrees()),
            action.receive(e, wr),
        ),
        action.forward(d, 270, 0.3).withTimeout(4),
        action.stash_coral(e, wr, wh),
    )

def backup_auto(d: drive.SwerveDrive,
    e: elevate.Elevator, wr: depo.DepositorWrist, f: funnel.Funnel,
    wh: depo.DepositorWheels, fv: april.VisionSystem, lk: lock.ClimbLock) -> commands2.Command:
    return move.Move(d, Translation2d(0, -3.5), None)

def simple_auto(d: drive.SwerveDrive,
    e: elevate.Elevator, wr: depo.DepositorWrist, f: funnel.Funnel,
    wh: depo.DepositorWheels, fv: april.VisionSystem, lk: lock.ClimbLock) -> commands2.Command:
    startpoint = Pose2d(1.3, -5.3, Rotation2d.fromDegrees(300))
    endpoint = Pose2d(0.5, -7, Rotation2d.fromDegrees(300))
    return commands2.SequentialCommandGroup(
        action.fake_start(wr, e, lk),
        commands2.PrintCommand("Starting Move to Reef"),
        commands2.ParallelCommandGroup(
            move.Move(d, startpoint.translation(), startpoint.rotation().degrees()),
            action.receive(e, wr),
        ),
        commands2.PrintCommand("Scoring"),
        action.coral_w_sweep(fv, void, d, e, wr, const.l3_ext, const.l_wrist_angle, f, aprilalign2.reef_leftoff_sweep),
        commands2.PrintCommand("Deploying"),
        action.deploy(d, e, wr, wh),
        commands2.PrintCommand("Starting Move to Human Station"),
        commands2.ParallelCommandGroup(
            move.Move(d, endpoint.translation(), endpoint.rotation().degrees()),
            action.receive(e, wr),
        )
    )
def simple_l2_auto(d: drive.SwerveDrive,
    e: elevate.Elevator, wr: depo.DepositorWrist, f: funnel.Funnel,
    wh: depo.DepositorWheels, fv: april.VisionSystem, lk: lock.ClimbLock) -> commands2.Command:
    startpoint = Pose2d(1.5, -2.3, Rotation2d.fromDegrees(240))
    midpnt = Translation2d(0.5, -3)
    endpoint = Pose2d(0.5, -7, Rotation2d.fromDegrees(300))
    return commands2.SequentialCommandGroup(
        action.fake_start(wr, e, lk),
        commands2.PrintCommand("Starting Move to Reef"),
        commands2.ParallelCommandGroup(
            move.Move(d, startpoint.translation(), startpoint.rotation().degrees()),
            action.receive(e, wr),
        ),
        commands2.PrintCommand("Scoring"),
        action.coral_w_sweep(fv, void, d, e, wr, const.l2_ext, const.l_wrist_angle, f, aprilalign2.reef_leftoff_sweep),
        commands2.PrintCommand("Deploying"),
        action.deploy(d, e, wr, wh),
        commands2.PrintCommand("Starting Move to Human Station"),
        commands2.ParallelCommandGroup(
            move.Move(d, midpnt, None),
            action.receive(e, wr),
        ),
        move.Move(d, endpoint.translation(), endpoint.rotation().degrees()),
        action.forward(d, 270, 0.3).withTimeout(4),
        action.stash_coral(e, wr, wh),
    )