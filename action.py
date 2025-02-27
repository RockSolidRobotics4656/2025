from typing import *
import commands2
import functools
import depo
import elevate
import lock
import drive
import const
import aprilalign2
import april
import funnel

def fake_start(wrist: depo.DepositorWrist, elevator: elevate.Elevator, lk: lock.ClimbLock) -> commands2.Command:
    # This is a temporary fix
    return wrist.fake_home().andThen(commands2.ParallelCommandGroup(
        receive(elevator, wrist),
        lk.unlock(),
        ))

# Algae
def low_algae(ele: elevate.Elevator, wrist: depo.DepositorWrist, wheels: depo.DepositorWheels) -> commands2.Command:
    return commands2.ParallelCommandGroup(
        commands2.RepeatCommand(ele.goto(0)),
        commands2.RepeatCommand(wrist.goto(120)),
        wheels.eject(1.0),
    )
def high_algae(ele: elevate.Elevator, wrist: depo.DepositorWrist, wheels: depo.DepositorWheels) -> commands2.Command:
    return commands2.ParallelCommandGroup(
        commands2.RepeatCommand(ele.goto(0.3)),
        commands2.RepeatCommand(wrist.goto(120)),
        wheels.eject(1.0),
    )

# POSITIONS
# BUG: impl gwithpause
def coral_wo_sweep(v: april.VisionSystem, ps: Callable[[], drive.Polar], 
        d: drive.SwerveDrive, ele: elevate.Elevator, wrist: depo.DepositorWrist,
        eheight: float, wangle: float, fun: Callable[[], bool], xoffset: float) -> commands2.Command:
        return commands2.ParallelDeadlineGroup(
            commands2.WaitUntilCommand(fun), reef(v, ps, d, ele, wrist, eheight, wangle, xoffset)
            )
def coral_w_sweep(v: april.VisionSystem, ps: Callable[[], drive.Polar], 
        d: drive.SwerveDrive, ele: elevate.Elevator, wrist: depo.DepositorWrist,
        eheight: float, wangle: float, fun: funnel.Funnel, xoffset: float) -> commands2.Command:
        return commands2.SequentialCommandGroup(
            reef(v, ps, d, ele, wrist, eheight, wangle, xoffset, spd=0.08, latpid=0.25).withTimeout(6.0),
            forward(d, 0, 0.1).until(fun.is_on_target),
        )
def deploy(d: drive.SwerveDrive, ele: elevate.Elevator, wrist: depo.DepositorWrist, wheels: depo.DepositorWheels) -> commands2.Command:
    return commands2.SequentialCommandGroup(
        wheels.deposite(),
        commands2.ParallelDeadlineGroup(
            forward(d, 270, 0.2).withTimeout(0.8),
            commands2.WaitCommand(0.25).andThen(receive(ele, wrist)),
        )
    )
def deploy4(d: drive.SwerveDrive, ele: elevate.Elevator, wrist: depo.DepositorWrist, wheels: depo.DepositorWheels) -> commands2.Command:
    return commands2.SequentialCommandGroup(
        wheels.deposite(),
        commands2.ParallelDeadlineGroup(
            commands2.WaitCommand(0.4).andThen(wrist.goto(90, clamp=0.4)),
            wheels.eject(0.6)
        ),
        forward(d, 270, 0.1).withTimeout(0.8),
        receive(ele, wrist)
        )
goto_l1 = lambda vis, cont, dri, ele, wri, interrupt, gl: coral_wo_sweep(vis, cont, dri, ele, wri, const.l1_ext, const.l_wrist_angle, interrupt, aprilalign2.reef_leftoff if gl else aprilalign2.reef_rightoff)
goto_l2 = lambda vis, cont, dri, ele, wri, interrupt, gl: coral_wo_sweep(vis, cont, dri, ele, wri, const.l2_ext, const.l_wrist_angle, interrupt, aprilalign2.reef_leftoff if gl else aprilalign2.reef_rightoff)
goto_l3 = lambda vis, cont, dri, ele, wri, interrupt, gl: coral_wo_sweep(vis, cont, dri, ele, wri, const.l3_ext, const.l_wrist_angle, interrupt, aprilalign2.reef_leftoff if gl else aprilalign2.reef_rightoff)
goto_l4 = lambda vis, cont, dri, ele, wri, interrupt, gl: coral_wo_sweep(vis, cont, dri, ele, wri, const.l4_ext, const.l_wrist_angle-10, interrupt, aprilalign2.reef_leftoff if gl else aprilalign2.reef_rightoff)

def reef(v: april.VisionSystem, ps: Callable[[], drive.Polar], 
        d: drive.SwerveDrive, ele: elevate.Elevator, wrist: depo.DepositorWrist,
        eheight: float, wangle: float, xoffset: float, spd=0.05, latpid=0.2) -> commands2.Command:
    return commands2.ParallelCommandGroup(
            commands2.RepeatCommand(wrist.goto(wangle)),
            commands2.RepeatCommand(ele.goto(eheight)),
            aprilalign2.Align(v, d, 90, ps, xoffset, spd=spd, latpid=latpid)
        )

# ACTION SEQUENCE
def receive(ele: elevate.Elevator, wrist: depo.DepositorWrist) -> commands2.Command:
    # TODO: Place to update code to remove the safety check
    return commands2.ParallelCommandGroup(
        commands2.SequentialCommandGroup(
            wrist.goto(0),
        ),
        commands2.SequentialCommandGroup(
            ele.goto(0.0),
            ele.home()
        )
    )
def stash_coral(ele: elevate.Elevator, wrist: depo.DepositorWrist, wheels: depo.DepositorWheels) -> commands2.Command:
    return commands2.ParallelDeadlineGroup(
        wheels.pickup(),
        wrist.goto(90, clamp=0.4),
    ).andThen(receive(ele, wrist))


# CAGE CLIMB
def upcage(ele: elevate.Elevator, wrist: depo.DepositorWrist) -> commands2.Command:
    return commands2.ParallelCommandGroup(
        ele.goto(0.20),
        wrist.goto(180),
    )
def downcage(ele: elevate.Elevator, wrist: depo.DepositorWrist, latch: lock.ClimbLock) -> commands2.Command:
    return commands2.SequentialCommandGroup(
        ele.goto(0).withTimeout(1.25),
        wrist.goto(90),
        latch.async_lock(),
        ele.home(dft=-0.45),
    )

# DriveTrain
def forward(d: drive.SwerveDrive, angle: float, speed: float):
    return commands2.FunctionalCommand(
        lambda: None,
        lambda: d.polar_drive(drive.Polar(speed, angle), 0, True),
        lambda x: d.polar_drive(drive.Polar(0, angle), 0, True),
        lambda: False,
        d
    )
def april_unwrap(vision: april.VisionSystem, drivetrain: drive.SwerveDrive, fwddir: float, ps: Callable[[], drive.Polar], targetlat: float, latpid: float = 0.2, spd=0.05):
    if vision() is not None:
        return aprilalign2.Align(vision, drivetrain, 270, ps, 0, latpid=latpid, spd=spd)
    return drivetrain.controller_drive(ps, 0)