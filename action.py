from typing import *
import commands2
import depo
import elevate
import lock
import drive
import const
import aprilalign2
import april
import funnel

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
# TODO: Can remove funnel form it
def goto_l(v: april.VisionSystem, ps: Callable[[], drive.Polar], 
        i: Callable[[], bool], wh: depo.DepositorWheels,
        d: drive.SwerveDrive, ele: elevate.Elevator, wrist: depo.DepositorWrist,
        eheight: float, wangle: float) -> commands2.Command:
    start = commands2.ParallelDeadlineGroup(
            commands2.WaitUntilCommand(i),
            commands2.RepeatCommand(wrist.goto(wangle)),
            commands2.RepeatCommand(ele.goto(eheight)),
            aprilalign2.Align(v, d, 90, ps)
        )
    return commands2.SequentialCommandGroup(start, deploy(d, ele, wrist, wh))
goto_l1 = lambda v, o, i, wh, d, e, w: goto_l(v, o, i, wh, d, e, w, const.l1_ext, 230)
goto_l2 = lambda v, o, i, wh, d, e, w: goto_l(v, o, i, wh, d, e, w, const.l2_ext, 230)
goto_l3 = lambda v, o, i, wh, d, e, w: goto_l(v, o, i, wh, d, e, w, const.l3_ext, 230)
goto_l4 = lambda v, o, i, wh, d, e, w: goto_l(v, o, i, wh, d, e, w, const.l4_ext, 230)

# ACTION SEQUENCE
def receive(ele: elevate.Elevator, wrist: depo.DepositorWrist) -> commands2.Command:
    return commands2.ParallelCommandGroup(
        commands2.SequentialCommandGroup(
            wrist.goto(0),
            wrist.home(),
        ),
        commands2.SequentialCommandGroup(
            ele.goto(0.0),
            ele.home()
        )
    )
def stash_coral(ele: elevate.Elevator, wrist: depo.DepositorWrist, wheels: depo.DepositorWheels) -> commands2.Command:
    return commands2.ParallelDeadlineGroup(
        wheels.pickup(),
        wrist.goto(90),
    ).andThen(receive(ele, wrist))

def deploy(d: drive.SwerveDrive, ele: elevate.Elevator, wrist: depo.DepositorWrist, wheels: depo.DepositorWheels) -> commands2.Command:
    return commands2.SequentialCommandGroup(
        wheels.deposite(),
        commands2.ParallelDeadlineGroup(
            forward(d, 270, 0.2).withTimeout(0.8),
            receive(ele, wrist),
        )
    )

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