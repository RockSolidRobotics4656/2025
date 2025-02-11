import commands2
import depo
import elevate

def goto_l1(ele: elevate.Elevator, wrist: depo.DepositorWrist) -> commands2.Command:
    return commands2.ParallelCommandGroup(
        wrist.goto(210),
        ele.goto(0)
    )
def goto_l2(ele: elevate.Elevator, wrist: depo.DepositorWrist) -> commands2.Command:
    return commands2.ParallelCommandGroup(
        wrist.goto(210),
        ele.goto(0.075)
    )
def goto_l3(ele: elevate.Elevator, wrist: depo.DepositorWrist) -> commands2.Command:
    return commands2.ParallelCommandGroup(
        wrist.goto(210),
        ele.goto(0.475)
    )
def goto_l4(ele: elevate.Elevator, wrist: depo.DepositorWrist) -> commands2.Command:
    return commands2.ParallelCommandGroup(
        wrist.goto(90),
        ele.goto(0.8)
    )
def upcage(ele: elevate.Elevator, wrist: depo.DepositorWrist) -> commands2.Command:
    return commands2.ParallelCommandGroup(
        ele.goto(0.35),
        wrist.goto(180),
    )
def downcage(ele: elevate.Elevator) -> commands2.Command:
    return commands2.SequentialCommandGroup(
        ele.home().withTimeout(1.5),
        ele.home(dft=-0.4)
    )

def deploy(ele: elevate.Elevator, wrist: depo.DepositorWrist, wheels: depo.DepositorWheels) -> commands2.Command:
    return commands2.SequentialCommandGroup(
        wheels.deposite(),
        receive(ele, wrist),
    )
def receive(ele: elevate.Elevator, wrist: depo.DepositorWrist) -> commands2.Command:
    return commands2.ParallelCommandGroup(
        commands2.SequentialCommandGroup(
            wrist.goto(0),
            wrist.home(),
            wrist.goto(5),
        ),
        ele.goto(0.0)
    )
