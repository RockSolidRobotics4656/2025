import commands2
import depo
import elevate

# Algae
def low_algae(ele: elevate.Elevator, wrist: depo.DepositorWrist) -> commands2.Command:
    pass
def high_algae(ele: elevate.Elevator, wrist: depo.DepositorWrist) -> commands2.Command:
    pass

# POSITIONS
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
def receive(ele: elevate.Elevator, wrist: depo.DepositorWrist) -> commands2.Command:
    return commands2.ParallelCommandGroup(
        commands2.SequentialCommandGroup(
            wrist.goto(0),
            wrist.home(),
            wrist.goto(5),
        ),
        ele.goto(0.0)
    )

# ACTION SEQUENCE
def deploy(ele: elevate.Elevator, wrist: depo.DepositorWrist, wheels: depo.DepositorWheels) -> commands2.Command:
    return commands2.SequentialCommandGroup(
        wheels.deposite(),
        commands2.PrintCommand("I need to implement backing up first or we will hit the pipe!")
        #receive(ele, wrist),
    )

# CAGE CLIMB
def upcage(ele: elevate.Elevator, wrist: depo.DepositorWrist) -> commands2.Command:
    return commands2.ParallelCommandGroup(
        ele.goto(0.20),
        wrist.goto(180),
    )
def downcage(ele: elevate.Elevator, wrist: depo.DepositorWrist) -> commands2.Command:
    return commands2.SequentialCommandGroup(
        ele.goto(0).withTimeout(1.25),
        wrist.goto(90),
        ele.home(dft=-0.45),
    )