import commands2
import depo
import position
import drive
import aprilalign2
import april
import funnel
import lock

void = lambda: drive.Polar(0,0)
def forward(d: drive.SwerveDrive, angle: float, speed: float):
    return commands2.FunctionalCommand(
        lambda: None,
        lambda: d.polar_drive(drive.Polar(speed, angle), 0, True),
        lambda x: d.polar_drive(drive.Polar(0, angle), 0, True),
        lambda: False,
        d
    )
def approach_coral(positional: position.Position, wheels: depo.DepositorWheels,
    drivetrain: drive.SwerveDrive) -> commands2.Command:
    return commands2.SequentialCommandGroup(
        commands2.ParallelCommandGroup(
            wheels.intake(1.0),
            forward(drivetrain, 270, 0.1),
            positional.acquire()
        )
    )
def retreat_coral(positional: position.Position, wheels: depo.DepositorWheels) -> commands2.Command:
    return commands2.ParallelCommandGroup(
        wheels.pickup(),
        positional.stow(wri_speed=0.6)
    )
def low_algae(positional: position.Position, wheels: depo.DepositorWheels) -> commands2.Command:
    return commands2.ParallelCommandGroup(
        positional.reef_position(position.P_AL),
        wheels.eject(1),
    )
def high_algae(positional: position.Position, wheels: depo.DepositorWheels) -> commands2.Command:
    return commands2.ParallelCommandGroup(
        positional.reef_position(position.P_AH),
        wheels.eject(1),
    )
def release_high_algae(d: drive.SwerveDrive, v: april.VisionSystem, p: position.Position, wheels: depo.DepositorWheels):
    return commands2.ParallelDeadlineGroup(
        commands2.SequentialCommandGroup(
            aprilalign2.Align(v, d, 90, void, aprilalign2.reef_middle_sweep, latpid=0.5, spd=0.2).until(d.is_front_grounded),
            commands2.WaitCommand(1.0),
            forward(d, 270, 0.1).withTimeout(1.0)
        ),
        high_algae(p, wheels)
    )
def release_low_algae(d: drive.SwerveDrive, v: april.VisionSystem, p: position.Position, wheels: depo.DepositorWheels):
    return commands2.ParallelDeadlineGroup(
        commands2.SequentialCommandGroup(
            aprilalign2.Align(v, d, 90, void, aprilalign2.reef_middle_sweep, latpid=0.5, spd=0.2).until(d.is_front_grounded),
            commands2.WaitCommand(1.0),
            forward(d, 270, 0.1).withTimeout(1.0)
        ),
        low_algae(p, wheels)
    )
def score_coral(drivetrain: drive.SwerveDrive, vision: april.VisionSystem,
    positional: position.Position, wheels: depo.DepositorWheels, 
    funn: funnel.Funnel, rp: int, left: bool) -> commands2.Command:
    sweep_spd = 0.05
    sweep = forward(drivetrain, 180, sweep_spd)
    if not left: sweep = forward(drivetrain, 0, sweep_spd)
    ready = lambda: drivetrain.is_front_grounded() and positional.ready()
    return commands2.SequentialCommandGroup(
        commands2.ParallelDeadlineGroup(
            commands2.SequentialCommandGroup(
                aprilalign2.Align(vision, drivetrain, 90, void, aprilalign2.reef_middle_sweep, latpid=0.5, spd=0.15).until(ready),
                forward(drivetrain, 90, 0.2).withTimeout(0.25),
                sweep.until(funn.is_on_target).withTimeout(3.0),
                drivetrain.stop(),
                wheels.deposite(),
            ),
            positional.reef_position(rp, ele_speed=0.8, wri_speed=1.0),
        ),
        commands2.ParallelDeadlineGroup(
            forward(drivetrain, 270, 0.35).withTimeout(0.6),
            commands2.WaitCommand(0.3).andThen(
                positional.stow(ele_speed=0.4, wri_speed=1.0),
            )
        ),
        commands2.ScheduleCommand(positional.stow())
    )
def score_coral4(drivetrain: drive.SwerveDrive, vision: april.VisionSystem,
    positional: position.Position, wheels: depo.DepositorWheels, 
    funn: funnel.Funnel, left: bool) -> commands2.Command:
    sweep_spd = 0.05
    sweep = forward(drivetrain, 180, sweep_spd)
    if not left: sweep = forward(drivetrain, 0, sweep_spd)
    ready = lambda: drivetrain.is_front_grounded() and positional.ready()
    return commands2.SequentialCommandGroup(
        commands2.ParallelDeadlineGroup(
            commands2.SequentialCommandGroup(
                aprilalign2.Align(vision, drivetrain, 90, void, aprilalign2.reef_middle_sweep, latpid=0.5, spd=0.15).until(ready),
                forward(drivetrain, 90, 0.2).withTimeout(0.25),
                sweep.until(funn.is_on_target).withTimeout(3.0),
                drivetrain.stop(),
            ),
            positional.scan(ele_speed=0.8, wri_speed=1.0)
        ),
        commands2.WaitCommand(0.2),
        positional.tall(ele_speed=0, wri_speed=1.0).withTimeout(0.4),
        positional.tall().withTimeout(0.6),
        positional.reef_position(position.P_L4).withTimeout(1.0),
        commands2.ParallelDeadlineGroup(
            wheels.deposite(),
            positional.reef_position(position.P_L4)
        ),
        commands2.ParallelDeadlineGroup(
            forward(drivetrain, 270, 0.1).withTimeout(0.5),
            positional.tall()
        ),
        commands2.ScheduleCommand(positional.stow())
    )


def upcage(positional: position.Position):
    return positional.outofway()
def downcage(positional: position.Position, latch: lock.ClimbLock):
    return commands2.SequentialCommandGroup(
        positional.elevator.home().withTimeout(1.25),
        positional.wrist.goto(90, 1.0),
        commands2.InstantCommand(lambda: positional.wrist.move(0)),
        latch.async_lock(),
        commands2.WaitCommand(0.4),
        positional.elevator.home(dft=-0.5).withTimeout(0.75),
    )