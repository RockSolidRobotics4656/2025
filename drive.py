from typing import *
import math
import commands2
import wpimath.controller
import wpimath.geometry
import wpimath.kinematics
import ncoms
import wpimath
import rev
import phoenix6
import encoder
import control
import dataclasses

@dataclasses.dataclass
class Polar:
    magnitude: float
    angle: float

def mix_polar(a: Polar, b: Polar) -> Polar:
    dx = math.cos(math.radians(a.angle))*a.magnitude+math.cos(math.radians(b.angle))*b.magnitude
    dy = math.sin(math.radians(a.angle))*a.magnitude+math.sin(math.radians(b.angle))*b.magnitude
    dist = math.sqrt(dx**2 + dy**2)
    angle = math.degrees(math.atan2(dy, dx))
    return Polar(dist, angle)

class SwerveDrive(commands2.Subsystem):
    def __init__(self) -> None:
        super().__init__()
        self.cells = (
            SwerveCell(1,  2,  3, turn_offset=-250+0),
            SwerveCell(4,  5,  6, turn_offset=-127+0),
            SwerveCell(7,  8,  9, turn_offset=-185+0),
            SwerveCell(10, 11, 12, turn_offset=0+0),
        )
        dx = 18
        hdx = dx / 2 / 1 # 39 inches per meter - replace 1 with 39
        self.translations = (
            wpimath.geometry.Translation2d(hdx, hdx),
            wpimath.geometry.Translation2d(hdx, -hdx),
            wpimath.geometry.Translation2d(-hdx, -hdx),
            wpimath.geometry.Translation2d(-hdx, hdx)
            )
        self.kinematics = wpimath.kinematics.SwerveDrive4Kinematics(
            self.translations[0], self.translations[1],
            self.translations[2], self.translations[3]
        )
        hardware_gyro = phoenix6.hardware.Pigeon2(17)
        supplier = hardware_gyro.get_yaw().as_supplier()
        self.gyro = encoder.EncoderAdapter(supplier)
        self.gyro.set_rotational(True)
        self.gyro.set_ticks_per_unit(1.0)
        self.gyro.reset()
    

    def periodic(self):
        self.cells[0].telemetry(ncoms.drtelem_tab, "Cell 1"),
        self.cells[1].telemetry(ncoms.drtelem_tab, "Cell 2"),
        self.cells[2].telemetry(ncoms.drtelem_tab, "Cell 3"),
        self.cells[3].telemetry(ncoms.drtelem_tab, "Cell 4"),
    
    def create_odometry(self):
        angle = wpimath.geometry.Rotation2d.fromDegrees(self.gyro())
        return wpimath.kinematics.SwerveDrive4Odometry(self.kinematics, angle,
            (
                self.cells[0].get(),
                self.cells[1].get(),
                self.cells[2].get(),
                self.cells[3].get(),
            ))

    def update_odo(self, odo: wpimath.kinematics.SwerveDrive4Odometry):
        angle = wpimath.geometry.Rotation2d.fromDegrees(self.gyro())
        odo.update(angle, (
                self.cells[0].get(),
                self.cells[1].get(),
                self.cells[2].get(),
                self.cells[3].get(),
            ))
    
    def polar_drive(self, trans: Polar, r: float, relative=False):
        desired_state = wpimath.kinematics.ChassisSpeeds.fromFieldRelativeSpeeds(
            math.cos(math.radians(trans.angle))*trans.magnitude,
            math.sin(math.radians(trans.angle))*trans.magnitude,
            r * 1/40,
            wpimath.geometry.Rotation2d.fromDegrees(self.gyro())
        )
        if relative:
            desired_state = wpimath.kinematics.ChassisSpeeds(
                math.cos(math.radians(trans.angle))*trans.magnitude,
                math.sin(math.radians(trans.angle))*trans.magnitude,
                r * 1/40
                )
        states = self.kinematics.toSwerveModuleStates(desired_state)
        for cell, state in zip(self.cells, states):
            cell.set(state)

    def controller_drive(self, dir: Callable[[], Polar], r: Callable[[], float]):
        def tick():
            self.polar_drive(dir(), r(), relative=False)
        return commands2.RunCommand(tick, self)

class SwerveCell:
    def __init__(self, linearid: int, turnid: int, angleid: int, turn_offset: float = 0.0):
        super().__init__()
        # Linear Encoder
        self.linear_motor = rev.SparkMax(linearid, rev.SparkMax.MotorType.kBrushless)
        _linear_encoder = self.linear_motor.getEncoder()
        self.linear_encoder = encoder.EncoderAdapter(_linear_encoder.getPosition)
        self.linear_encoder.set_ticks_per_unit(25)
        self.linear_encoder.reset()

        # Turn
        self.turn_motor = rev.SparkMax(turnid, rev.SparkMax.MotorType.kBrushless)
        _turn_encoder = phoenix6.hardware.CANcoder(angleid)
        _abs_turn = _turn_encoder.get_absolute_position()
        _abs_turn.set_update_frequency(100)
        self.turn_encoder = encoder.EncoderAdapter(
            _abs_turn.as_supplier(),
            turn_offset
        )
        self.turn_encoder.set_rotational(True)
        self.turn_encoder.set_ticks_per_unit(1/360)

        self.swerve_func = control.SwerveFunc()
    
    def get(self) -> wpimath.kinematics.SwerveModulePosition:
        return wpimath.kinematics.SwerveModulePosition(
            self.linear_encoder(),
            wpimath.geometry.Rotation2d.fromDegrees(self.turn_encoder())
        )

    def set(self, new_state: wpimath.kinematics.SwerveModuleState):
        cstate = self.get()

        # BUG: WPILIB BUG FIX!!!!!
        # I found a bug right here in wpilib - I need to check for null
        # Optimize can return None - for some reason
        nstate = wpimath.kinematics.SwerveModuleState.optimize(new_state, cstate.angle)
        if nstate is None: nstate = new_state

        # Linear
        self.linear_motor.set(nstate.speed)

        # Turn
        current_angle = cstate.angle.degrees()
        target_angle = nstate.angle.degrees()
        self.turn_motor.set(self.swerve_func(current_angle, target_angle))

    def telemetry(self, telem, name: Optional[str] = None) -> None:
        name = name if name else ncoms.uname()
        state = self.get()
        telem.putNumber(f"{name} Linear", state.distance)
        telem.putNumber(f"{name} Turn", state.angle.degrees())
