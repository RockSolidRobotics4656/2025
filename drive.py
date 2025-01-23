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
        hdx = dx / 2
        self.kinematics = wpimath.kinematics.SwerveDrive4Kinematics(
            wpimath.geometry.Translation2d(hdx, hdx),
            wpimath.geometry.Translation2d(hdx, -hdx),
            wpimath.geometry.Translation2d(-hdx, -hdx),
            wpimath.geometry.Translation2d(-hdx, hdx)
        )
        hardware_gyro = phoenix6.hardware.Pigeon2(17)
        supplier = hardware_gyro.get_yaw().as_supplier()
        self.gyro = encoder.EncoderAdapter(supplier)
        self.gyro.set_rotational(True)
        self.gyro.set_ticks_per_unit(1.0)
        self.gyro.reset()
    
    def telemetry(self, destination) -> commands2.Command:
        return commands2.ParallelCommandGroup(
            self.cells[0].telemetry(destination, "Cell 1"),
            self.cells[1].telemetry(destination, "Cell 2"),
            self.cells[2].telemetry(destination, "Cell 3"),
            self.cells[3].telemetry(destination, "Cell 4"),
            commands2.RunCommand(lambda: ncoms.drtelem_tab.putNumber("gyangle", self.gyro()))
        )
    
    def polar_drive(self, angle: Callable[[], float], mag: Callable[[], float]) -> commands2.Command:
        def update_cells():
            desired_state = wpimath.kinematics.ChassisSpeeds.fromFieldRelativeSpeeds(
                math.cos(math.radians(angle()))*mag(),
                math.sin(math.radians(angle()))*mag(),
                0, # zero rotation
                wpimath.geometry.Rotation2d.fromDegrees(self.gyro())
            )
            states = self.kinematics.toSwerveModuleStates(desired_state)
            for cell, state in zip(self.cells, states):
                cell.set(state)
        return commands2.RunCommand(update_cells, self)
        



class SwerveCell:
    def __init__(self, linearid: int, turnid: int, angleid: int, turn_offset: float = 0.0):
        super().__init__()
        # Linear Encoder
        self.linear_motor = rev.SparkMax(linearid, rev.SparkMax.MotorType.kBrushless)
        _linear_encoder = self.linear_motor.getEncoder()
        self.linear_encoder = encoder.EncoderAdapter(_linear_encoder.getPosition)
        self.linear_encoder.set_ticks_per_unit(1/10)
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

    def telemetry(self, telem = None, name: Optional[str] = None) -> commands2.Command:
        name = name if name else ncoms.uname()
        def logtotelem():
            state = self.get()
            if telem:
                telem.putNumber(f"{name} Linear", state.distance)
                telem.putNumber(f"{name} Turn", state.angle.degrees())
            return None
        return commands2.RunCommand(logtotelem)
