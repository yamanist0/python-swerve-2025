import math

import rev
import phoenix6
import wpimath.controller
import wpimath.geometry
import wpimath.kinematics
import wpimath.trajectory

# physical constants
kWheelDiameterInches = 4.0
kWheelCircumferenceMeters = kWheelDiameterInches * math.pi * 0.0254  # wheel circumference
kDriveGearRatio = 6.75  # drive gear ratio

# conversion factors
kDrivePositionFactor = kWheelCircumferenceMeters / kDriveGearRatio  # meters per revolution
kDriveVelocityFactor = kDrivePositionFactor / 60.0  # meters per second

kModuleMaxAngularVelocity = 11.5  # max turn speed
kModuleMaxAngularAcceleration = 2 * math.tau  # max turn acceleration


# swerve module implementation
class SwerveModule:
    def __init__(
        self,
        driveMotorID: int,
        turningMotorID: int,
        cancoderID: int,
        encoderOffset: float,
        driveInverted: bool = False,
        turningInverted: bool = False,
    ) -> None:
        # drive motor configuration
        self.driveMotor = rev.SparkMax(driveMotorID, rev.SparkMax.MotorType.kBrushless)

        driveConfig = rev.SparkMaxConfig()
        driveConfig.setIdleMode(rev.SparkMaxConfig.IdleMode.kBrake)
        driveConfig.smartCurrentLimit(40)
        driveConfig.openLoopRampRate(0.25)
        driveConfig.inverted(driveInverted)
        driveConfig.voltageCompensation(12)
        driveConfig.encoder.positionConversionFactor(kDrivePositionFactor)
        driveConfig.encoder.velocityConversionFactor(kDriveVelocityFactor)
        self.driveMotor.configure(
            driveConfig,
            rev.ResetMode.kResetSafeParameters,
            rev.PersistMode.kPersistParameters,
        )

        self.driveEncoder = self.driveMotor.getEncoder()

        # turning motor configuration
        self.turningMotor = rev.SparkMax(turningMotorID, rev.SparkMax.MotorType.kBrushless)

        turnConfig = rev.SparkMaxConfig()
        turnConfig.setIdleMode(rev.SparkMaxConfig.IdleMode.kBrake)
        turnConfig.smartCurrentLimit(20)
        turnConfig.openLoopRampRate(0.25)
        turnConfig.inverted(turningInverted)
        turnConfig.voltageCompensation(12)
        self.turningMotor.configure(
            turnConfig,
            rev.ResetMode.kResetSafeParameters,
            rev.PersistMode.kPersistParameters,
        )

        # cancoder configuration
        self.cancoder = phoenix6.hardware.CANcoder(cancoderID)
        self.encoderOffsetRad = math.radians(encoderOffset)

        # pid controllers
        self.drivePIDController = wpimath.controller.PIDController(1.0, 0, 0)

        self.turningPIDController = wpimath.controller.ProfiledPIDController(
            7.5,
            0,
            0.1,
            wpimath.trajectory.TrapezoidProfile.Constraints(
                kModuleMaxAngularVelocity,
                kModuleMaxAngularAcceleration,
            ),
        )
        self.turningPIDController.enableContinuousInput(-math.pi, math.pi)

        # feedforward controllers
        self.driveFeedforward = wpimath.controller.SimpleMotorFeedforwardMeters(0.1, 2.65)

    def _getTurningAngle(self) -> wpimath.geometry.Rotation2d:
        # get absolute turning angle
        absPos = self.cancoder.get_absolute_position().value
        angleRad = absPos * math.tau - self.encoderOffsetRad
        return wpimath.geometry.Rotation2d(angleRad)

    def _getDriveVelocity(self) -> float:
        # get drive velocity
        return self.driveEncoder.getVelocity()

    def _getDrivePosition(self) -> float:
        # get drive position
        return self.driveEncoder.getPosition()

    def getState(self) -> wpimath.kinematics.SwerveModuleState:
        # get module state
        return wpimath.kinematics.SwerveModuleState(
            self._getDriveVelocity(),
            self._getTurningAngle(),
        )

    def getPosition(self) -> wpimath.kinematics.SwerveModulePosition:
        # get module position
        return wpimath.kinematics.SwerveModulePosition(
            self._getDrivePosition(),
            self._getTurningAngle(),
        )

    def setDesiredState(
        self, desiredState: wpimath.kinematics.SwerveModuleState
    ) -> None:
        # set module state
        encoderRotation = self._getTurningAngle()

        desiredState.optimize(encoderRotation)
        desiredState.cosineScale(encoderRotation)

        driveOutput = self.drivePIDController.calculate(
            self._getDriveVelocity(), desiredState.speed
        )
        driveFF = self.driveFeedforward.calculate(desiredState.speed)
        self.driveMotor.setVoltage(driveOutput + driveFF)

        turnOutput = self.turningPIDController.calculate(
            encoderRotation.radians(), desiredState.angle.radians()
        )
        self.turningMotor.setVoltage(turnOutput)

    def stop(self) -> None:
        # stop both motors in place without re-aiming the wheels
        self.driveMotor.stopMotor()
        self.turningMotor.stopMotor()

