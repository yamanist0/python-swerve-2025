#
# Swerve Module — NEO (SparkMax) + CANCoder
# RobotPy 2025 / WPILib native
#

import math

import rev
import phoenix6
import wpimath.controller
import wpimath.geometry
import wpimath.kinematics
import wpimath.trajectory

# --- Fiziksel sabitler ---
kWheelDiameterInches = 4.0
kWheelCircumferenceMeters = kWheelDiameterInches * math.pi * 0.0254  # ~0.3192 m
kDriveGearRatio = 6.75  # motor dönüşü / tekerlek dönüşü
kAngleGearRatio = 12.8  # motor dönüşü / modül dönüşü

# Drive encoder → metre ve m/s dönüşüm katsayısı
#   1 motor dönüşü = (çevre / gear_ratio) metre
kDrivePositionFactor = kWheelCircumferenceMeters / kDriveGearRatio  # metre/dönüş
kDriveVelocityFactor = kDrivePositionFactor / 60.0  # metre/saniye  (RPM → m/s)

# Angle encoder → radyan dönüşüm katsayısı
kAnglePositionFactor = math.tau / kAngleGearRatio  # radyan/motor-dönüşü

kModuleMaxAngularVelocity = 11.5  # rad/s
kModuleMaxAngularAcceleration = 2 * math.tau  # rad/s²


class SwerveModule:
    """Tek bir swerve modülü: NEO drive + NEO azimuth + CANCoder absolute encoder."""

    def __init__(
        self,
        driveMotorID: int,
        turningMotorID: int,
        cancoderID: int,
        encoderOffset: float,  # derece cinsinden
        driveInverted: bool = False,
        turningInverted: bool = False,
    ) -> None:
        # ===================== DRIVE MOTOR (SparkMax + NEO) =====================
        self.driveMotor = rev.SparkMax(driveMotorID, rev.SparkMax.MotorType.kBrushless)

        driveConfig = rev.SparkMaxConfig()
        driveConfig.setIdleMode(rev.SparkMaxConfig.IdleMode.kBrake)
        driveConfig.smartCurrentLimit(40)
        driveConfig.openLoopRampRate(0.25)
        driveConfig.inverted(driveInverted)
        driveConfig.encoder.positionConversionFactor(kDrivePositionFactor)
        driveConfig.encoder.velocityConversionFactor(kDriveVelocityFactor)
        self.driveMotor.configure(
            driveConfig,
            rev.SparkMax.ResetMode.kResetSafeParameters,
            rev.SparkMax.PersistMode.kPersistParameters,
        )

        self.driveEncoder = self.driveMotor.getEncoder()

        # ===================== TURNING MOTOR (SparkMax + NEO) ===================
        self.turningMotor = rev.SparkMax(turningMotorID, rev.SparkMax.MotorType.kBrushless)

        turnConfig = rev.SparkMaxConfig()
        turnConfig.setIdleMode(rev.SparkMaxConfig.IdleMode.kBrake)
        turnConfig.smartCurrentLimit(20)
        turnConfig.openLoopRampRate(0.25)
        turnConfig.inverted(turningInverted)
        self.turningMotor.configure(
            turnConfig,
            rev.SparkMax.ResetMode.kResetSafeParameters,
            rev.SparkMax.PersistMode.kPersistParameters,
        )

        # ===================== CANCODER (Absolute Encoder) ======================
        self.cancoder = phoenix6.hardware.CANcoder(cancoderID)
        # Offset derece cinsinden kaydedildi; CANCoder radyan cinsinden okuyacağız
        self.encoderOffsetRad = math.radians(encoderOffset)

        # ===================== PID KONTROLCÜLERİ ===============================
        self.drivePIDController = wpimath.controller.PIDController(
            0.0020645, 0, 0
        )

        self.turningPIDController = wpimath.controller.ProfiledPIDController(
            0.0020645,
            0,
            0,
            wpimath.trajectory.TrapezoidProfile.Constraints(
                kModuleMaxAngularVelocity,
                kModuleMaxAngularAcceleration,
            ),
        )
        self.turningPIDController.enableContinuousInput(-math.pi, math.pi)

        self.driveFeedforward = wpimath.controller.SimpleMotorFeedforwardMeters(0, 3)
        self.turnFeedforward = wpimath.controller.SimpleMotorFeedforwardMeters(0, 0.5)

    # ---- Yardımcı: CANCoder'dan modülün mutlak açısını oku ----
    def _getTurningAngle(self) -> wpimath.geometry.Rotation2d:
        """CANCoder'dan mutlak açıyı okur (offset uygulanmış, radyan)."""
        # CANCoder rotations (0-1 arası) → radyan
        absPos = self.cancoder.get_absolute_position().value  # rotations
        angleRad = absPos * math.tau - self.encoderOffsetRad
        return wpimath.geometry.Rotation2d(angleRad)

    # ---- Yardımcı: Sürüş hızı (m/s) ----
    def _getDriveVelocity(self) -> float:
        return self.driveEncoder.getVelocity()

    # ---- Yardımcı: Sürüş pozisyonu (m) ----
    def _getDrivePosition(self) -> float:
        return self.driveEncoder.getPosition()

    def getState(self) -> wpimath.kinematics.SwerveModuleState:
        return wpimath.kinematics.SwerveModuleState(
            self._getDriveVelocity(),
            self._getTurningAngle(),
        )

    def getPosition(self) -> wpimath.kinematics.SwerveModulePosition:
        return wpimath.kinematics.SwerveModulePosition(
            self._getDrivePosition(),
            self._getTurningAngle(),
        )

    def setDesiredState(
        self, desiredState: wpimath.kinematics.SwerveModuleState
    ) -> None:
        encoderRotation = self._getTurningAngle()

        # En kısa yol optimizasyonu (>90° dönme yerine motoru ters çevir)
        desiredState.optimize(encoderRotation)
        # Açı hatasının kosinüsüyle hızı ölçekle (daha yumuşak sürüş)
        desiredState.cosineScale(encoderRotation)

        # Drive çıkışı
        driveOutput = self.drivePIDController.calculate(
            self._getDriveVelocity(), desiredState.speed
        )
        driveFF = self.driveFeedforward.calculate(desiredState.speed)
        self.driveMotor.setVoltage(driveOutput + driveFF)

        # Turning çıkışı
        turnOutput = self.turningPIDController.calculate(
            encoderRotation.radians(), desiredState.angle.radians()
        )
        turnFF = self.turnFeedforward.calculate(
            self.turningPIDController.getSetpoint().velocity
        )
        self.turningMotor.setVoltage(turnOutput + turnFF)
