#
# Drivetrain — NavX + 4x Swerve Module
# RobotPy 2025 / WPILib native
#

import math
import navx
import wpimath.geometry
import wpimath.kinematics
import swervemodule

kMaxSpeed = 4.5  # metre/saniye
kMaxAngularSpeed = 11.5  # rad/saniye


class Drivetrain:
    """Swerve drive alt sistemi — 4 adet NEO swerve modülü + NavX gyro."""

    def __init__(self) -> None:
        # Modül konumları (inç → metre): 9.84 inç × 0.0254 = ~0.24994 m
        dist = 9.84 * 0.0254

        self.frontLeftLocation = wpimath.geometry.Translation2d(dist, dist)
        self.frontRightLocation = wpimath.geometry.Translation2d(dist, -dist)
        self.backLeftLocation = wpimath.geometry.Translation2d(-dist, -dist)
        self.backRightLocation = wpimath.geometry.Translation2d(-dist, dist)

        # --- Swerve Modülleri (CAN ID'leri + CANCoder offset'leri) ---
        self.frontLeft = swervemodule.SwerveModule(
            driveMotorID=21, turningMotorID=22, cancoderID=15,
            encoderOffset=0.78,
        )
        self.frontRight = swervemodule.SwerveModule(
            driveMotorID=27, turningMotorID=28, cancoderID=17,
            encoderOffset=0.76,
        )
        self.backLeft = swervemodule.SwerveModule(
            driveMotorID=23, turningMotorID=24, cancoderID=16,
            encoderOffset=0.49,
        )
        self.backRight = swervemodule.SwerveModule(
            driveMotorID=25, turningMotorID=26, cancoderID=18,
            encoderOffset=344.0,
        )

        # --- NavX Gyroscope (SPI MXP) ---
        self.gyro = navx.AHRS(navx.AHRS.NavXComType.kMXP_SPI)
        self.gyroInverted = True

        # --- Kinematics & Odometry ---
        self.kinematics = wpimath.kinematics.SwerveDrive4Kinematics(
            self.frontLeftLocation,
            self.frontRightLocation,
            self.backLeftLocation,
            self.backRightLocation,
        )

        self.odometry = wpimath.kinematics.SwerveDrive4Odometry(
            self.kinematics,
            self._getGyroRotation(),
            (
                self.frontLeft.getPosition(),
                self.frontRight.getPosition(),
                self.backLeft.getPosition(),
                self.backRight.getPosition(),
            ),
        )

    def _getGyroRotation(self) -> wpimath.geometry.Rotation2d:
        """NavX'ten yaw açısını döndürür (inverted destekli)."""
        yaw = self.gyro.getYaw()
        if self.gyroInverted:
            yaw = -yaw
        return wpimath.geometry.Rotation2d.fromDegrees(yaw)

    def resetGyro(self) -> None:
        """Gyroscope'u sıfırlar."""
        self.gyro.reset()

    def drive(
        self,
        xSpeed: float,
        ySpeed: float,
        rot: float,
        fieldRelative: bool,
        periodSeconds: float,
    ) -> None:
        """
        Joystick girdileriyle robotu sürer.
        :param xSpeed: İleri/geri hız (m/s).
        :param ySpeed: Sağa/sola hız (m/s).
        :param rot: Dönüş hızı (rad/s).
        :param fieldRelative: Saha referanslı mı?
        :param periodSeconds: Döngü periyodu.
        """
        swerveModuleStates = self.kinematics.toSwerveModuleStates(
            wpimath.kinematics.ChassisSpeeds.discretize(
                (
                    wpimath.kinematics.ChassisSpeeds.fromFieldRelativeSpeeds(
                        xSpeed, ySpeed, rot, self._getGyroRotation()
                    )
                    if fieldRelative
                    else wpimath.kinematics.ChassisSpeeds(xSpeed, ySpeed, rot)
                ),
                periodSeconds,
            )
        )
        wpimath.kinematics.SwerveDrive4Kinematics.desaturateWheelSpeeds(
            swerveModuleStates, kMaxSpeed
        )
        self.frontLeft.setDesiredState(swerveModuleStates[0])
        self.frontRight.setDesiredState(swerveModuleStates[1])
        self.backLeft.setDesiredState(swerveModuleStates[2])
        self.backRight.setDesiredState(swerveModuleStates[3])

    def stop(self) -> None:
        """Tüm modülleri durdurur."""
        zeroState = wpimath.kinematics.SwerveModuleState(
            0, wpimath.geometry.Rotation2d(0)
        )
        self.frontLeft.setDesiredState(zeroState)
        self.frontRight.setDesiredState(zeroState)
        self.backLeft.setDesiredState(zeroState)
        self.backRight.setDesiredState(zeroState)

    def updateOdometry(self) -> None:
        """Sahadaki konumu günceller."""
        self.odometry.update(
            self._getGyroRotation(),
            (
                self.frontLeft.getPosition(),
                self.frontRight.getPosition(),
                self.backLeft.getPosition(),
                self.backRight.getPosition(),
            ),
        )
