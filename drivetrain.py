import math
import wpilib
import navx
import wpimath.controller
import wpimath.geometry
import wpimath.kinematics
import wpimath.trajectory
import swervemodule

kMaxSpeed = 4.5  # max speed in meters per second
kMaxAngularSpeed = math.pi  # max rotation speed in radians per second


# drivetrain subsystem
class Drivetrain:
    def __init__(self) -> None:
        # module distance from center
        dist = 11.1 * 0.0254

        self.frontLeftLocation = wpimath.geometry.Translation2d(dist, dist)
        self.frontRightLocation = wpimath.geometry.Translation2d(dist, -dist)
        self.backLeftLocation = wpimath.geometry.Translation2d(-dist, dist)
        self.backRightLocation = wpimath.geometry.Translation2d(-dist, -dist)

        # swerve modules setup
        self.frontLeft = swervemodule.SwerveModule(
            driveMotorID=21, turningMotorID=22, cancoderID=15,
            encoderOffset=105.029,
            driveInverted=True, turningInverted=True,
        )
        self.frontRight = swervemodule.SwerveModule(
            driveMotorID=27, turningMotorID=28, cancoderID=17,
            encoderOffset=94.833,
            driveInverted=True, turningInverted=True,
        )
        self.backLeft = swervemodule.SwerveModule(
            driveMotorID=23, turningMotorID=24, cancoderID=16,
            encoderOffset=183.4277,
            driveInverted=True, turningInverted=True,
        )
        self.backRight = swervemodule.SwerveModule(
            driveMotorID=25, turningMotorID=26, cancoderID=18,
            encoderOffset=325.28,
            driveInverted=True, turningInverted=True,
        )

        # navx gyro setup
        self.gyro = navx.AHRS(navx.AHRS.NavXComType.kMXP_SPI)
        self.gyroInverted = False

        # kinematics and odometry setup
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

        # closed-loop heading controller for autonomous turns
        self.turnController = wpimath.controller.ProfiledPIDController(
            4.0,
            0.0,
            0.2,
            wpimath.trajectory.TrapezoidProfile.Constraints(
                2 * math.pi, 4 * math.pi
            ),
        )
        self.turnController.setTolerance(math.radians(2))
        self.turnTarget = None

    def _getGyroRotation(self) -> wpimath.geometry.Rotation2d:
        # get gyro rotation angle in WPILib convention (CCW positive)
        rotation = self.gyro.getRotation2d()
        if self.gyroInverted:
            rotation = -rotation
        return rotation

    def resetGyro(self) -> None:
        # reset gyro angle
        self.gyro.reset()

    def _getContinuousAngle(self) -> float:
        # continuous CCW-positive heading in radians
        # (navx getAngle is clockwise-positive and does not wrap at +/-180)
        return -math.radians(self.gyro.getAngle())

    def turnToAngle(self, deltaDegrees: float, periodSeconds: float) -> bool:
        # closed-loop relative turn; returns True when the target heading is reached
        current = self._getContinuousAngle()
        if self.turnTarget is None:
            self.turnTarget = current + math.radians(deltaDegrees)
            self.turnController.reset(current)
        rot = self.turnController.calculate(current, self.turnTarget)
        rot = max(-kMaxAngularSpeed, min(kMaxAngularSpeed, rot))
        self.drive(0, 0, rot, False, periodSeconds)
        if self.turnController.atGoal():
            self.turnTarget = None
            self.stop()
            return True
        return False

    def cancelTurn(self) -> None:
        # abort an in-progress closed-loop turn
        self.turnTarget = None

    def drive(
        self,
        xSpeed: float,
        ySpeed: float,
        rot: float,
        fieldRelative: bool,
        periodSeconds: float,
    ) -> None:
        # drive robot with joystick inputs
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
        swerveModuleStates = wpimath.kinematics.SwerveDrive4Kinematics.desaturateWheelSpeeds(
            swerveModuleStates, kMaxSpeed
        )
        self.frontLeft.setDesiredState(swerveModuleStates[0])
        self.frontRight.setDesiredState(swerveModuleStates[1])
        self.backLeft.setDesiredState(swerveModuleStates[2])
        self.backRight.setDesiredState(swerveModuleStates[3])

    def stop(self) -> None:
        # stop all modules in place
        self.turnTarget = None
        self.frontLeft.stop()
        self.frontRight.stop()
        self.backLeft.stop()
        self.backRight.stop()

    def updateOdometry(self) -> None:
        # update robot field position
        self.odometry.update(
            self._getGyroRotation(),
            (
                self.frontLeft.getPosition(),
                self.frontRight.getPosition(),
                self.backLeft.getPosition(),
                self.backRight.getPosition(),
            ),
        )

        # debug: calibration telemetry, remove after tuning
        wpilib.SmartDashboard.putNumber("gyro/yaw", self._getGyroRotation().degrees())
        pose = self.odometry.getPose()
        wpilib.SmartDashboard.putNumber("odometry/x", pose.X())
        wpilib.SmartDashboard.putNumber("odometry/y", pose.Y())
        for name, module in (
            ("fl", self.frontLeft),
            ("fr", self.frontRight),
            ("bl", self.backLeft),
            ("br", self.backRight),
        ):
            wpilib.SmartDashboard.putNumber(
                f"module/{name}/angle", module._getTurningAngle().degrees()
            )
            wpilib.SmartDashboard.putNumber(
                f"module/{name}/speed", module._getDriveVelocity()
            )

