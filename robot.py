#!/usr/bin/env python3
#
# FRC 8828 — Swerve Robot (RobotPy 2025)
#

import math
import wpilib
import wpimath
import wpimath.filter
import drivetrain


class MyRobot(wpilib.TimedRobot):
    def robotInit(self) -> None:
        self.controller = wpilib.XboxController(0)
        self.swerve = drivetrain.Drivetrain()

        # Slew rate limiter — ani hız değişimlerini yumuşatır (1/3 s'de 0→1)
        self.xspeedLimiter = wpimath.filter.SlewRateLimiter(3)
        self.yspeedLimiter = wpimath.filter.SlewRateLimiter(3)
        self.rotLimiter = wpimath.filter.SlewRateLimiter(3)

        # Otonom zamanlayıcı
        self.autoTimer = wpilib.Timer()
        self.autoPhase = 0

    # ==========================================================================
    # OTONOM — Basit zamanlayıcı bazlı sekans
    # ==========================================================================
    #   Aşama 0: 1 s ileri sürüş (0.5 m/s)
    #   Aşama 1: 2 s bekleme (fren)
    #   Aşama 2: 1 s 360° dönüş (2π rad/s)
    #   Aşama 3: Dur

    def autonomousInit(self) -> None:
        self.autoPhase = 0
        self.autoTimer.restart()

    def autonomousPeriodic(self) -> None:
        elapsed = self.autoTimer.get()

        if self.autoPhase == 0:
            # 1 s boyunca ileri sür
            if elapsed < 1.0:
                self.swerve.drive(0.5, 0, 0, False, self.getPeriod())
            else:
                self.swerve.stop()
                self.autoTimer.restart()
                self.autoPhase = 1

        elif self.autoPhase == 1:
            # 2 s boyunca dur (fren)
            if elapsed < 2.0:
                self.swerve.stop()
            else:
                self.autoTimer.restart()
                self.autoPhase = 2

        elif self.autoPhase == 2:
            # 1 s boyunca yerinde 360° dön (2π rad/s)
            if elapsed < 1.0:
                self.swerve.drive(0, 0, 2 * math.pi, False, self.getPeriod())
            else:
                self.swerve.stop()
                self.autoPhase = 3

        else:
            # Otonom bitti — dur
            self.swerve.stop()

        self.swerve.updateOdometry()

    # ==========================================================================
    # TELEOP
    # ==========================================================================

    def teleopPeriodic(self) -> None:
        self._driveWithJoystick(fieldRelative=True)

    def _driveWithJoystick(self, fieldRelative: bool) -> None:
        # İleri/geri (sol joystick Y — ters çevrilmiş)
        xSpeed = (
            -self.xspeedLimiter.calculate(
                wpimath.applyDeadband(self.controller.getLeftY(), 0.1)
            )
            * drivetrain.kMaxSpeed
        )

        # Sağa/sola (sol joystick X — ters çevrilmiş)
        ySpeed = (
            -self.yspeedLimiter.calculate(
                wpimath.applyDeadband(self.controller.getLeftX(), 0.1)
            )
            * drivetrain.kMaxSpeed
        )

        # Dönüş (sağ joystick X — ters çevrilmiş)
        rot = (
            -self.rotLimiter.calculate(
                wpimath.applyDeadband(self.controller.getRightX(), 0.1)
            )
            * drivetrain.kMaxAngularSpeed
        )

        self.swerve.drive(xSpeed, ySpeed, rot, fieldRelative, self.getPeriod())


