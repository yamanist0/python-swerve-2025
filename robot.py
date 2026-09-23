import math
import wpilib
import wpimath
import wpimath.filter
import drivetrain
import fuel_subsystem
import bicerdover_subsystem
import kaldirirdover_subsystem

# controller deadband: stick movement below this is ignored (drift filter)
kDeadband = 0.1


def _deadband(value: float) -> float:
    # ignore micro-scale stick movement, rescale the rest to 0..1
    if abs(value) < kDeadband:
        return 0.0
    return math.copysign((abs(value) - kDeadband) / (1.0 - kDeadband), value)


# main robot class
class MyRobot(wpilib.TimedRobot):
    def robotInit(self) -> None:
        self.controller = wpilib.XboxController(0)
        self.swerve = drivetrain.Drivetrain()

        # additional subsystems from raptor-2026
        self.fuel = fuel_subsystem.FuelSubsystem()
        self.bicerdover = bicerdover_subsystem.BicerdoverSubsystem()
        self.kaldirirdover = kaldirirdover_subsystem.KaldirirdoverSubsystem()

        # initialize limiters
        self.xspeedLimiter = wpimath.filter.SlewRateLimiter(3)
        self.yspeedLimiter = wpimath.filter.SlewRateLimiter(3)
        self.rotLimiter = wpimath.filter.SlewRateLimiter(3)

        # autonomous timer
        self.autoTimer = wpilib.Timer()
        self.autoPhase = 0

        # edge detection for RS button gyro reset
        self.prevRs = False

        # edge detection and state for A button indirirdover toggle
        self.prevA = False
        self.indirirdoverTargetUp = False

    def robotPeriodic(self) -> None:
        # update odometry and publish telemetry in all modes (incl. disabled)
        self.swerve.updateOdometry()

        # run indirirdover pid loop every cycle
        self.bicerdover.update_indirirdover_pid()

        # publish bicerdover telemetry
        self.bicerdover.update_telemetry()

        # debug: controller diagnostics, remove after tuning
        for i in range(6):
            wpilib.SmartDashboard.putNumber(
                f"joy/axis{i}", wpilib.DriverStation.getStickAxis(0, i)
            )

    def autonomousInit(self) -> None:
        self.autoPhase = 0
        self.autoTimer.restart()
        self.swerve.cancelTurn()

    def autonomousPeriodic(self) -> None:
        elapsed = self.autoTimer.get()

        if self.autoPhase == 0:
            # drive forward
            if elapsed < 1.0:
                self.swerve.drive(2.5, 0, 0, False, self.getPeriod())
            else:
                self.swerve.stop()
                self.autoTimer.restart()
                self.autoPhase = 1

        elif self.autoPhase == 1:
            # pause
            if elapsed < 2.0:
                self.swerve.stop()
            else:
                self.autoTimer.restart()
                self.autoPhase = 3

        # elif self.autoPhase == 2:
        #     # turn 360 degrees, closed-loop on the gyro
        #     done = self.swerve.turnToAngle(360, self.getPeriod())
        #     if done or elapsed > 5.0:
        #         self.swerve.stop()
        #         self.autoPhase = 3

        else:
            # stop
            self.swerve.stop()

    def teleopPeriodic(self) -> None:
        self._driveWithJoystick(fieldRelative=True)
        self._handleSubsystemButtons()

        # RS (Right Stick click): reset gyro heading (rising edge only)
        rsNow = self.controller.getRightStickButton()
        if rsNow and not self.prevRs:
            self.swerve.resetGyro()
        self.prevRs = rsNow

    def _handleSubsystemButtons(self) -> None:
        # lb (left bumper): intake while held
        if self.controller.getLeftBumperButton():
            self.fuel.intake()
        # rb (right bumper): launch while held
        elif self.controller.getRightBumperButton():
            self.fuel.launch()
        else:
            self.fuel.stop()

        # rt (right trigger): kaldirirdover up while held
        if self.controller.getRightTriggerAxis() > 0.5:
            self.kaldirirdover.yukari()
        # lt (left trigger): kaldirirdover down while held
        elif self.controller.getLeftTriggerAxis() > 0.5:
            self.kaldirirdover.asagi()
        else:
            self.kaldirirdover.stop()

        # a button: toggle indirirdover between up (8.0) and down (0.5)
        aNow = self.controller.getAButton()
        if aNow and not self.prevA:
            self.indirirdoverTargetUp = not self.indirirdoverTargetUp
            if self.indirirdoverTargetUp:
                self.bicerdover.set_indirirdover_target(
                    bicerdover_subsystem.INDIRIRDOVER_UP_POSITION
                )
            else:
                self.bicerdover.set_indirirdover_target(
                    bicerdover_subsystem.INDIRIRDOVER_DOWN_POSITION
                )
        self.prevA = aNow

        # bicerdover + eject run while indirirdover is targeting up
        if self.indirirdoverTargetUp:
            self.bicerdover.bicerdover_run()
            self.fuel.eject()
        else:
            self.bicerdover.bicerdover_stop()

        # b button: donmedolap slow forward while held
        if self.controller.getBButton():
            self.bicerdover.donmedolap_run()
        # y button: donmedolap slow reverse while held
        elif self.controller.getYButton():
            self.bicerdover.donmedolap_reverse()
        else:
            self.bicerdover.donmedolap_stop()

    def _driveWithJoystick(self, fieldRelative: bool) -> None:
        joyX = _deadband(self.controller.getLeftY())
        joyY = _deadband(self.controller.getLeftX())
        pov = self.controller.getPOV()

        # use d-pad for precise directional driving when left stick is idle
        if joyX == 0 and joyY == 0 and pov != -1:
            pov_rad = math.radians(pov)
            raw_x = round(math.cos(pov_rad), 4)
            raw_y = round(-math.sin(pov_rad), 4)
        else:
            raw_x = -joyX
            raw_y = -joyY

        # forward speed
        xSpeed = self.xspeedLimiter.calculate(raw_x) * drivetrain.kMaxSpeed

        # strafe speed
        ySpeed = self.yspeedLimiter.calculate(raw_y) * drivetrain.kMaxSpeed

        # rotation speed
        rot = (
            -self.rotLimiter.calculate(
                _deadband(self.controller.getRightX())
            )
            * drivetrain.kMaxAngularSpeed
        )

        # drive robot
        self.swerve.drive(xSpeed, ySpeed, rot, fieldRelative, self.getPeriod())

        # debug: commanded outputs, remove after tuning
        wpilib.SmartDashboard.putNumber("joy/xSpeed", xSpeed)
        wpilib.SmartDashboard.putNumber("joy/ySpeed", ySpeed)
        wpilib.SmartDashboard.putNumber("joy/rot", rot)



