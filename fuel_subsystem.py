import rev
import wpilib

# can ids
FEEDER_MOTOR_ID = 42
INTAKE_LAUNCHER_MOTOR_ID = 41

# current limits
FEEDER_MOTOR_CURRENT_LIMIT = 60
LAUNCHER_MOTOR_CURRENT_LIMIT = 60

# voltage constants
INTAKING_FEEDER_VOLTAGE = -12.0
INTAKING_INTAKE_VOLTAGE = 10.0
LAUNCHING_FEEDER_VOLTAGE = 9.0
LAUNCHING_LAUNCHER_VOLTAGE = 10.6
SPIN_UP_FEEDER_VOLTAGE = 6.0
SPIN_UP_SECONDS = 1.0


# fuel subsystem: feeder + intake/launcher rollers
class FuelSubsystem:
    def __init__(self) -> None:
        # motor 41 
        self.feeder = rev.SparkMax(INTAKE_LAUNCHER_MOTOR_ID, rev.SparkMax.MotorType.kBrushless)
        feederConfig = rev.SparkMaxConfig()
        feederConfig.inverted(False)
        feederConfig.smartCurrentLimit(FEEDER_MOTOR_CURRENT_LIMIT)
        self.feeder.configure(
            feederConfig,
            rev.ResetMode.kResetSafeParameters,
            rev.PersistMode.kPersistParameters,
        )

        # motor 42 
        self.launcher = rev.SparkMax(FEEDER_MOTOR_ID, rev.SparkMax.MotorType.kBrushless)
        launcherConfig = rev.SparkMaxConfig()
        launcherConfig.inverted(True)
        launcherConfig.smartCurrentLimit(LAUNCHER_MOTOR_CURRENT_LIMIT)
        self.launcher.configure(
            launcherConfig,
            rev.ResetMode.kResetSafeParameters,
            rev.PersistMode.kPersistParameters,
        )

    def intake(self) -> None:
        # run both rollers inward to collect game pieces
        self.feeder.setVoltage(INTAKING_FEEDER_VOLTAGE)
        self.launcher.setVoltage(INTAKING_INTAKE_VOLTAGE)

    def eject(self) -> None:
        # reverse both rollers to push game pieces out
        self.feeder.setVoltage(-INTAKING_FEEDER_VOLTAGE)
        self.launcher.setVoltage(-INTAKING_INTAKE_VOLTAGE)

    def launch(self) -> None:
        # run both rollers at launch speed
        self.feeder.setVoltage(LAUNCHING_FEEDER_VOLTAGE)
        self.launcher.setVoltage(LAUNCHING_LAUNCHER_VOLTAGE)

    def spin_up(self) -> None:
        # spin up rollers before launching
        self.feeder.setVoltage(SPIN_UP_FEEDER_VOLTAGE)
        self.launcher.setVoltage(LAUNCHING_LAUNCHER_VOLTAGE)

    def stop(self) -> None:
        # stop both motors
        self.feeder.set(0)
        self.launcher.set(0)
