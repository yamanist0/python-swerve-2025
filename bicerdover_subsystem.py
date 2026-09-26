import rev
import wpilib
import wpimath.controller

# can ids
BICER_DOVER_ID = 43
INDIRIR_DOVER_ID = 44
DONME_DOLAP_ID = 45

# bicerdover voltage
BICERDOVER_RUN_VOLTAGE = 5.0

# donmedolap voltages
DONMEDOLAP_SLOW_VOLTAGE = 2.0
DONMEDOLAP_REVERSE_VOLTAGE = -2.0

# indirirdover pid gains
INDIRIRDOVER_KP = 4.0
INDIRIRDOVER_KI = 0.0
INDIRIRDOVER_KD = 0.1
INDIRIRDOVER_MAX_VOLTAGE = 6.0

# indirirdover positions (in rotations, 0-1 range)
INDIRIRDOVER_UP_POSITION = 8.0
INDIRIRDOVER_DOWN_POSITION = 0.5

# current limit for all motors
CURRENT_LIMIT = 40


# bicerdover subsystem: bicerdover + indirirdover (pid) + donmedolap
class BicerdoverSubsystem:
    def __init__(self) -> None:
        # bicerdover motor (id 43)
        self.bicerdover_motor = rev.SparkMax(BICER_DOVER_ID, rev.SparkMax.MotorType.kBrushless)
        bicerdoverConfig = rev.SparkMaxConfig()
        bicerdoverConfig.smartCurrentLimit(CURRENT_LIMIT)
        self.bicerdover_motor.configure(
            bicerdoverConfig,
            rev.ResetMode.kResetSafeParameters,
            rev.PersistMode.kPersistParameters,
        )

        # indirirdover motor (id 44)
        self.indirirdover_motor = rev.SparkMax(INDIRIR_DOVER_ID, rev.SparkMax.MotorType.kBrushless)
        indirirdoverConfig = rev.SparkMaxConfig()
        indirirdoverConfig.smartCurrentLimit(CURRENT_LIMIT)
        self.indirirdover_motor.configure(
            indirirdoverConfig,
            rev.ResetMode.kResetSafeParameters,
            rev.PersistMode.kPersistParameters,
        )

        # donmedolap motor (id 45)
        self.donmedolap_motor = rev.SparkMax(DONME_DOLAP_ID, rev.SparkMax.MotorType.kBrushless)
        donmedolapConfig = rev.SparkMaxConfig()
        donmedolapConfig.smartCurrentLimit(CURRENT_LIMIT)
        self.donmedolap_motor.configure(
            donmedolapConfig,
            rev.ResetMode.kResetSafeParameters,
            rev.PersistMode.kPersistParameters,
        )

        # indirirdover pid controller
        self.indirirdover_pid = wpimath.controller.PIDController(
            INDIRIRDOVER_KP, INDIRIRDOVER_KI, INDIRIRDOVER_KD
        )
        self.indirirdover_pid.setTolerance(0.02)
        self.indirirdover_pid_enabled = False

    # bicerdover

    def bicerdover_run(self) -> None:
        # run bicerdover mechanism
        self.bicerdover_motor.setVoltage(BICERDOVER_RUN_VOLTAGE)

    def bicerdover_stop(self) -> None:
        # stop bicerdover mechanism
        self.bicerdover_motor.setVoltage(0)

    # donmedolap

    def donmedolap_run(self) -> None:
        # run donmedolap forward at slow speed
        self.donmedolap_motor.setVoltage(DONMEDOLAP_SLOW_VOLTAGE)

    def donmedolap_reverse(self) -> None:
        # run donmedolap in reverse at slow speed
        self.donmedolap_motor.setVoltage(DONMEDOLAP_REVERSE_VOLTAGE)

    def donmedolap_stop(self) -> None:
        # stop donmedolap
        self.donmedolap_motor.setVoltage(0)

    # indirirdover (pid position control)

    def set_indirirdover_target(self, position: float) -> None:
        # set indirirdover target position and enable pid
        self.indirirdover_pid.setSetpoint(position)
        self.indirirdover_pid_enabled = True

    def stop_indirirdover(self) -> None:
        # disable pid and stop indirirdover motor
        self.indirirdover_pid_enabled = False
        self.indirirdover_motor.setVoltage(0)

    def is_indirirdover_at_target(self) -> bool:
        # check if indirirdover reached the target position
        return self.indirirdover_pid.atSetpoint()

    def update_indirirdover_pid(self) -> None:
        # call this every robot period to run the pid loop
        if not self.indirirdover_pid_enabled:
            return
        current = self.indirirdover_motor.getEncoder().getPosition()
        output = self.indirirdover_pid.calculate(current)
        # clamp output to safe voltage range
        output = max(-INDIRIRDOVER_MAX_VOLTAGE, min(INDIRIRDOVER_MAX_VOLTAGE, output))
        self.indirirdover_motor.setVoltage(output)

    # combined commands

    def full_intake(self) -> None:
        # indirirdover goes to up position + bicerdover runs
        self.set_indirirdover_target(INDIRIRDOVER_UP_POSITION)
        self.bicerdover_run()

    def stop_all(self) -> None:
        # stop all motors in this subsystem
        print("Stopping all motors")
        self.bicerdover_stop()
        self.stop_indirirdover()
        self.donmedolap_stop()

    def update_telemetry(self) -> None:
        # publish encoder values to smartdashboard
        wpilib.SmartDashboard.putNumber(
            "Bicerdover Encoder",
            self.bicerdover_motor.getEncoder().getPosition(),
        )
        wpilib.SmartDashboard.putNumber(
            "Indirirdover Encoder",
            self.indirirdover_motor.getEncoder().getPosition(),
        )
        wpilib.SmartDashboard.putNumber(
            "Indirirdover Target",
            self.indirirdover_pid.getSetpoint(),
        )
        wpilib.SmartDashboard.putBoolean(
            "Indirirdover At Target",
            self.is_indirirdover_at_target(),
        )
