import rev

# can id
KALDIRIR_DOVER_ID = 46

# voltage for up/down movement
KALDIRIRDOVER_VOLTAGE = 6.0

# current limit
CURRENT_LIMIT = 40


# kaldirirdover subsystem: lifter motor (up/down)
class KaldirirdoverSubsystem:
    def __init__(self) -> None:
        # lifter motor (id 46)
        self.motor = rev.SparkMax(KALDIRIR_DOVER_ID, rev.SparkMax.MotorType.kBrushless)
        config = rev.SparkMaxConfig()
        config.inverted(False)
        config.smartCurrentLimit(CURRENT_LIMIT)
        self.motor.configure(
            config,
            rev.ResetMode.kResetSafeParameters,
            rev.PersistMode.kPersistParameters,
        )

    def yukari(self) -> None:
        # move lifter up (positive voltage)
        self.motor.setVoltage(KALDIRIRDOVER_VOLTAGE)

    def asagi(self) -> None:
        # move lifter down (negative voltage)
        self.motor.setVoltage(-KALDIRIRDOVER_VOLTAGE)

    def stop(self) -> None:
        # stop lifter motor
        self.motor.setVoltage(0)
