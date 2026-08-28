import math
import wpilib
import rev
import navx
import commands2
if not hasattr(commands2, "CommandBase"):
    commands2.CommandBase = commands2.Command

from wpimath import applyDeadband
from wpimath.geometry import Rotation2d, Translation2d
from wpimath.kinematics import SwerveModuleState
from swervepy import u
import swervepy.impl
import swervepy.subsystem
import swervepy.abstract.sensor

# ==============================================================================
# 0. JİROSKOP TANIMI (NavX Gyro)
# ==============================================================================

class NavXGyro(swervepy.abstract.sensor.Gyro):
    def __init__(self, port=wpilib.SPI.Port.kMXP, invert=False):
        super().__init__()
        self._navx = navx.AHRS(port)
        self.invert = invert
        wpilib.SmartDashboard.putData("NavX Gyro", self)

    def zero_heading(self):
        self._navx.reset()

    @property
    def heading(self) -> Rotation2d:
        yaw = self._navx.getYaw()
        if self.invert:
            yaw = -yaw
        return Rotation2d.fromDegrees(yaw)

# ==============================================================================
# 1. FİZİKSEL SÜRÜCÜ VE DÖNÜŞ PARAMETRELERİ (physicalproperties & pidfproperties)
# ==============================================================================

# Sürüş (Drive) Motor Parametreleri
drive_params = swervepy.impl.NEOCoaxialDriveComponent.Parameters(
    wheel_circumference=4 * math.pi * u.inch,  # 4 inç tekerlek çapı
    gear_ratio=6.75 / 1,                       # Dişli oranı (Drive)
    max_speed=4.5 * (u.m / u.s),                # Maksimum hız
    open_loop_ramp_rate=0.25,                  # Ramp Rate
    closed_loop_ramp_rate=0.25,
    continuous_current_limit=40,               # Akım sınırı (40A)
    peak_current_limit=40,
    neutral_mode=rev.CANSparkMax.IdleMode.kBrake, # FREN MODU
    kP=0.0020645, kI=0, kD=0, kS=0, kV=0, kA=0, # PID değerleri
    invert_motor=False,                        # Motor yönü
)

# Dönüş (Azimuth/Angle) Motor Parametreleri
azimuth_params = swervepy.impl.NEOCoaxialAzimuthComponent.Parameters(
    gear_ratio=12.8 / 1,                       # Dişli oranı (Angle)
    max_angular_velocity=11.5 * (u.rad / u.s), # Maksimum açısal hız
    ramp_rate=0.25,                            # Ramp Rate
    continuous_current_limit=20,               # Akım sınırı (20A)
    peak_current_limit=20,
    neutral_mode=rev.CANSparkMax.IdleMode.kBrake,
    kP=0.0020645, kI=0, kD=0,                  # PID değerleri
    invert_motor=False,                        # Motor yönü
)

# ==============================================================================
# 2. MODÜLLERİN BİLEŞENLERİ VE KONUMLARI (modules/*.json)
# ==============================================================================

# --- Sol Ön (Front Left) Modül ---
fl_drive = swervepy.impl.NEOCoaxialDriveComponent(21, drive_params)
fl_encoder = swervepy.impl.AbsoluteCANCoder(15)
fl_offset = Rotation2d.fromDegrees(0.78)
fl_azimuth = swervepy.impl.NEOCoaxialAzimuthComponent(22, fl_offset, azimuth_params, fl_encoder)
front_left = swervepy.impl.CoaxialSwerveModule(
    drive=fl_drive,
    azimuth=fl_azimuth,
    placement=Translation2d(9.84 * 0.0254, 9.84 * 0.0254)
)

# --- Sağ Ön (Front Right) Modül ---
fr_drive = swervepy.impl.NEOCoaxialDriveComponent(27, drive_params)
fr_encoder = swervepy.impl.AbsoluteCANCoder(17)
fr_offset = Rotation2d.fromDegrees(0.76)
fr_azimuth = swervepy.impl.NEOCoaxialAzimuthComponent(28, fr_offset, azimuth_params, fr_encoder)
front_right = swervepy.impl.CoaxialSwerveModule(
    drive=fr_drive,
    azimuth=fr_azimuth,
    placement=Translation2d(9.84 * 0.0254, -9.84 * 0.0254)
)

# --- Sol Arka (Back Left) Modül ---
bl_drive = swervepy.impl.NEOCoaxialDriveComponent(23, drive_params)
bl_encoder = swervepy.impl.AbsoluteCANCoder(16)
bl_offset = Rotation2d.fromDegrees(0.49)
bl_azimuth = swervepy.impl.NEOCoaxialAzimuthComponent(24, bl_offset, azimuth_params, bl_encoder)
back_left = swervepy.impl.CoaxialSwerveModule(
    drive=bl_drive,
    azimuth=bl_azimuth,
    placement=Translation2d(-9.84 * 0.0254, 9.84 * 0.0254)
)

# --- Sağ Arka (Back Right) Modül ---
br_drive = swervepy.impl.NEOCoaxialDriveComponent(25, drive_params)
br_encoder = swervepy.impl.AbsoluteCANCoder(18)
br_offset = Rotation2d.fromDegrees(344.0)
br_azimuth = swervepy.impl.NEOCoaxialAzimuthComponent(26, br_offset, azimuth_params, br_encoder)
back_right = swervepy.impl.CoaxialSwerveModule(
    drive=br_drive,
    azimuth=br_azimuth,
    placement=Translation2d(-9.84 * 0.0254, -9.84 * 0.0254)
)

modules = [front_left, front_right, back_left, back_right]

# ==============================================================================
# 3. JİROSKOP VE SWERVE ALT SİSTEMİ
# ==============================================================================

gyro = NavXGyro(wpilib.SPI.Port.kMXP, invert=True)

MAX_VELOCITY = 4.5 * (u.m / u.s)
MAX_ANGULAR_VELOCITY = 11.5 * (u.rad / u.s)

swerve = swervepy.subsystem.SwerveDrive(modules, gyro, MAX_VELOCITY, MAX_ANGULAR_VELOCITY)

# ==============================================================================
# 4. ROBOT SINIFI VE OTONOM SIRALAMA (Autonomous & TeleOp)
# ==============================================================================

class MyRobot(commands2.TimedCommandRobot):
    def robotInit(self):
        # Teleop Sürüş Komutu
        self.joystick = wpilib.XboxController(0)
        self.teleop_command = swerve.teleop_command(
            translation=lambda: applyDeadband(-self.joystick.getLeftY(), 0.1),
            strafe=lambda: applyDeadband(-self.joystick.getLeftX(), 0.1),
            rotation=lambda: applyDeadband(-self.joystick.getRightX(), 0.1),
            field_relative=True,
            open_loop=True,
        )
        swerve.setDefaultCommand(self.teleop_command)

        # Sıfır durum nesnesi (4 modül için 0 m/s hız ve sabit 0 derece açı)
        zero_states = (
            SwerveModuleState(0, Rotation2d(0)),
            SwerveModuleState(0, Rotation2d(0)),
            SwerveModuleState(0, Rotation2d(0)),
            SwerveModuleState(0, Rotation2d(0)),
        )

        # Otonom Komut Dizisi:
        # 1. Aşama: 1 saniye boyunca yavaşça ileri sürüş (0.5 m/s)
        drive_forward = commands2.RunCommand(
            lambda: swerve.drive(Translation2d(0.5, 0), 0, field_relative=False, open_loop=True),
            swerve
        ).withTimeout(1.0)

        # 2. Aşama: Havada testlerde titreşimi engellemek için 2 saniye boyunca sabit 0 açısı ve 0 hız vermek
        stop_during_pause = commands2.RunCommand(
            lambda: swerve.desire_module_states(zero_states, open_loop=True, rotate_in_place=True),
            swerve
        ).withTimeout(2.0)

        # 3. Aşama: 1 saniyede 360 derece dönüş (2*pi rad/s ≈ 360°/s)
        rotate_360 = commands2.RunCommand(
            lambda: swerve.drive(Translation2d(0, 0), 2 * math.pi, field_relative=False, open_loop=True),
            swerve
        ).withTimeout(1.0)

        # 4. Aşama: Robotu tamamen durdurma
        stop_robot = commands2.InstantCommand(
            lambda: swerve.desire_module_states(zero_states, open_loop=True, rotate_in_place=True),
            swerve
        )

        # Komutları sırayla çalıştıracak otonom grubu
        self.auto_command = commands2.SequentialCommandGroup(
            drive_forward,
            stop_during_pause,
            rotate_360,
            stop_robot
        )

    def autonomousInit(self):
        if self.auto_command:
            self.auto_command.schedule()

    def teleopInit(self):
        if self.auto_command:
            self.auto_command.cancel()

if __name__ == "__main__":
    wpilib.run(MyRobot)