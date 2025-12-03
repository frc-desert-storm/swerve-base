package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public class SwerveDriveConstants {
    // EVERYTHING IS METRIC
    public static final double kTrackWidth = Units.inchesToMeters(16.5);
    public static final double kWheelBase = Units.inchesToMeters(16.5);

    public static Translation2d kFrontLeftModulePosition = new Translation2d(+kWheelBase / 2, +kTrackWidth / 2);
    public static Translation2d kFrontRightModulePosition = new Translation2d(+kWheelBase / 2, -kTrackWidth / 2);
    public static Translation2d kBackLeftModulePosition = new Translation2d(-kWheelBase / 2, +kTrackWidth / 2);
    public static Translation2d kBackRightModulePosition = new Translation2d(-kWheelBase / 2, -kTrackWidth / 2);

    public static final SwerveDriveKinematics kDriveKinematics = 
        new SwerveDriveKinematics(
            kFrontLeftModulePosition,
            kFrontRightModulePosition,
            kBackLeftModulePosition,
            kBackRightModulePosition
        );

    public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;

    public static final double kDriveGearRatio = 6.12;
    public static final double kSteerGearRatio = 12.8;
    // Kraken X60 Free Speed RPM from documentation
    public static final double kMotorFreeSpeedRPM = 5800;


    private static final double maxSpeedPercentage = .8;

    public static final double kMaxSpeedMetersPerSecond =
            ((kMotorFreeSpeedRPM / kDriveGearRatio) * kWheelCircumferenceMeters / 60.0) * maxSpeedPercentage;

    public static final double kMaxAngularSpeedRadiansPerSecond =
            kMaxSpeedMetersPerSecond / Math.hypot(kWheelBase/2, kTrackWidth/2);

    public static final CANBus kCanBus = new CANBus("bob");

    public static enum SwerveModuleConstants {
        FrontLeft(11,12,13),
        FrontRight(14,15,16),
        BackLeft(18,17,19),
        BackRight(21,20,22);

        public final int DriveCanId;
        public final int SteeringCanId;
        public final int EncoderCanId;

        SwerveModuleConstants(int DriveCanId, int SteeringCanId, int EncoderCanId){
            this.DriveCanId = DriveCanId;
            this.SteeringCanId = SteeringCanId;
            this.EncoderCanId = EncoderCanId;
        }
    }
}
/*            case "Front Left":
                m_driveMotor = new TalonFX(11, "bob");
                m_steerMotor = new TalonFX(12, "bob");
                m_steerEncoder = new CANcoder(13, "bob");
                break;
            case "Front Right":
                m_driveMotor = new TalonFX(14, "bob");
                m_steerMotor = new TalonFX(15, "bob");
                m_steerEncoder = new CANcoder(16, "bob");
                break;
            case "Back Left":
                m_driveMotor = new TalonFX(18, "bob");
                m_steerMotor = new TalonFX(17, "bob");
                m_steerEncoder = new CANcoder(19, "bob");
                break;
            case "Back Right":
                m_driveMotor = new TalonFX(21, "bob");
                m_steerMotor = new TalonFX(20, "bob");
                m_steerEncoder = new CANcoder(22, "bob");
                break;
 */