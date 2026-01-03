package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

public class SwerveConstants {
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
    public static final double kWheelRadiusMeters = kWheelDiameterMeters / 2;
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
    
    // Drive PID values
    public static final double kDriveP = 0.05;
    public static final double kDriveD = 0.0;
    // Drive Feedforward values
    public static final double kDriveS = 0.0;
    public static final double kDriveV = 0.0;
    public static final double kDriveA = 0.0;
    // Steering PID values
    public static final double kSteerP = 0.3;
    public static final double kSteerD = 0.0;
    // Steering Feedforward values
    public static final double kSteerS = 0.0;
    public static final double kSteerV = 0.0;
    public static final double kSteerA = 0.0;

    public static final CANBus kCanBus = new CANBus("bob");
    
    public static final int kPigeonCanId = 10;

    public enum ModuleConfig {
        FrontLeft(11,12,13),
        FrontRight(14,15,16),
        BackLeft(18,17,19),
        BackRight(21,20,22);

        public final int DriveCanId;
        public final int SteeringCanId;
        public final int EncoderCanId;

        ModuleConfig(int DriveCanId, int SteeringCanId, int EncoderCanId){
            this.DriveCanId = DriveCanId;
            this.SteeringCanId = SteeringCanId;
            this.EncoderCanId = EncoderCanId;
        }
    }
}