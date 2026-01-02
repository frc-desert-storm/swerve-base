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