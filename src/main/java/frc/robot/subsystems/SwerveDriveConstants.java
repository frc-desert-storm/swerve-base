package frc.robot.subsystems;

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
}
