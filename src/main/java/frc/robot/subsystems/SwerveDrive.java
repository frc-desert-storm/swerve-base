package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveDrive extends SubsystemBase {

    private final SwerveDriveKinematics m_swerveDriveKinematics = SwerveDriveConstants.kDriveKinematics;

    private final Pigeon2 pigeon = new Pigeon2(10,SwerveDriveConstants.kCanBus);

    private final SwerveDriveModule m_frontLeft = new SwerveDriveModule(
            SwerveDriveConstants.SwerveModuleConstants.FrontLeft,
            SwerveDriveConstants.kDriveGearRatio,
            SwerveDriveConstants.kSteerGearRatio,
            SwerveDriveConstants.kWheelDiameterMeters
    );

    private final SwerveDriveModule m_frontRight = new SwerveDriveModule(
            SwerveDriveConstants.SwerveModuleConstants.FrontRight,
            SwerveDriveConstants.kDriveGearRatio,
            SwerveDriveConstants.kSteerGearRatio,
            SwerveDriveConstants.kWheelDiameterMeters
    );

    private final SwerveDriveModule m_backLeft = new SwerveDriveModule(
            SwerveDriveConstants.SwerveModuleConstants.BackLeft,
            SwerveDriveConstants.kDriveGearRatio,
            SwerveDriveConstants.kSteerGearRatio,
            SwerveDriveConstants.kWheelDiameterMeters
    );

    private final SwerveDriveModule m_backRight = new SwerveDriveModule(
            SwerveDriveConstants.SwerveModuleConstants.BackRight,
            SwerveDriveConstants.kDriveGearRatio,
            SwerveDriveConstants.kSteerGearRatio,
            SwerveDriveConstants.kWheelDiameterMeters
    );

    public void drive(double forward, double strafe, double rotation, Boolean fieldRelative) {

        double forwardMetersPerSecond = forward * SwerveDriveConstants.kMaxSpeedMetersPerSecond;
        double strafeMetersPerSecond = strafe * SwerveDriveConstants.kMaxSpeedMetersPerSecond;
        double rotationsRadiansPerSecond = rotation * SwerveDriveConstants.kMaxAngularSpeedRadiansPerSecond;
        
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(forwardMetersPerSecond, strafeMetersPerSecond, rotationsRadiansPerSecond);
        if(fieldRelative) chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds, pigeon.getRotation2d());
        // Convert to individual module states
        SwerveModuleState[] states = m_swerveDriveKinematics.toSwerveModuleStates(chassisSpeeds);

        // !IMPORTANT m_swerveDriveKinematics.toSwerveModuleStates(chassisSpeeds) ALWAYS
        // returns the states in FL, FR, BL, BR order because this the order we set them
        // in our constants
        m_frontLeft.setDesiredState(states[0]);
        m_frontRight.setDesiredState(states[1]);
        m_backLeft.setDesiredState(states[2]);
        m_backRight.setDesiredState(states[3]);

        pigeon.setYaw(0);
    }

    public void xStance(){
        SwerveModuleState frontLeftState = new SwerveModuleState(0.0, Rotation2d.fromDegrees(0));
        SwerveModuleState frontRightState = new SwerveModuleState(0.0, Rotation2d.fromDegrees(0));
        SwerveModuleState backLeftState = new SwerveModuleState(0.0, Rotation2d.fromDegrees(0));
        SwerveModuleState backRightState = new SwerveModuleState(0.0, Rotation2d.fromDegrees(0));

        m_frontLeft.setDesiredState(frontLeftState);
        m_frontRight.setDesiredState(frontRightState);
        m_backLeft.setDesiredState(backLeftState);
        m_backRight.setDesiredState(backRightState);
    }

    public void zeroStance(){
        SwerveModuleState zeroState = new SwerveModuleState(0.0, Rotation2d.fromDegrees(0));

        m_frontLeft.setDesiredState(zeroState);
        m_frontRight.setDesiredState(zeroState);
        m_backLeft.setDesiredState(zeroState);
        m_backRight.setDesiredState(zeroState);
        
    }
}
