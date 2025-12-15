package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SwerveDrive extends SubsystemBase {

    private final SwerveDriveKinematics m_swerveDriveKinematics = SwerveDriveConstants.kDriveKinematics;

    private final Pigeon2 pigeon = new Pigeon2(10,SwerveDriveConstants.kCanBus);

    private final SwerveDriveModule m_frontLeft = new SwerveDriveModule(
            SwerveDriveConstants.SwerveModuleConstants.FrontLeft
    );

    private final SwerveDriveModule m_frontRight = new SwerveDriveModule(
            SwerveDriveConstants.SwerveModuleConstants.FrontRight
    );

    private final SwerveDriveModule m_backLeft = new SwerveDriveModule(
            SwerveDriveConstants.SwerveModuleConstants.BackLeft
    );

    private final SwerveDriveModule m_backRight = new SwerveDriveModule(
            SwerveDriveConstants.SwerveModuleConstants.BackRight
    );

    private final SwerveDrivePoseEstimator m_odometry = new SwerveDrivePoseEstimator(
        m_swerveDriveKinematics, pigeon.getRotation2d(),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backLeft.getPosition(),
            m_backRight.getPosition()
        }, new Pose2d(0, 0, new Rotation2d()));
  
    public Pose2d m_pose = new Pose2d();
    public Field2d m_field = new Field2d();

    public Vision m_vision = new Vision();

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
    }
    
    @Override
    public void periodic() {
        m_pose = m_odometry.update(
            pigeon.getRotation2d(),
            new SwerveModulePosition[] {
                m_frontLeft.getPosition(),
                m_frontRight.getPosition(),
                m_backLeft.getPosition(),
                m_backRight.getPosition()
            }
        );
        m_odometry.addVisionMeasurement(m_vision.getRobotPose().toPose2d(),m_vision.getLatestTimestamp());
        m_field.setRobotPose(m_pose);
        SmartDashboard.putData(m_field);
    }
    
    public void xStance(){
        SwerveModuleState frontLeftState = new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45));
        SwerveModuleState frontRightState = new SwerveModuleState(0.0, Rotation2d.fromDegrees(45));
        SwerveModuleState backLeftState = new SwerveModuleState(0.0, Rotation2d.fromDegrees(45));
        SwerveModuleState backRightState = new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45));

        m_frontLeft.setDesiredState(frontLeftState);
        m_frontRight.setDesiredState(frontRightState);
        m_backLeft.setDesiredState(backLeftState);
        m_backRight.setDesiredState(backRightState);
    }

    public void zeroStance(){
        SwerveModuleState zero = new SwerveModuleState(0.0, Rotation2d.fromDegrees(0));
        
        m_frontLeft.setDesiredState(zero);
        m_frontRight.setDesiredState(zero);
        m_backLeft.setDesiredState(zero);
        m_backRight.setDesiredState(zero);
    }
}
