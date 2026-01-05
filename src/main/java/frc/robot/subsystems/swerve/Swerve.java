package frc.robot.subsystems.swerve;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.swerve.gyro.GyroIO;
import frc.robot.subsystems.swerve.gyro.GyroIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

public class Swerve extends SubsystemBase {
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR
  
  private Rotation2d rawGyroRotation = new Rotation2d();
  private final SwerveModulePosition[] lastModulePositions = new SwerveModulePosition[]{
      new SwerveModulePosition(),
      new SwerveModulePosition(),
      new SwerveModulePosition(),
      new SwerveModulePosition()
  };
  private final SwerveDriveKinematics kinematics = SwerveConstants.kDriveKinematics;
  
  private SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(
          kinematics,
          rawGyroRotation,
          lastModulePositions,
          new Pose2d());
  
  
  private final SwerveDrivePoseEstimator odometry = new SwerveDrivePoseEstimator(
      kinematics, rawGyroRotation,
      lastModulePositions, new Pose2d(0, 0, new Rotation2d()));

  
  public Swerve(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO) {
    this.gyroIO = gyroIO;
    modules[0] = new Module(flModuleIO, 0);
    modules[1] = new Module(frModuleIO, 1);
    modules[2] = new Module(blModuleIO, 2);
    modules[3] = new Module(brModuleIO, 3);
  }
  
  @Override
  public void periodic() {
    Logger.processInputs("Drive/Gyro", gyroInputs);
    
    for (var module : modules) {
      module.periodic();
    }
    
    if (gyroInputs.connected) {
      rawGyroRotation = gyroInputs.yawPosition;
    }
    
    for (int i = 0; i < 4; i++) {
      lastModulePositions[i] = modules[i].getPosition();
    }
    
    poseEstimator.update(rawGyroRotation, lastModulePositions);
    odometry.update(rawGyroRotation, lastModulePositions);
  }
  
  public void drive(double forward, double strafe, double rotation, Boolean fieldRelative) {
    double forwardMetersPerSecond = forward * SwerveConstants.kMaxSpeedMetersPerSecond;
    double strafeMetersPerSecond = strafe * SwerveConstants.kMaxSpeedMetersPerSecond;
    double rotationsRadiansPerSecond = rotation * SwerveConstants.kMaxAngularSpeedRadiansPerSecond;
    
    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(forwardMetersPerSecond, strafeMetersPerSecond, rotationsRadiansPerSecond);
    if (fieldRelative) chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds, Rotation2d.fromRadians(gyroIO.getRotation()));
    
    runVelocity(chassisSpeeds);
  }
  
  public void runVelocity(ChassisSpeeds speeds){
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(
        setpointStates, SwerveConstants.kMaxSpeedMetersPerSecond);

    for (int i = 0; i < 4; i++) {
      modules[i].runSetpoint(setpointStates[i]);
    }
    
    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
  }
  
  public void xStance() {
    var states = new SwerveModuleState[4];
    states[0] = new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45));
    states[1] = new SwerveModuleState(0.0, Rotation2d.fromDegrees(45));
    states[2] = new SwerveModuleState(0.0, Rotation2d.fromDegrees(45));
    states[3] = new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45));
    
    for (int i = 0; i < 4; i++) {
      modules[i].runSetpoint(states[i]);
    }
  }
  
  public void zeroStance() {
    SwerveModuleState zero = new SwerveModuleState(0.0, Rotation2d.fromDegrees(0));
    
    for (int i = 0; i < 4; i++) {
      modules[i].runSetpoint(zero);
    }
  }
  
  
  public void addVisionMeasurement(Pose2d pose, double timestamp, Matrix<N3, N1> estimationStdDevs) {
    odometry.addVisionMeasurement(pose, timestamp, estimationStdDevs);
  }
}
