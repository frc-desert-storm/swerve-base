package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface ModuleIO {
  @AutoLog
  public static class ModuleIOInputs {
    // Drive
    public double drivePositionRad = 0.0;
    public double driveVelocityRadPerSec = 0.0;
    public double driveAppliedVolts = 0.0;
    public double driveCurrentAmps = 0.0;
    
    // Steer
    public Rotation2d steerPositionAbsolute = Rotation2d.kZero;
    public Rotation2d steerPosition = Rotation2d.kZero;
    public double steerVelocityRadPerSec = 0.0;
    public double steerAppliedVolts = 0.0;
    public double steerCurrentAmps = 0.0;
  }
  
  
  public default void updateInputs(ModuleIOInputs inputs) {}
  
  public default void setDriveVoltage(double volts) {}
  
  public default void setSteerVoltage(double volts) {}
}
