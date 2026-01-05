package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Units;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.Radian;
import static frc.robot.subsystems.swerve.SwerveConstants.*;

public class Module {
    private final ModuleIO io;
    private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
    private final int index;
  
    private final SimpleMotorFeedforward steerFF = new SimpleMotorFeedforward(kSteerS, kSteerV, kSteerA);
    private final PIDController steerPID = new PIDController(kSteerP, 0, kSteerD);
  
    private final SimpleMotorFeedforward driveFF = new SimpleMotorFeedforward(kDriveS, kDriveV, kDriveA);
    private final PIDController drivePID = new PIDController(kDriveP, 0, kDriveD);
    
    private Rotation2d angleSetpoint = null;
    private Double speedSetpoint = null;
    private SwerveModulePosition odometryPosition = new SwerveModulePosition();
    

    public Module(ModuleIO io, int index) {
      this.io = io;
      this.index = index;
      
      steerPID.enableContinuousInput(-Math.PI, Math.PI);
    }
    
    public void periodic() {
      io.updateInputs(inputs);
      Logger.processInputs("Drive/Module" + index, inputs);
      
      if (angleSetpoint != null) {
        io.setSteerVoltage(
            steerPID.calculate(getAngle().getRadians(), angleSetpoint.getRadians()));
        
        if (speedSetpoint != null) {
          // Drives slower when the wheel is away from the setpoint
          double adjustSpeedSetpoint = speedSetpoint * Math.cos(steerPID.getError());
          
          // Run drive controller
          double velocityRadPerSec = adjustSpeedSetpoint / kWheelRadiusMeters;
          io.setDriveVoltage(
              drivePID.calculate(velocityRadPerSec)
                  + driveFF.calculateWithVelocities(inputs.driveVelocityRadPerSec, velocityRadPerSec));
        }
      }
      
      odometryPosition = new SwerveModulePosition(
          inputs.drivePositionRad * kWheelRadiusMeters,
          getAngle()
      );
    }
    
    public void runSetpoint(SwerveModuleState state){
      state.optimize(getAngle());
      
      angleSetpoint = state.angle;
      speedSetpoint = state.speedMetersPerSecond;
    }
  
    public void stop() {
      io.setSteerVoltage(0.0);
      io.setDriveVoltage(0.0);
      
      angleSetpoint = null;
      speedSetpoint = null;
    }
    
    public Rotation2d getAngle() {
      return inputs.steerPosition;
    }
    
    public SwerveModulePosition getPosition() {
      return odometryPosition;
    }
    
    public SwerveModuleState getState() {
      return new SwerveModuleState(
          inputs.driveVelocityRadPerSec * kWheelRadiusMeters,
          getAngle()
      );
    }
}
