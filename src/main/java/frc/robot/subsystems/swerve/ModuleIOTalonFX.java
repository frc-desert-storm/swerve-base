package frc.robot.subsystems.swerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class ModuleIOTalonFX implements ModuleIO {
  private final TalonFX driveMotor;
  private final TalonFX steerMotor;
  private final CANcoder steerEncoder;
  
  private final StatusSignal<Angle> drivePosition;
  private final StatusSignal<AngularVelocity> driveVelocity;
  private final StatusSignal<Voltage> driveAppliedVolts;
  private final StatusSignal<Current> driveCurrent;
  
  private final StatusSignal<Angle> steerAbsolutePosition;
  private final StatusSignal<Angle> steerPosition;
  private final StatusSignal<AngularVelocity> steerVelocity;
  private final StatusSignal<Voltage> steerAppliedVolts;
  private final StatusSignal<Current> steerCurrent;
  
  public ModuleIOTalonFX(SwerveConstants.ModuleConfig config) {
    driveMotor = new TalonFX(config.DriveCanId, SwerveConstants.kCanBus);
    steerMotor = new TalonFX(config.SteeringCanId, SwerveConstants.kCanBus);
    steerEncoder = new CANcoder(config.EncoderCanId, SwerveConstants.kCanBus);
    
    var driveConfig = new TalonFXConfiguration();
    var driveMotorOutputConfig = new MotorOutputConfigs();
    driveConfig.CurrentLimits.SupplyCurrentLimit = 40.0;
    driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    driveMotorOutputConfig.NeutralMode = NeutralModeValue.Brake; // idk why this needs a extra config
    driveMotor.getConfigurator().apply(driveConfig);
    driveMotor.getConfigurator().apply(driveMotorOutputConfig);
    
    var steerConfig = new TalonFXConfiguration();
    var steerMotorOutputConfig = new MotorOutputConfigs();
    steerConfig.CurrentLimits.SupplyCurrentLimit = 30.0;
    steerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    steerMotorOutputConfig.NeutralMode = NeutralModeValue.Brake;
    steerMotor.getConfigurator().apply(steerConfig);
    steerMotor.getConfigurator().apply(steerMotorOutputConfig);
    
    drivePosition = driveMotor.getPosition();
    driveVelocity = driveMotor.getVelocity();
    driveAppliedVolts = driveMotor.getMotorVoltage();
    driveCurrent = driveMotor.getStatorCurrent();
    
    steerAbsolutePosition = steerEncoder.getAbsolutePosition();
    
    steerPosition = steerMotor.getPosition();
    steerVelocity = steerMotor.getVelocity();
    steerAppliedVolts = steerMotor.getMotorVoltage();
    steerCurrent = steerMotor.getStatorCurrent();
  }
  
  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    BaseStatusSignal.refreshAll(
      drivePosition,
      driveVelocity,
      driveAppliedVolts,
      driveCurrent,
      steerAbsolutePosition,
      steerPosition,
      steerVelocity,
      steerAppliedVolts,
      steerCurrent
    );
    
    inputs.drivePositionRad =
        Units.rotationsToRadians(drivePosition.getValueAsDouble())
            / SwerveConstants.kDriveGearRatio;
    inputs.driveVelocityRadPerSec =
        Units.rotationsToRadians(driveVelocity.getValueAsDouble())
            / SwerveConstants.kDriveGearRatio;
    inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
    inputs.driveCurrentAmps = driveCurrent.getValueAsDouble();
    
    inputs.steerPositionAbsolute =
        Rotation2d.fromRotations(steerAbsolutePosition.getValueAsDouble());
    inputs.steerPosition =
        Rotation2d.fromRotations(steerPosition.getValueAsDouble() / SwerveConstants.kSteerGearRatio);
    inputs.steerVelocityRadPerSec =
        Units.rotationsToRadians(steerVelocity.getValueAsDouble()) / SwerveConstants.kSteerGearRatio;
    inputs.steerAppliedVolts = steerAppliedVolts.getValueAsDouble();
    inputs.steerCurrentAmps = steerCurrent.getValueAsDouble();
  }
  
  @Override
  public void setDriveVoltage(double volts) {
    driveMotor.setControl(new VoltageOut(volts));
  }
  
  @Override
  public void setSteerVoltage(double volts) {
    steerMotor.setControl(new VoltageOut(volts));
  }
}
