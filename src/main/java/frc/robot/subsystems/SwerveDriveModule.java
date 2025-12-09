package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Units;

import static edu.wpi.first.units.Units.Radian;
import static frc.robot.subsystems.SwerveDriveConstants.kCanBus;

public class SwerveDriveModule {
    private final TalonFX m_driveMotor;
    private final TalonFX m_steerMotor;

    private final CANcoder m_steerEncoder;

    private final SimpleMotorFeedforward m_steerFeedforward = new SimpleMotorFeedforward(0, 0, 0);
    private final PIDController m_steerPid = new PIDController(.3, 0, 0);
  
    private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(0, 0, 0);
    private final PIDController m_drivePid = new PIDController(.3, 0, 0);

    public SwerveDriveModule(SwerveDriveConstants.SwerveModuleConstants constants) {
        m_driveMotor = new TalonFX(constants.DriveCanId, kCanBus);
        m_steerMotor = new TalonFX(constants.SteeringCanId, kCanBus);
        m_steerEncoder = new CANcoder(constants.EncoderCanId, kCanBus);
    }

    public Rotation2d getCurrentAngle(){
        return Rotation2d.fromRadians(m_steerEncoder.getAbsolutePosition().getValue().in(Radian));
    }
    
    public SwerveModuleState getState(){
        double speedMetersPerSecond = (m_driveMotor.getVelocity().getValue().in(Units.RPM) / 60.0)
                * SwerveDriveConstants.kWheelCircumferenceMeters
                * SwerveDriveConstants.kDriveGearRatio;

        Rotation2d angle = getCurrentAngle();

        return new SwerveModuleState(speedMetersPerSecond, angle);
    }

    public void setDesiredState(SwerveModuleState desiredState){
        SwerveModuleState optimized = new SwerveModuleState(
                desiredState.speedMetersPerSecond,
                desiredState.angle
        );
        
        SwerveModuleState currentState = getState();

        optimized.optimize(currentState.angle);
        
        double driveOutput = m_drivePid.calculate(currentState.speedMetersPerSecond, optimized.speedMetersPerSecond)
                + m_driveFeedforward.calculate(optimized.speedMetersPerSecond);

        m_driveMotor.set(driveOutput);
        
        double steerOutput = m_steerPid.calculate(currentState.angle.getRadians(), optimized.angle.getRadians())
                + m_steerFeedforward.calculate(optimized.angle.getRadians());
        
        m_steerMotor.set(m_steerPid.calculate(m_steerEncoder.getAbsolutePosition().getValue().in(Radian),optimized.angle.getRadians()));
    }
}
