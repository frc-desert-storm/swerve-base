package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Units;

import static edu.wpi.first.units.Units.Radian;
import static frc.robot.subsystems.SwerveDriveConstants.kCanBus;

public class SwerveDriveModule {


    private final TalonFX m_driveMotor;
    private final TalonFX m_steerMotor;

    private final CANcoder m_steerEncoder;

    private final PIDController pid = new PIDController(.3, 0, 0);

    public SwerveDriveModule(SwerveDriveConstants.SwerveModuleConstants constants, double driveGearRatio, double steerGearRatio, double wheelDiameterMeters) {


        m_driveMotor = new TalonFX(constants.DriveCanId, kCanBus);
        m_steerMotor = new TalonFX(constants.SteeringCanId, kCanBus);
        m_steerEncoder = new CANcoder(constants.EncoderCanId, kCanBus);
    }

    public Rotation2d getCurrentAngle(){
        return Rotation2d.fromRadians(m_steerEncoder.getAbsolutePosition().getValue().in(Radian));
    }

    public void setDesiredState(SwerveModuleState desiredState){
        Rotation2d currentAngle = getCurrentAngle();

        SwerveModuleState optimized = new SwerveModuleState(
                desiredState.speedMetersPerSecond,
                desiredState.angle
        );

        optimized.optimize(currentAngle);

        double driveOutput = optimized.speedMetersPerSecond / SwerveDriveConstants.kMaxSpeedMetersPerSecond;

        m_driveMotor.set(driveOutput);
        
        m_steerMotor.set(pid.calculate(m_steerEncoder.getAbsolutePosition().getValue().in(Radian),optimized.angle.getRadians()));
    }
}
