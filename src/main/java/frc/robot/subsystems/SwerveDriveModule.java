package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Units;

import static edu.wpi.first.units.Units.Radian;

public class SwerveDriveModule {
    private final String name;
    private final double driveGearRatio;
    private final double steerGearRatio;
    private final double wheelDiameterMeters;

    private final TalonFX m_driveMotor;
    private final TalonFX m_steerMotor;

    private final CANcoder m_steerEncoder;

    private static final double kTurnP = .2;

    public SwerveDriveModule(String name, double driveGearRatio, double steerGearRatio, double wheelDiameterMeters) {
        this.name = name;
        this.driveGearRatio = driveGearRatio;
        this.steerGearRatio = steerGearRatio;
        this.wheelDiameterMeters = wheelDiameterMeters;

        switch (name){
            case "Front Left":
                m_driveMotor = new TalonFX(11, "bob");
                m_steerMotor = new TalonFX(12, "bob");
                m_steerEncoder = new CANcoder(13, "bob");
                break;
            case "Front Right":
                m_driveMotor = new TalonFX(14, "bob");
                m_steerMotor = new TalonFX(15, "bob");
                m_steerEncoder = new CANcoder(16, "bob");
                break;
            case "Back Left":
                m_driveMotor = new TalonFX(18, "bob");
                m_steerMotor = new TalonFX(17, "bob");
                m_steerEncoder = new CANcoder(19, "bob");
                break;
            case "Back Right":
                m_driveMotor = new TalonFX(21, "bob");
                m_steerMotor = new TalonFX(20, "bob");
                m_steerEncoder = new CANcoder(22, "bob");
                break;
            default:
                m_driveMotor = new TalonFX(0);
                m_steerMotor = new TalonFX(0);
                m_steerEncoder = new CANcoder(0);
                break;
        }
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

        if(Math.abs(driveOutput) > 0.05){
            driveOutput = MathUtil.clamp(driveOutput, -1.0, 1.0);
        }else {
            driveOutput = 0.0;
        }

        m_driveMotor.set(driveOutput);

        double angleError = optimized.angle.minus(currentAngle).getRadians();
        double turnOutput = kTurnP * angleError;

        if(Math.abs(turnOutput) > 0.05){
            turnOutput = MathUtil.clamp(turnOutput, -1.0, 1.0);
        }else {
            turnOutput = 0.0;
        }

        m_steerMotor.set(turnOutput);
    }
}
