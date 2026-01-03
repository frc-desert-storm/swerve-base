// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.commands.SwerveXCommand;
import frc.robot.commands.SwerveZeroCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.swerve.ModuleIO;
import frc.robot.subsystems.swerve.ModuleIOTalonFX;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.subsystems.swerve.gyro.GyroIO;
import frc.robot.subsystems.swerve.gyro.GyroIOPigeon2;
import frc.robot.subsystems.vision.Vision;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final Swerve m_drivetrain;
  private final Vision m_vision;
  
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    if(Robot.isReal()){
      m_drivetrain =
          new Swerve(
              new GyroIOPigeon2(),
              new ModuleIOTalonFX(SwerveConstants.ModuleConfig.FrontLeft),
              new ModuleIOTalonFX(SwerveConstants.ModuleConfig.FrontRight),
              new ModuleIOTalonFX(SwerveConstants.ModuleConfig.BackLeft),
              new ModuleIOTalonFX(SwerveConstants.ModuleConfig.BackRight));
    }else {
      m_drivetrain =
          new Swerve(
              new GyroIO() {},
              new ModuleIO() {},
              new ModuleIO() {},
              new ModuleIO() {},
              new ModuleIO() {}); // TODO add sim modules
    }
    m_vision = new Vision(m_drivetrain::addVisionMeasurement);
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    m_drivetrain.setDefaultCommand(new SwerveDriveCommand(
            m_drivetrain,
            () -> MathUtil.applyDeadband(m_driverController.getLeftY() * 0.5,0.1), //TODO REMOVE the * 0.5
            () -> MathUtil.applyDeadband(m_driverController.getLeftX() * 0.5,0.1),
            () -> MathUtil.applyDeadband(m_driverController.getRightX() * 0.5,0.1)
    ));
    m_driverController.x().onTrue(new SwerveXCommand(m_drivetrain));
    m_driverController.b().onTrue(new SwerveZeroCommand(m_drivetrain));
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null;
  }
}