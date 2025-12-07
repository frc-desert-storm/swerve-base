package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDrive;

import java.util.function.DoubleSupplier;

public class SwerveZeroCommand extends Command {

    private final SwerveDrive m_swerveDrive;

    public SwerveZeroCommand(SwerveDrive swerveDrive) {
        m_swerveDrive = swerveDrive;
    }

    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        super.end(interrupted);
    }

    @Override
    public void execute() {
        m_swerveDrive.zeroStance();
    }

    @Override
    public void initialize() {
        // TODO Auto-generated method stub
        super.initialize();
    }

    @Override
    public boolean isFinished() {
        // TODO Auto-generated method stub
        return super.isFinished();
    }

}