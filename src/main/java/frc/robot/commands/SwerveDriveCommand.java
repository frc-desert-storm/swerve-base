package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveDrive;

public class SwerveDriveCommand extends Command {

    private final SwerveDrive m_swerveDrive;
    private final DoubleSupplier m_forward;
    private final DoubleSupplier m_strafe;
    private final DoubleSupplier m_rotation;

    public SwerveDriveCommand(SwerveDrive swerveDrive, DoubleSupplier forward, DoubleSupplier strafe, DoubleSupplier rotation) {
        m_swerveDrive = swerveDrive;
        m_forward = forward;
        m_strafe = strafe;
        m_rotation = rotation;

        this.addRequirements(swerveDrive);
    }

    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        super.end(interrupted);
    }

    @Override
    public void execute() {

        double forward = m_forward.getAsDouble();
        double strafe = m_strafe.getAsDouble();
        double rotation = m_rotation.getAsDouble();

        m_swerveDrive.drive(forward, strafe, rotation);
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