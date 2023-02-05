package frc.robot.commands.swerve;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;

public class BalanceCommand extends CommandBase {

    private final SwerveDriveSubsystem swerveDrive;
    private boolean done;

    public BalanceCommand(SwerveDriveSubsystem swerveDrive) {

        this.swerveDrive = swerveDrive;

        addRequirements(swerveDrive);
    }

    @Override
    public void initialize() {
        done = false;
    }

    @Override
    public void execute() {

        double pitch = swerveDrive.getPitch();

        // TODO drive backwards and forwards to bring the pitch into a tolerable range

    }

    @Override
    public boolean isFinished() {
        return done;
    }
}
