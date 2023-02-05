package frc.robot.commands.swerve;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;

public class SwerveOrbitToggleCommand extends InstantCommand {

    public SwerveOrbitToggleCommand(SwerveDriveSubsystem swerveDrive) {
        super(swerveDrive::toggleKinematics, swerveDrive);
    }
}
