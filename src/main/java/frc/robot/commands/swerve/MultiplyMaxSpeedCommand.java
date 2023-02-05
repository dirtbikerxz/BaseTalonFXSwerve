package frc.robot.commands.swerve;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;

public class MultiplyMaxSpeedCommand extends InstantCommand {

    public MultiplyMaxSpeedCommand(SwerveDriveSubsystem swerveDrive, double factor) {
        super(() -> swerveDrive.multiplyMaxSpeeds(factor), swerveDrive);
    }
}
