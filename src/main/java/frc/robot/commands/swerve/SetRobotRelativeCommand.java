package frc.robot.commands.swerve;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;

public class SetRobotRelativeCommand extends InstantCommand {

    public SetRobotRelativeCommand(SwerveDriveSubsystem swerveDrive, boolean robotRelative) {
        super(() -> swerveDrive.setRobotRelative(robotRelative), swerveDrive);
    }
}
