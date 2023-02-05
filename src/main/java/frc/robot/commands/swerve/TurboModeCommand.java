package frc.robot.commands.swerve;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;

public class TurboModeCommand extends InstantCommand {

    public TurboModeCommand(SwerveDriveSubsystem swerveDrive, boolean turbo) {
        super(() -> swerveDrive.setTurboMode(true), swerveDrive);
    }
}
