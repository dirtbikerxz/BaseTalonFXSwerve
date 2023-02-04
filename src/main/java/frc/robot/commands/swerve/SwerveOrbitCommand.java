package frc.robot.commands.swerve;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;

import static frc.robot.subsystems.swerve.SwerveConfig.defaultKinematics;
import static frc.robot.subsystems.swerve.SwerveConfig.orbitKinematics;

public class SwerveOrbitCommand extends InstantCommand {

    public SwerveOrbitCommand(SwerveDriveSubsystem swerveDrive, boolean orbit) {
        super(() -> swerveDrive.setKinematics(orbit ? orbitKinematics : defaultKinematics), swerveDrive);
    }
}
