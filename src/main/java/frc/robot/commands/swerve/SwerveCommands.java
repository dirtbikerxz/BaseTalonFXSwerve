package frc.robot.commands.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;

public class SwerveCommands {

    public static Command setTurboMode(SwerveDriveSubsystem drive, boolean turbo) {
        return new InstantCommand(() -> drive.setTurboMode(turbo), drive);
    }

    public static Command setRobotRelative(SwerveDriveSubsystem drive, boolean robotRelative) {
        return new InstantCommand(() -> drive.setRobotRelative(robotRelative), drive);
    }

    public static Command setOrbitMode(SwerveDriveSubsystem drive, boolean orbit) {
        return new InstantCommand(() -> drive.setOrbitMode(orbit), drive);
    }

    public static Command zeroGyro(SwerveDriveSubsystem drive) {
        return new InstantCommand(() -> drive.zeroGyro(), drive);
    }

    public static Command turnWheels(SwerveDriveSubsystem drive, double degrees) {

        final SwerveModuleState [] states = new SwerveModuleState[4];
        for (int i=0; i<states.length; i++) {
            states[i] = new SwerveModuleState(0.1, Rotation2d.fromDegrees(degrees));
        }

        return new InstantCommand(() -> drive.setModuleStates(states), drive);
    }
}
