package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.SwerveTest;

public class DiagnoseDrive extends SequentialCommandGroup {

    public DiagnoseDrive(SwerveTest swerve) {
        super(
                new TestSwerveDriveMotors(swerve, 0, 1.0).withTimeout(1.0),
                new TestSwerveDriveMotors(swerve, 1, 1.0).withTimeout(1.0),
                new TestSwerveDriveMotors(swerve, 2, 1.0).withTimeout(1.0),
                new TestSwerveDriveMotors(swerve, 3, 1.0).withTimeout(1.0)
        );
    }
}
