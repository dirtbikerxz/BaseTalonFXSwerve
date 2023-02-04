package frc.robot.commands.swerve;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;

public class SwerveTeleopCommand extends CommandBase {

    private final SwerveDriveSubsystem swerveDrive;
    private final DoubleSupplier pxSupplier;
    private final DoubleSupplier pySupplier;
    private final DoubleSupplier pomegaSupplier;

    public SwerveTeleopCommand(SwerveDriveSubsystem swerveDrive,
                               DoubleSupplier pxSupplier,
                               DoubleSupplier pySupplier,
                               DoubleSupplier pomegaSupplier) {

        this.swerveDrive = swerveDrive;
        this.pxSupplier = pxSupplier;
        this.pySupplier = pySupplier;
        this.pomegaSupplier = pomegaSupplier;

        addRequirements(swerveDrive);
    }

    @Override
    public void execute() {
        swerveDrive.drive(
                pxSupplier.getAsDouble(),
                pySupplier.getAsDouble(),
                pomegaSupplier.getAsDouble());
    }
}