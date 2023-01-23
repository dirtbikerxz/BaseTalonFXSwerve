package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;

public class TeleopSwerve extends CommandBase {    

    private SwerveDriveSubsystem swerveDrive;    
    private Supplier<ChassisSpeeds> speedSupplier;
    private BooleanSupplier fieldRelativeSupplier;

    public TeleopSwerve(SwerveDriveSubsystem swerveDrive, Supplier<ChassisSpeeds> speedSupplier, BooleanSupplier fieldRelativeSupplier) {

        this.swerveDrive = swerveDrive;
        this.speedSupplier = speedSupplier;
        this.fieldRelativeSupplier = fieldRelativeSupplier;

        addRequirements(swerveDrive);
    }

    @Override
    public void execute() {
        swerveDrive.drive(
            speedSupplier.get(),
            fieldRelativeSupplier.getAsBoolean());
    }
}