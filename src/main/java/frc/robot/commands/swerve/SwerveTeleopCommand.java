package frc.robot.commands.swerve;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.swerve.SwerveConfig;
import frc.robot.subsystems.swerve.SwerveDriveSubsystem;

public class SwerveTeleopCommand extends CommandBase {

    private SwerveDriveSubsystem swerveDrive;    
    private Supplier<ChassisSpeeds> speedSupplier;
    private BooleanSupplier highspeedSupplier;

    public SwerveTeleopCommand(SwerveDriveSubsystem swerveDrive, Supplier<ChassisSpeeds> speedSupplier, BooleanSupplier highspeedsupplier) {

        this.swerveDrive = swerveDrive;
        this.speedSupplier = speedSupplier;
        this.highspeedSupplier = highspeedsupplier;

        addRequirements(swerveDrive);
    }

    @Override
    public void execute() {
        ChassisSpeeds speeds=speedSupplier.get();
        double maxspeed;
        if (highspeedSupplier.getAsBoolean()) {
            maxspeed = SwerveConfig.maxSpeed;
        }
        else {
            maxspeed = SwerveConfig.maxSpeed / 2;
        }
        swerveDrive.drive(
            speeds.vxMetersPerSecond,
            speeds.vyMetersPerSecond,
            speeds.omegaRadiansPerSecond,
            maxspeed);
    }
}