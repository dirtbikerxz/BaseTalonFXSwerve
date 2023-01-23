package frc.robot.commands;

import frc.robot.subsystems.SwerveConfig;
import frc.robot.subsystems.SwerveDriveSubsystem;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TeleopSwerve extends CommandBase {    

    public static final double stickDeadband = 0.1;

    private SwerveDriveSubsystem s_Swerve;    
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;

    public TeleopSwerve(SwerveDriveSubsystem s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/
        double translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), stickDeadband);
        double strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), stickDeadband);
        double rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), stickDeadband);

        /* Drive */
        s_Swerve.drive(
            translationVal * SwerveConfig.maxSpeed,
            strafeVal * SwerveConfig.maxSpeed,
            rotationVal * SwerveConfig.maxAngularVelocity, 
            !robotCentricSup.getAsBoolean());
    }
}